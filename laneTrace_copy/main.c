/////////////////////////////////////////////////////////////////////////
//////////////////////// INCLUDE FILES //////////////////////////////////
/////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "car_lib.h"

#include <signal.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>

#include "util.h"

#include "display-kms.h"
#include "v4l2.h"
#include "vpe-common.h"
#include "drawing.h"
#include "input_cmd.h"
#include "exam_cv.h"
#include "laneDetection.h"
#include "passing_master.h"
#include "koo_driving.h"

/////////////////////////////////////////////////////////////////////////
//////////////////////// Defines ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

#define LIGHT_BEEP        // to test light and beep
#define POSITION_CONTROL  // to test postion control
#define SPEED_CONTROL     // to test speed control
#define SERVO_CONTROL     // to test servo control(steering & camera position)
#define LINE_TRACE        // to test line trace sensor
#define DISTANCE_SENSOR   // to test distance sensor

#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"

#define VPE_OUTPUT_W        320
#define VPE_OUTPUT_H        180

#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3)
#define VPE_OUTPUT_FORMAT       "bgr24"

#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')
#define OVERLAY_DISP_W          480
#define OVERLAY_DISP_H          272

#define TIME_TEXT_X             385 //320
#define TIME_TEXT_Y             260 //240
#define TIME_TEXT_COLOR         0xffffffff //while

#define DUMP_MSGQ_KEY           1020
#define DUMP_MSGQ_MSG_TYPE      0x02
/////////////////////////////////////////////////////////////////////////
//////////////////////// Functions //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

typedef enum {
    DUMP_NONE,
    DUMP_CMD,
    DUMP_READY,
    DUMP_WRITE_TO_FILE,
    DUMP_DONE
}DumpState;

// 2019.11.16 관형 추가
typedef enum {
    AUTO_DRIVE,
    BEFORE_PASSING_OVER, // 추월차로 미션 전의 임의의 미션
    PASSING_OVER_LEFT,
    PASSING_OVER_RIGHT,
    WAIT,
    PASSING_OVER_RETURN_LEFT,
    PASSING_OVER_RETURN_RIGHT,
    STOP
}MissionState;

typedef struct _DumpMsg{
    long type;
    int  state_msg;
}DumpMsg;

struct thr_data {
    struct display *disp;
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct buffer **input_bufs;

    double angle;
	signed short speed;
    //struct passing *passing; // 2019.11.16 관형 추가
    int distance; // 1번 적외선센서 거리값, 관형 추가
    char* direction = "NONE"; // 추월 차로 진행 방향, left or right
    char* yellow_stop_line = "NONE"; // 정지선 인식 변수, 관형 추가
    int white_stop_line = -1; // 정지선 인식 변수, 관형 추가

    MissionState mission_state;
    DumpState dump_state;
    unsigned char dump_img_data[VPE_OUTPUT_IMG_SIZE];

    int msgq_id;
    bool bfull_screen;
    bool bstream_start;
    pthread_t threads[3];
};
signed short real_speed = 0;
//int is_Traffic_Light = 0; //1 is traffic light mission 1 is left, 2 is right
//int passing_where = -1; // 1 is left 2 is right
//int passing = 0;

static int allocate_input_buffers(struct thr_data *data);
static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar);
static void draw_operatingtime(struct display *disp, uint32_t time);
double getSteeringWithLane(struct display *disp, struct buffer *cambuf);
void * capture_thread(void *arg);
void * capture_dump_thread(void *arg);
void * input_thread(void *arg);
double distance_calculate(double data);
double distance_sensor();
static void passing_master(struct display *disp, struct buffer *cambuf, void *arg); // 2019.11.16 관형 변경

static struct thr_data* pexam_data = NULL;
void signal_handler(int sig);

int main(int argc, char **argv)
{
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct thr_data tdata;
    int disp_argc = 3;
    char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"};
    int ret = 0;

    printf("laneTrace Start ~~~ pky\n");

    tdata.dump_state = DUMP_NONE;
    tdata.mission_state = AUTO_DRIVE; // mission state를 기본주행으로 default
    memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data)); // dump data를 0으로 채워서 초기화

    // VPE 하드웨어 사용을 위한 초기화
    vpe = vpe_open(); if(!vpe) return 1;
    // 이미지 입력 속성
    vpe->src.width  = CAPTURE_IMG_W;
    vpe->src.height = CAPTURE_IMG_H;
    describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);
    // 이미지 출력 속성
    vpe->dst.width  = VPE_OUTPUT_W;
    vpe->dst.height = VPE_OUTPUT_H;
    describeFormat (VPE_OUTPUT_FORMAT, &vpe->dst);

    // display 영상 출력을 위한 초기화
    vpe->disp = disp_open(disp_argc, disp_argv);
    if (!vpe->disp) {
        ERROR("disp open error!");
        vpe_close(vpe);
        return 1;
    }

    // 기본 설정 세팅
    set_z_order(vpe->disp, vpe->disp->overlay_p.id);
    set_global_alpha(vpe->disp, vpe->disp->overlay_p.id);
    set_pre_multiplied_alpha(vpe->disp, vpe->disp->overlay_p.id);
    alloc_overlay_plane(vpe->disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);
    vpe->translen = 1;
    if (vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 || vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0) {
        ERROR("Invalid parameters\n");
    }

    // 영상 캡쳐를 위한 초기화
    v4l2 = v4l2_open(vpe->src.fourcc, vpe->src.width, vpe->src.height);
    if (!v4l2) {
        ERROR("v4l2 open error!");
        disp_close(vpe->disp);
        vpe_close(vpe);
        return 1;
    }

    tdata.disp = vpe->disp;
    tdata.v4l2 = v4l2;
    tdata.vpe = vpe;
    tdata.bfull_screen = true;
    tdata.bstream_start = false;

    if(-1 == (tdata.msgq_id = msgget((key_t)DUMP_MSGQ_KEY, IPC_CREAT | 0666))) {
        fprintf(stderr, "%s msg create fail!!!\n", __func__);
        return -1;
    }

    pexam_data = &tdata;

    ret = pthread_create(&tdata.threads[0], NULL, capture_thread, &tdata);
    if(ret) MSG("Failed creating capture thread");
    pthread_detach(tdata.threads[0]);

    ret = pthread_create(&tdata.threads[1], NULL, capture_dump_thread, &tdata);
    if(ret) MSG("Failed creating capture dump thread");
    pthread_detach(tdata.threads[1]);

    ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
    if(ret) MSG("Failed creating input thread");
    pthread_detach(tdata.threads[2]);

    /* register signal handler for <CTRL>+C in order to clean up */
    if(signal(SIGINT, signal_handler) == SIG_ERR) {
        MSG("could not register signal handler");
        closelog();
        exit(EXIT_FAILURE);
    }

    printf("\n\n 2. speed control\n");

    unsigned char status;
    unsigned char gain;
    signed short speed, check_speed;
    double angle = 1500;
    int camera_angle;
	int position, posInit, posDes, posRead;
	int channel;
	int tol;
	int data, distance; // distance 센서용

	CarControlInit();
    SpeedControlOnOff_Write(UNCONTROL);

    //jobs to be done beforehand;
    /** SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!! */
    /** speed = tdata.speed; // speed set     --> speed must be set when using position controller */
    /** DesireSpeed_Write(speed); */

     //camera y servo set
    camera_angle = CameraYServoControl_Read();
    printf("CameraYServoControl_Read() = %d\n", camera_angle);    //default = 1500, 0x5dc

    camera_angle = 1650;
    CameraYServoControl_Write(camera_angle);    

    //speed set     */
    check_speed = DesireSpeed_Read();  
    printf("DesireSpeed_Read() = %d \n", check_speed); 
    /** speed = 120; */
    /** DesireSpeed_Write(speed); */
/**  */
    while(1){
		// if(is_Traffic_Light >= 1)
		// 	break;
		// if(passing == 1)
		// 	break;

        switch(tdata.mission_state){
            // 기본주행 모드
            case AUTO_DRIVE : 
                while(distance_sensor > 8){
                    DesireSpeed_Wirte(tdata.speed);
                    DesireSpeed_Wirte(tdata.angle);
                    printf("Speed : %d", DesireSpeed_Read());
                    usleep(50000);
                }
                tdata.mission_state = PRE_PASSING_OVER;

            // 추월 미션 진입
            case PASSING_OVER_LEFT :
                passing_go_back();
                passing_left();

            case PASSING_OVER_RIGHT :
                passing_go_back();
                passing_right();

            case WAIT : 
                // 규열이의 차선주행
                DesireSpeed_Wirte(tdata.speed);
                DesireSpeed_Wirte(tdata.angle);
                printf("Speed : %d", DesireSpeed_Read());
                usleep(50000); // 이게 없으면 속력을 계속해서 주기 때문에 매우 낮은 속도로 감

            case PASSING_OVER_RETURN_RIGHT :
                passing_go_back_later();
                passing_right_later();

            case PASSING_OVER_RETURN_LEFT :
                passing_go_back_later();
                passing_left_later();

            case STOP :
                stop();
                // 미션 끝
        }
        //1 left 2 right -1 아무곳으로

        /* //*****************여기가 기본주행************
        angle = 1500-(tdata.angle/90)*500;
		/** speed = real_speed;  
		printf("speed = %d\n", tdata.speed);

		SteeringServoControl_Write(angle); 
		DesireSpeed_Write(tdata.speed);
		if(tdata.speed == 0)
			usleep(500000);
		usleep(50000);
        */
    }

	// if(is_Traffic_Light >= 1){
	// 	//시간제어
	// 	//좌회전 하도록 하기
	// 	if(is_Traffic_Light == 1){
	// 		//left
	// 		SteeringServoControl_Write(1950);
	// 		DesireSpeed_Write(200);
	// 		usleep(1700000);
	// 		printf("step 1...\n");

	// 		SteeringServoControl_Write(1500);
	// 		usleep(100000);
	// 		printf("step 2...\n");

	// 		printf("Basic Mode is ready... traffic light finished..!!!\n");
	// 		DesireSpeed_Write(0); //E-Stop;
	// 	}
	// 	else if(is_Traffic_Light == 2){
	// 		//right
	// 		SteeringServoControl_Write(1050);
	// 		DesireSpeed_Write(100);
	// 		usleep(2000000);
	// 		printf("step 1...\n");

	// 		SteeringServoControl_Write(1500);
	// 		usleep(1000000);
	// 		printf("step 2...\n");

	// 		printf("Basic Mode is ready... traffic light finished..!!!\n");
	// 		DesireSpeed_Write(0); //E-Stop;
	// 	}
	// 	else{
	// 		printf("ERROR!!!!\n");
	// 	}
	// }
	// printf("main passing_where = %d\n", passing_where);
	// printf("main passing = %d\n", passing);


	//koo
	pause();

	return ret;
}


/**
  * @brief  Camera capture, capture image covert by VPE and display after sobel edge
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
signed short color_detection(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;
	signed short speed = 0;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        speed = OpenCV_red_Detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
        is_Traffic_Light = OpenCV_green_Detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
		/** printf("speeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeed : %d\n", speed); */
        //¿¿¿¿ ¿¿¿
        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }

	/** printf("speeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeed = %d\n", speed); */
	/** real_speed = speed; */
	/** printf("color_detection func speed: %d\n", speed); */
	return speed;

}


void * capture_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    struct v4l2 *v4l2 = data->v4l2;
    struct vpe *vpe = data->vpe;
    struct buffer *capt;
    bool isFirst = true;
    int index;
    int count = 0;
    int i, k;
    int distance; // 적외선 센서로부터 받아오는 거리값

    v4l2_reqbufs(v4l2, NUMBUF); // 영상 저장할 큐 버퍼 메모리 할당
    vpe_input_init(vpe); // VPE 입력 초기화
    allocate_input_buffers(data); // VPE 입력 버퍼 할당

    // 이미지 색공간에 따른 설정 (NV12(Y plane UV plane 따로 있음)이면 multiplanar)))
    if(vpe->dst.coplanar) vpe->disp->multiplanar = true;
    else vpe->disp->multiplanar = false;
    vpe_output_init(vpe); // VPE 출력 초기화
    vpe_output_fullscreen(vpe, data->bfull_screen); // scaling 여부 설정

    // 영상 큐 처리 권한을 드라이버에게 이전하여 다음 영상 프레임 요청
    for (i = 0; i < NUMBUF; i++) v4l2_qbuf(v4l2,vpe->input_buf_dmafd[i], i);
    // VPE 출력 큐 처리 권한을 드라이버에게 이전하여 드라이버가 영상 가공 후 출력 큐에 데이터 저장
    for (i = 0; i < NUMBUF; i++) vpe_output_qbuf(vpe, i);

    v4l2_streamon(v4l2); // 영상 캡쳐 시작
    vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE); // VPE 하드웨어 입력 시작
    vpe->field = V4L2_FIELD_ANY;


    while(1) {
        index = v4l2_dqbuf(v4l2, &vpe->field); // 캡처 queue의 소유권으로 Application으로 가지고 옴
        vpe_input_qbuf(vpe, index); // VPE 입력 큐 처리 권한을 드라이버에게 이전 (VPE 드라이버가 영상 가공)
        // vpe-stream_on : VPE 입력 시작, 초기에 한번 호출
        if (isFirst) {
            vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE); // VPE 하드웨어 출력 시작
            isFirst = false;
            MSG("streaming started...");
            data->bstream_start = true;
        }
        index = vpe_output_dqbuf(vpe); // VPE 출력 큐 처리 권한을 어플리케이션이 가지고 옴
        capt = vpe->disp_bufs[index];

// ---- 적외선 센서 ---- 
        data->distance = distance_sensor();
        printf("distance = 0x%04X(%d) \n", data->distance);

// -------------------- capt로 이미지 처리 ----------------------------------

        // 여기서 data->mission_state로 던져줍니다

        if (data->mission_state == AUTO_DRIVE){
            data->angle = getSteeringWithLane(vpe->disp, capt); // 차선인식
		    data->speed = color_detection(vpe->disp, capt); 
        }

        // 맨 처음에 PASSING_OVER 미션의 전 미션이라고 생각하자
        data->mission_state = BEFORE_PASSING_OVER;
        
        // 추월 미션 진입 트리거
        else if (data->mission_state == BEFORE_PASSING_OVER && data->distance < 20){
            data->direction = passing_master(vpe->disp, capt, &data);
            if (data->direction == "left")
                data->mission_state = PASSING_OVER_LEFT;
            else if (data->direction == "right")
                data->mission_state = PASSING_OVER_RIGHT;
            else if (data->direction == "fail")
                data->mission_state = PASSING_OVER_LEFT; // 만일 역히스토그램 투영으로 방향을 도출해내지 못하면 왼쪽으로 간다고 설정
        }

        // 추월 이후 정지선을 인식할 때까지 차선 인식
        else if (data->mission_state == WAIT){ // WAIT은 불가피하게 main에서 바꾸어준다
            data->angle = getSteeringWithLane(vpe->disp, capt); // 차선인식 
		    data->speed = color_detection(vpe->disp, capt); 
            data->yellow_stop_line = stop_line(vpe->disp, capt, &data); // 정지선 인식하면 stop_line_recognition을 1로 return
        }

        // 정지선을 인식하면 다시 차로로 return
        else if (data->mission_state == WAIT && data->yellow_stop_line == "stop"){
            if (data->direction == "left" || data->direction == "fail")
                data->mission_state = PASSING_OVER_RETURN_RIGHT; 
            else if (data->direction == "right")
                data->mission_state = PASSING_OVER_RETURN_LEFT;
        }

        else if (data->mission_state == PASSING_OVER_RETURN_RIGHT || data->mission_state == PASSING_OVER_RETURN_LEFT){
            data->white_stop_line = line_trace_sensor();
            if (data->white_stop_line == 0) // 0이면 흰색, 1이면 검은색 -> 흰색인 정지선을 만날 때 멈추어야 한다
                data->mission_state = STOP;
        }

        /**  */
        /** if (data->angle == 1234) { */
        /**     data->angle = 0; */
        /**     [> data->speed = 0; <] */
		/**     real_speed = 0; */
        /** } */
        /** [> else { <] */
        /**     [> data->speed = 120; <] */
		/**     real_speed = 120; */
        /** } */
        /**  */

        // fineLane(vpe->disp, capt);



// ----------------------- 이미지 처리 end ----------------------------------


        // 영상 출력 버퍼에 영상 데이터 입력
        if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height)) {
            ERROR("Post buffer failed");
            return NULL;
        }
        update_overlay_disp(vpe->disp); // overay plane 출력                                       

        // dump 키보드 입력 들어올 시 캡처 이미지 저장
        if(data->dump_state == DUMP_READY) {
            DumpMsg dumpmsg;
            unsigned char* pbuf[4];

            if(get_framebuf(capt, pbuf) == 0) {
                switch(capt->fourcc) {
                    case FOURCC('Y','U','Y','V'):
                    case FOURCC('B','G','R','3'):
                        memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_IMG_SIZE);
                        break;
                    case FOURCC('N','V','1','2'):
                        memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H); // y data
                        memcpy(data->dump_img_data+VPE_OUTPUT_W*VPE_OUTPUT_H, pbuf[1], VPE_OUTPUT_W*VPE_OUTPUT_H/2); // uv data
                        break;
                    default :
                        MSG("DUMP.. not yet support format : %.4s\n", (char*)&capt->fourcc);
                        break;
                }
            } else {
                MSG("dump capture buf fail !");
            }

            dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
            dumpmsg.state_msg = DUMP_WRITE_TO_FILE;
            data->dump_state = DUMP_WRITE_TO_FILE;
            if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                MSG("state:%d, msg send fail\n", dumpmsg.state_msg);
            }
        }

        vpe_output_qbuf(vpe, index); // VPE 출력 큐 처리 권한을 드라이버에게 이전
        index = vpe_input_dqbuf(vpe); // VPE 입력 큐 처리 권한을 어플리케이션이 가지고 옴
        v4l2_qbuf(v4l2, vpe->input_buf_dmafd[index], index); // 영상 큐 처리 권한을 드라이버에게 이전하여 다음 영상 프레임 요청

    }

    MSG("Ok!");
    return NULL;
}


static char* passing_master(struct display *disp, struct buffer *cambuf, void *arg){
    struct thr_data *data = (struct thr_data *)arg;
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et; 
    signed short speed;
    double distance;
    char* direction; // 갈 방향, left of right!
	/** printf("Koo is Passing master!!!!!!! wow!!!!!!\n"); */

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);
    
//=================================================================================
        // switch(data->mission_state) {
        //     case PRE_PASSING_OVER : //그림자 히스토그램을 구하고 hist에 저장
        //         data->passing.hist = pre_histogram_backprojection(srcBuf, iw, ih);
        //         if (data->passing.hist_check == 1) {
        //             data->mission_state = WAIT;
        //         }
            
        //     case PASSING_OVER : //뒤로 간 후에 전체 이미지에 대해 그림자 히스토그램을 검색 -> 방향 결정
        //         decision = histogram_backprojection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, data->passing.hist);
        //         passing_where = decision;
        //         data->mission_state = WAIT;
        // }
    
        direction = histogram_backprojection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
       
		/** printf("passing_where = %d\n", passing_where); */
//=======================================================================================

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }

    return direction;

}


static char* stop_line(struct display *disp, struct buffer *cambuf, void *arg){
    struct thr_data *data = (struct thr_data *)arg;
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et; 
    signed short speed;
    double distance;
    char* stop_line_recognition; // 갈 방향, left or right!
	/** printf("Koo is Passing master!!!!!!! wow!!!!!!\n"); */

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);
    
//================================================================================= 
        stop_line_recognition = stop_line_detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
        
		/** printf("passing_where = %d\n", passing_where); */
//=======================================================================================

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }

    return stop_line_recognition;

}


/**
  * @brief  Hough transform the captured image dump and save to file
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * capture_dump_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    FILE *fp;
    char file[50];
    struct timeval timestamp;
    struct tm *today;
    DumpMsg dumpmsg;

    while(1) {
        if(msgrcv(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), DUMP_MSGQ_MSG_TYPE, 0) >= 0) {
            switch(dumpmsg.state_msg) {
                case DUMP_CMD :
                    gettimeofday(&timestamp, NULL);
                    today = localtime(&timestamp.tv_sec);
                    sprintf(file, "dump_%04d%02d%02d_%02d%02d%02d.%s", today->tm_year+1900, today->tm_mon+1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec,VPE_OUTPUT_FORMAT);
                    data->dump_state = DUMP_READY;
                    MSG("file name:%s", file);
                    break;

                case DUMP_WRITE_TO_FILE :
                    if((fp = fopen(file, "w+")) == NULL){
                        ERROR("Fail to fopen");
                    } else {
                        fwrite(data->dump_img_data, VPE_OUTPUT_IMG_SIZE, 1, fp);
                    }
                    fclose(fp);
                    data->dump_state = DUMP_DONE;
                    break;

                default :
                    MSG("dump msg wrong (%d)", dumpmsg.state_msg);
                    break;
            }
        }
    }

    return NULL;
}

/**
  * @brief  handling an input command
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * input_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    char cmd_input[128];
    char cmd_ready = true;

    while(!data->bstream_start) {
        usleep(100*1000);
    }

    MSG("\n\nInput command:");
    MSG("\t dump  : display image(%s, %dx%d) dump", VPE_OUTPUT_FORMAT, VPE_OUTPUT_W, VPE_OUTPUT_H);
    MSG("\n");

    while(1)
    {
        if(cmd_ready == true) {
            /*standby to input command */
            cmd_ready = StandbyInput(cmd_input);     //define in cmd.cpp
        } else {
            if(0 == strncmp(cmd_input,"dump",4)) {
                DumpMsg dumpmsg;
                dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
                dumpmsg.state_msg = DUMP_CMD;
                data->dump_state = DUMP_CMD;
                MSG("image dump start");
                if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                    printf("dump cmd msg send fail\n");
                }

                while(data->dump_state != DUMP_DONE) {
                    usleep(5*1000);
                }
                data->dump_state = DUMP_NONE;
                MSG("image dump done");
            } else {
                printf("cmd_input:%s \n", cmd_input);
            }
            cmd_ready = true;
        }
    }

    return NULL;
}



/**
  * @brief  handling an SIGINT(CTRL+C) signal
  * @param  sig: signal type
  * @retval none
  */
void signal_handler(int sig)
{
    if(sig == SIGINT) {
        pthread_cancel(pexam_data->threads[0]);
        pthread_cancel(pexam_data->threads[1]);
        pthread_cancel(pexam_data->threads[2]);

        msgctl(pexam_data->msgq_id, IPC_RMID, 0);

        v4l2_streamoff(pexam_data->v4l2);
        vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

        disp_free_buffers(pexam_data->vpe->disp, NUMBUF);
        free_input_buffers(pexam_data->input_bufs, NUMBUF, false);
        free_overlay_plane(pexam_data->vpe->disp);

        disp_close(pexam_data->vpe->disp);
        vpe_close(pexam_data->vpe);
        v4l2_close(pexam_data->v4l2);

        printf("-- laneTrace End ... pky --\n");
    }
}




/**
  * @brief  Alloc vpe input buffer and a new buffer object
  * @param  data: pointer to parameter of thr_data
  * @retval none
  */
static int allocate_input_buffers(struct thr_data *data)
{
    int i;
    struct vpe *vpe = data->vpe;

    data->input_bufs = calloc(NUMBUF, sizeof(*data->input_bufs));
    for(i = 0; i < NUMBUF; i++) {
        data->input_bufs[i] = alloc_buffer(vpe->disp, vpe->src.fourcc, vpe->src.width, vpe->src.height, false);
    }
    if (!data->input_bufs)
        ERROR("allocating shared buffer failed\n");

    for (i = 0; i < NUMBUF; i++) {
        /** Get DMABUF fd for corresponding buffer object */
        vpe->input_buf_dmafd[i] = omap_bo_dmabuf(data->input_bufs[i]->bo[0]);
        data->input_bufs[i]->fd[0] = vpe->input_buf_dmafd[i];
    }
    return 0;
}

/**
  * @brief  Free vpe input buffer and destroy a buffer object
  * @param  buffer: pointer to parameter of buffer object
                  n : count of buffer object
                  bmultiplanar : multipanar value of buffer object
  * @retval none
  */
static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        if (buffer[i]) {
            close(buffer[i]->fd[0]);
            omap_bo_del(buffer[i]->bo[0]);
            if(bmultiplanar){
                close(buffer[i]->fd[1]);
                omap_bo_del(buffer[i]->bo[1]);
            }
        }
    }
    free(buffer);
}



/**
  * @brief  Draw operating time to overlay buffer.
  * @param  disp: pointer to parameter of struct display
                  time : operate time (ms)
  * @retval none
  */
static void draw_operatingtime(struct display *disp, uint32_t time)
{
    FrameBuffer tmpFrame;
    unsigned char* pbuf[4];
    char strtime[128];

    memset(strtime, 0, sizeof(strtime));

    sprintf(strtime, "%03d(ms)", time);

    if(get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
        tmpFrame.buf = pbuf[0];
        tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
        tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

        drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
    }
}



/**
  * @brief  Handle houht transform with opencv api
  * @param  disp: pointer to parameter of struct display
                 cambuf: vpe output buffer that converted capture image
  * @retval none
  */
double getSteeringWithLane(struct display *disp, struct buffer *cambuf)
{
    double angle;
    signed short speed;
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        angle = laneDetection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
		/** speed = OpenCV_red_Detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);  */
	

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
	return angle;
}

