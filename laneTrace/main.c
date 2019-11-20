/////////////////////////////////////////////////////////////////////////
//////////////////////// INCLUDE FILES //////////////////////////////////
/////////////////////////////////////////////////////////////////////////

#include <stdio.h>
/** #include "car_lib.h" */
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
#include "controlSensor.h"
/** #include "passing_master.h" */

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

typedef struct _DumpMsg{
    long type;
    int  state_msg;
}DumpMsg;
struct thr_data {
    struct display *disp;
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct buffer **input_bufs;

    double angle; /// 규열이만 이 변수 set 가능, 나머진 전부 이 변수 get 가능
	signed short speed;
	signed short speed_ratio;//태영 edit this var 0 or 1

	double sensor1;//front
	double sensor2;//right1
	double sensor3;//right2
	double sensor4;//back
	double sensor5;//left1
	double sensor6;//left2

    DumpState dump_state;
    unsigned char dump_img_data[VPE_OUTPUT_IMG_SIZE];

    int msgq_id;
    bool bfull_screen;
    bool bstream_start;
    pthread_t threads[3];

    int mission_id; /// by dy
	int park;//jh for parking
    bool driving_flag_onoff; /// by dy: true면 주행중, false면 주행종료
    //double speed_ratio = 1; /// by dy: 태영이랑 도연이만 이 변수 건드릴 수 있음. 정지 표지판이나 회전교차로에서 정지해야하면 이 비율을 0으로 두기
    bool stop_line_detect; /// by dy: true 면 정지선 인식된거임
};

signed short real_speed = 0;
int is_Traffic_Light = 0; //1 is traffic light mission 1 is left, 2 is right
int passing_where = -1; // 1 is left 2 is right
int passing = 0; // decide the time to passing other car

static int allocate_input_buffers(struct thr_data *data);
static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar);
static void draw_operatingtime(struct display *disp, uint32_t time);
double getSteeringWithLane(struct display *disp, struct buffer *cambuf); //detect yellow lane
void * capture_thread(void *arg);
void * capture_dump_thread(void *arg);
void * input_thread(void *arg);
double distance_calculate(double data); // make sensor input data to real distance data
double distance_sensor(); //get sensor input data
static void passing_master(struct display *disp, struct buffer *cambuf); // passing other car 차선변경 미션

static struct thr_data* pexam_data = NULL;
void signal_handler(int sig);


int main(int argc, char **argv)
{
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct thr_data tdata;
    struct thr_data* data;
    int disp_argc = 3;
    char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"};
    int ret = 0;

    printf("laneTrace Start ~~~ pky\n");

    tdata.dump_state = DUMP_NONE;
    memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data)); // dump data를 0으로 채워서 초기화
	//init data struct
	tdata.mission_id = 0;
    tdata.driving_flag_onoff = true; /// by dy: true면 주행중, false면 주행종료
    tdata.speed_ratio = 1; /// by dy: 태영이랑 도연이만 이 변수 건드릴 수 있음. 정지 표지판이나 회전교차로에서 정지해야하면 이 비율을 0으로 두기
    tdata.stop_line_detect = false; /// by dy: true 면 정지선 인식된거임
	tdata.park = 0;//non

    // init for using VPE hardware
    vpe = vpe_open(); if(!vpe) return 1;
    // image input
    vpe->src.width  = CAPTURE_IMG_W;
    vpe->src.height = CAPTURE_IMG_H;
    describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);
    // image output
    vpe->dst.width  = VPE_OUTPUT_W;
    vpe->dst.height = VPE_OUTPUT_H;
    describeFormat (VPE_OUTPUT_FORMAT, &vpe->dst);

    // init for display 
    vpe->disp = disp_open(disp_argc, disp_argv);
    if (!vpe->disp) {
        ERROR("disp open error!");
        vpe_close(vpe);
        return 1;
    }

    // basic setting
    set_z_order(vpe->disp, vpe->disp->overlay_p.id);
    set_global_alpha(vpe->disp, vpe->disp->overlay_p.id);
    set_pre_multiplied_alpha(vpe->disp, vpe->disp->overlay_p.id);
    alloc_overlay_plane(vpe->disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);
    vpe->translen = 1;
    if (vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 || vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0) {
        ERROR("Invalid parameters\n");
    }

    // init for capture the video
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

    unsigned char gain;
    signed short speed, check_speed;
    double angle = 1500;
    int camera_angle;
	int position, posInit, posDes, posRead;

	CarControlInit();
    SpeedControlOnOff_Write(UNCONTROL);
 
     //camera y servo set
    camera_angle = CameraYServoControl_Read();
    printf("CameraYServoControl_Read() = %d\n", camera_angle);    //default = 1500

	//camera setting
    camera_angle = 1650;
    CameraYServoControl_Write(camera_angle);    

    //speed set
    check_speed = DesireSpeed_Read();  
    printf("DesireSpeed_Read() = %d \n", check_speed); 

    ////////////////////////////////////////////////////////// DY added /////////////////////////////////////////////////////////
//[TODO]일단은 주석처리 해둠... 실전에서 사용하기
	while (true){
		if (data->mission_id == 1) {        } /// start
		else if (data->mission_id == 2) {   } /// highway
		else if (data->mission_id == 3) {   } /// 회전
		else if (data->mission_id == 4) {
			tunnel_adv();
			data->mission_id = 0;//example
		} /// 터널 
		else if (data->mission_id == 5) {
			parking();
			data->mission_id = 0;//example
		} /// 수직주차
		else if (data->mission_id == 6) {
			parparking();
			data->mission_id = 0;//example
		} /// 수평주차
		else if (data->mission_id == 7) {   } /// 추월
		else if (data->mission_id == 8) {//[TODO] 판교가서 튜닝
			printf("traffic mission!!!!\n");
			if(is_Traffic_Light == 1){
				//go left
				SteeringServoControl_Write(1950);
				DesireSpeed_Write(200);
				usleep(1700000);
				printf("step 1...\n");

				SteeringServoControl_Write(1500);
				usleep(100000);
				printf("step 2...\n");

				printf("traffic light finished..!!!\n");
				DesireSpeed_Write(0); //E-Stop;
			}
			else if(is_Traffic_Light == 2){
				//right
				SteeringServoControl_Write(1050);
				DesireSpeed_Write(100);
				usleep(1700000);
				printf("step 1...\n");

				SteeringServoControl_Write(1500);
				usleep(1000000);
				printf("step 2...\n");

				printf("traffic light finished..!!!\n");
				DesireSpeed_Write(0); //E-Stop;
			}
			else{
				printf("ERROR!!!!\n");
			}
		}	/// 신호등
		else { //basic driving 
			angle = 1500-(tdata.angle/90)*500; //get angle from data structure
			printf("tdata.speed = %d\n", tdata.speed);//error

			SteeringServoControl_Write(angle); 
			DesireSpeed_Write(tdata.speed);
			if(tdata.speed == 0){
				printf("stop!!\n");
				usleep(500000); //calibrate IO delay
			}
			usleep(50000); //calibrate IO delay

		} 


		if (data->driving_flag_onoff == false) /// mission end, the car will be stopped.
		{//신호등 이후에 하자
			/// 추후에 이 코드 종료 미션에 맞게 바꿀 것
			DesireSpeed_Write(0);
			Alarm_Write(ON);
			usleep(1000000);
			Alarm_Write(OFF);
			break;
			
		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**  */
    /** while(1){ */
	/**     if(is_Traffic_Light >= 1) //Traffic light mission */
	/**         break; */
	/**     if(passing == 1) //passing other car mission */
	/**         break; */
    /**  */
    /**     angle = 1500-(tdata.angle/90)*500; //get angle from data structure */
	/**     printf("tdata.speed = %d\n", tdata.speed);//error */
    /**  */
	/**     SteeringServoControl_Write(angle);  */
	/**     DesireSpeed_Write(tdata.speed); */
	/**     if(tdata.speed == 0){ */
	/**         printf("stop!!\n"); */
	/**         usleep(500000); //calibrate IO delay */
	/**     } */
	/**     usleep(50000); //calibrate IO delay */
    /** } */
    /**  */
	/** if(is_Traffic_Light >= 1){ //traffic mission */
	/**     printf("traffic mission!!!!\n"); */
	/**     if(is_Traffic_Light == 1){ */
	/**         //go left */
	/**         SteeringServoControl_Write(1950); */
	/**         DesireSpeed_Write(200); */
	/**         usleep(1700000); */
	/**         printf("step 1...\n"); */
    /**  */
	/**         SteeringServoControl_Write(1500); */
	/**         usleep(100000); */
	/**         printf("step 2...\n"); */
    /**  */
	/**         printf("traffic light finished..!!!\n"); */
	/**         DesireSpeed_Write(0); //E-Stop; */
	/**     } */
	/**     else if(is_Traffic_Light == 2){ */
	/**         //right */
	/**         SteeringServoControl_Write(1050); */
	/**         DesireSpeed_Write(100); */
	/**         usleep(1700000); */
	/**         printf("step 1...\n"); */
    /**  */
	/**         SteeringServoControl_Write(1500); */
	/**         usleep(1000000); */
	/**         printf("step 2...\n"); */
    /**  */
	/**         printf("traffic light finished..!!!\n"); */
	/**         DesireSpeed_Write(0); //E-Stop; */
	/**     } */
	/**     else{ */
	/**         printf("ERROR!!!!\n"); */
	/**     } */
	/** } */
	/** printf("main passing_where = %d\n", passing_where); */
	/** printf("main passing = %d\n", passing); */
    /**  */
	/** if(passing_where != -1){//passing other car mission */
	/**     printf("Go back!\n"); */
	/**     SpeedControlOnOff_Write(CONTROL); */
	/**     speed = -200; */
    /**  */
	/**     //control on/off */
	/**     PositionControlOnOff_Write(CONTROL); */
    /**  */
	/**     //position controller gain set */
	/**     gain = 30; */
	/**     PositionProportionPoint_Write(gain); */
    /**  */
	/**     //position write */
	/**     posInit = 0;  //initialize */
	/**     EncoderCounter_Write(posInit); */
    /**  */
	/**     //position set */
	/**     posDes = -800; */
	/**     position = posInit+posDes; */
	/**     DesireEncoderCount_Write(position); */
    /**  */
	/**     position=DesireEncoderCount_Read(); */
	/**     printf("DesireEncoderCount_Read() = %d\n", position); */
    /**  */
	/**     sleep(1); */
	/**     speed = 0; */
	/**     DesireSpeed_Write(speed); */
	/**     //mission_count ++; */
    /**  */
	/**     CarLight_Write(ALL_ON); */
	/**     Alarm_Write(ON); */
	/**     usleep(100000); */
	/**     Alarm_Write(OFF); */
	/**     CarLight_Write(ALL_OFF); */
    /**  */
	/**     if(passing_where == 1){ */
	/**         printf("Go left!\n"); */
	/**         //[TODO]To be continue... */
	/**         //However, mechanism is similar with go right! */
	/**     } */
	/**     else if(passing_where == 2){ */
	/**         printf("GO right!\n"); */
    /**  */
	/**         SpeedControlOnOff_Write(CONTROL); */
    /**  */
	/**         speed = 70; */
	/**         DesireSpeed_Write(speed); */
    /**  */
	/**         angle = 1150; */
	/**         SteeringServoControl_Write(angle); */
    /**  */
	/**         //control on/off */
	/**         PositionControlOnOff_Write(CONTROL); */
    /**  */
	/**         //position controller gain set */
	/**         gain = 30; */
	/**         PositionProportionPoint_Write(gain); */
    /**  */
	/**         //position write */
	/**         posInit = 0;  //initialize */
	/**         EncoderCounter_Write(posInit); */
    /**  */
	/**         //position set */
	/**         posDes = 1000; */
	/**         position = posInit+posDes; */
	/**         DesireEncoderCount_Write(position); */
    /**  */
	/**         position=DesireEncoderCount_Read(); */
	/**         printf("DesireEncoderCount_Read() = %d\n", position); */
    /**         [>  <] */
	/**         [> tol = 100;    // tolerance <] */
	/**         [> while(abs(posRead-position)>tol) <] */
	/**         [> { <] */
	/**         [>     posRead=EncoderCounter_Read(); <] */
	/**         [>     printf("EncoderCounter_Read() = %d\n", posRead); <] */
	/**         [>     speed = DesireSpeed_Read(); <] */
	/**         [>     printf("DesireSpeed_Read() = %d \n", speed); <] */
	/**         [> } <] */
	/**         [> sleep(1); <] */
    /**  */
	/**         PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!! */
	/**         SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!! */
    /**  */
	/**         speed = 70; */
	/**         DesireSpeed_Write(speed); */
	/**          */
	/**         // basic driving */
	/**         angle = 1850; */
	/**         SteeringServoControl_Write(angle); */
	/**         usleep(1500000); */
    /**  */
	/**         angle = 1500; */
	/**         SteeringServoControl_Write(angle); */
    /**  */
	/**         speed = 70; */
	/**         DesireSpeed_Write(speed); */
    /**  */
	/**         //stop */
	/**         usleep(150000); */
	/**         speed = 0; */
	/**         DesireSpeed_Write(speed); */
    /**  */
	/**     } */
	/** } */
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
	signed short speed_ratio = 0;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        speed_ratio = OpenCV_red_Detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
		is_Traffic_Light = OpenCV_green_Detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
		/** printf("speed : %d\n", speed); *///ok

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }

	return speed_ratio;
}


void * capture_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    struct v4l2 *v4l2 = data->v4l2;
    struct vpe *vpe = data->vpe;
    struct buffer *capt;
    bool isFirst = true;
    int index;
    int i;

	//for understanding capture_thread...

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


// -------------------- image process by capt ----------------------------------
        /** data->angle = getSteeringWithLane(vpe->disp, capt);  */

		if (data->angle == 1234) { //fail to detect lane
			data->angle = 0;
			data->speed = 0;
			real_speed = 0;
			//printf("fail to detect lane!!!!\n");
		}
		data->speed_ratio = color_detection(vpe->disp, capt); 
		data->speed = data->speed * data->speed_ratio;
		/** printf("sppppppppppppppppppppppppppppppppppppeedd: %d\n", data->speed);//ok */

// ----------------------- end of image process ----------------------------------


        // input video data to disp_buf
        if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height)) {
            ERROR("Post buffer failed");
            return NULL;
        }
        update_overlay_disp(vpe->disp); // diplay overay plane                                       

        // save image frame when input 'dump' 
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

double getSteeringWithLane(struct display *disp, struct buffer *cambuf) //detect lane
{
    double angle;
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        angle = laneDetection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H); //get angle value from laneDetection()

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
	return angle;
}

double distance_calculate(double data){ // make sensor input data to real distance data
    double middle = data / 22691; //for the car of Alpha car team
	/** printf("middle = %lf\n", middle); */
    double distance = pow(middle, -(1/1.07)); //for the car of Alpha car team
    return distance;
}

double distance_sensor(){ //get sensor input data
	int channel = 1;
	double data;
    /** printf("distance sensor start!!!\n"); */
    double distance = 1;
    while(1){
        data = DistanceSensor(channel);
        distance = distance_calculate(data);
        /** printf("channel = %d, distance = %d\n", channel, distance); */
        return distance;
    }
}

static void passing_master(struct display *disp, struct buffer *cambuf){
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;
    int decision; 
    signed short speed;
    double distance;
    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);
        gettimeofday(&st, NULL);

        decision = histogram_backprojection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
		decision = 2;//[TODO] delete this!!!
        
        distance = distance_sensor();
        /**  */
        /** if(distance > 15){  */
        /**     printf("basic driving mode\n"); */
        /**     speed = 100; */
        /**     DesireSpeed_Write(speed); */
        /** } */
		/** printf("decision= %d\n", decision); */

        //pass when the distance between mycar and other car below 15
        if(distance < 15){
            /** if(decision == -1){ */
            /** printf("basic driving mode\n");             */
            /** speed = 100; */
            /** DesireSpeed_Write(speed); */
            /** } */
            if(decision == 1){
                //passing_left();
				passing_where = 1;
            } 
            else if(decision == 2){
                //passing_right();
				passing_where = 2;
            }
        }
		/** printf("passing_where = %d\n", passing_where); */

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }


}

