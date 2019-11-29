#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>

#include "util.h"

#include "car_lib.h"
#include "koo_driving.h"
#include "display-kms.h"
#include "v4l2.h"
#include "vpe-common.h"
#include "drawing.h"
#include "input_cmd.h"
#include "exam_cv.h"
#include "laneDetection.h"
#include "DynamicObs.h"

/** #include "controlSensor.h" */
/** #include "passing_master.h" */

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
//--koo structure
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
//---
struct thr_data {
    struct display *disp;
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct buffer **input_bufs;

    double angle; /// pky set this value
    double pre_angle;
	signed short speed;
	signed short speed_ratio;// lizard edit this var 0 or 1

	//region of junho[TODO]init
	int ParkingSignal_1;
	int ParkingSignal_2;//after 수직주차 1
	int parParkingSignal_1;
	int parParkingSignal_2;
	int tunnelSignal;

	int I_data_1, I_data_2, I_data_3, I_data_4, I_data_5, I_data_6;
	int O_data_1, O_data_2, O_data_3, O_data_4, O_data_5, O_data_6;
	//end
	//
	//region of koo
	//struct passing *passing; // 2019.11.16 관형 추가
    int distance; // 1번 적외선센서 거리값, 관형 추가
    char* direction; // 추월 차로 진행 방향, left or right
    char* yellow_stop_line; // 정지선 인식 변수, 관형 추가
    int white_stop_line; // 정지선 인식 변수, 관형 추가
    MissionState mission_state;
	//end
	
	// ----dY
	bool stop_line_detect;
	// ---- end
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
    bool stop_line; /// by dy: true 면 정지선 인식된거임
};

/** signed short real_speed = 0; */
int is_Traffic_Light = 0; //1 is traffic light mission 1 is left, 2 is right
int passing_where = -1; // 1 is left 2 is right
int passing = 0; // decide the time to passing other car

static int allocate_input_buffers(struct thr_data *data);
static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar);
static void draw_operatingtime(struct display *disp, uint32_t time);

void getSteeringWithLane(struct display *disp, struct buffer *cambuf, double *steer, double *speed);
void * capture_thread(void *arg);
void * capture_dump_thread(void *arg);
void * input_thread(void *arg);
double distance_calculate(double data); // make sensor input data to real distance data
int distance_sensor(); //get sensor input data
static char* passing_master(struct display *disp, struct buffer *cambuf, void *arg); // 2019.11.16 관형 변경
void warmSensorTrigger(); 
static struct thr_data* pexam_data = NULL;
void signal_handler(int sig);
int stopLine_detect(void); // 정지선 인식하는 함수 1이면 정지선 위, 0이면 아님 by Doyeon

int main(int argc, char **argv)
{
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct thr_data tdata;
    struct thr_data* data;
	data = &tdata;
    int disp_argc = 3;
    char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"};
    int ret = 0;

    clock_t start_1=0, start_2=0, start_3=0;
    float endtime_1=0, endtime_2=0, endtime_3=0;

    tdata.dump_state = DUMP_NONE;
    memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data)); // dump data를 0으로 채워서 초기화

	//init data struct
	tdata.mission_id = 0; // 0 is basic driving 1 is for testing
    tdata.driving_flag_onoff = true; /// by dy: true면 주행중, false면 주행종료
    tdata.pre_angle = 0;
    tdata.speed_ratio = 1; /// by dy: 태영이랑 도연이만 이 변수 건드릴 수 있음. 정지 표지판이나 회전교차로에서 정지해야하면 이 비율을 0으로 두기
    tdata.stop_line_detect = false; /// by dy: true 면 정지선 인식된거임
	tdata.park = 0;//non
	tdata.ParkingSignal_1 = 0;
	tdata.ParkingSignal_2 = 0;
	tdata.parParkingSignal_1 = 0;
	tdata.parParkingSignal_2 = 0;
	tdata.tunnelSignal = 0;
	tdata.I_data_1 = 0;
	tdata.I_data_2 = 0;
	tdata.I_data_3 = 0;
	tdata.I_data_4 = 0;
	tdata.O_data_1 = 0;
	tdata.O_data_2 = 0;
	tdata.O_data_3 = 0;
	tdata.O_data_4 = 0;
    tdata.direction = "NONE"; // 추월 차로 진행 방향, left or right
    tdata.yellow_stop_line = "NONE"; // 정지선 인식 변수, 관형 추가
    tdata.white_stop_line = -1; // 정지선 인식 변수, 관형 추가
	tdata.mission_state = BEFORE_PASSING_OVER;

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
    SpeedControlOnOff_Write(CONTROL);
    PositionControlOnOff_Write(UNCONTROL);

    //speed controller gain set
    //P-gain
    gain = SpeedPIDProportional_Read();        // default value = 10, range : 1~50
    printf("SpeedPIDProportional_Read() = %d \n", gain);
    gain = 10;
    SpeedPIDProportional_Write(gain);

    //I-gain
    gain = SpeedPIDIntegral_Read();        // default value = 10, range : 1~50
    printf("SpeedPIDIntegral_Read() = %d \n", gain);
    gain = 10;
    SpeedPIDIntegral_Write(gain);
    
    //D-gain
    gain = SpeedPIDDifferential_Read();        // default value = 10, range : 1~50
    printf("SpeedPIDDefferential_Read() = %d \n", gain);
    gain = 10;
    SpeedPIDDifferential_Write(gain);

     //camera y servo set
    camera_angle = CameraYServoControl_Read();
    printf("CameraYServoControl_Read() = %d\n", camera_angle);    //default = 1500

	//camera setting
    camera_angle = 1650;//1650
    CameraYServoControl_Write(camera_angle);    

    //speed set
    check_speed = DesireSpeed_Read();  
    printf("DesireSpeed_Read() = %d \n", check_speed); 

    ////////////////////////////////////////////////////////// DY added /////////////////////////////////////////////////////////
	while (true){
		/** printf("mission_id = %d\n", data->mission_id); */

		if (data->mission_id == 1) {//test driving
			DesireSpeed_Write(100);
			usleep(1000000);
		} 
		/// start & highway
		else if (data->mission_id == 2) {   } ///
		else if (data->mission_id == 3) {/// 회전
			dynamic_obs_ver2(data->angle, data->speed, data->speed_ratio);
		} 
		else if (data->mission_id == 4) {/// 터널
			if(data->tunnelSignal == 1 && data->O_data_2 < 30){
				start_3 = clock();
			}
			if(data->O_data_2 < 30 && data->O_data_3 < 30){
				float endtime_3 = (clock()- start_3)/(CLOCKS_PER_SEC);
				if(endtime_3 < 3){
					tunnel_adv();
				}
			}
		}  
		else if (data->mission_id == 5) {//수직
			if(data->ParkingSignal_2 == 0 && data->ParkingSignal_1 == 0 && data->O_data_2 < 30 && data->O_data_3 > 30){
				start_1 = clock();
				data->ParkingSignal_1 = 1;
				printf("Parking Point Detected...\n");
				// continue;
			}
			if(data->ParkingSignal_1 == 1 && data->O_data_2 > 30 && data->O_data_3 < 30){
				data->ParkingSignal_1 = 2;
				printf("Parking Area here\n");
				float endtime_1 = (clock() - start_1)/(CLOCKS_PER_SEC);
				if(endtime_1 > 10){
					data->ParkingSignal_1 = 0;
				}
				//continue;
			}
			if(data->ParkingSignal_1 == 2 && data->O_data_2 > 30 && data->O_data_3 > 30){
				data->ParkingSignal_1 = 3;
			}
			if(data->ParkingSignal_1 == 3 && data->O_data_3 < 30){
				parking();
				data->ParkingSignal_2 = 1;
				data->parParkingSignal_2 = 1;
				data->mission_id = 1;// test driving edit it to 0
			}
		} 
		else if (data->mission_id == 6) {//수평주차
			if(data->parParkingSignal_2 == 1 && data->parParkingSignal_1 == 0 && data->O_data_2 < 30 && data->O_data_3 > 30){
				start_2 = clock();
				data->parParkingSignal_1 = 1;
			}
			if(data->parParkingSignal_1 == 1 && data->O_data_2 > 30 && data->O_data_3 < 30){
				data->parParkingSignal_1 = 2;  
				float endtime_2 = (clock() - start_2)/(CLOCKS_PER_SEC);
				if(endtime_2 > 10){
					data->parParkingSignal_1 = 0;
				}
			}
			if(data->parParkingSignal_1 == 2 && data->O_data_2 > 30 && data->O_data_3 > 30)
			{
				data->parParkingSignal_1 = 3;         
			}
			if(data->parParkingSignal_1 == 3 && data->O_data_3 < 30)
			{
				parparking();
				data->parParkingSignal_2 = 2;
				data->tunnelSignal = 1;
				data->mission_id = 1;// test driving edit it to 0
			}

		} /// 수평주차
		else if (data->mission_id == 7) {//passing master
			switch(data->mission_state){//[TODO] what is initial mission_state?
				// 기본주행 모드
				case AUTO_DRIVE : 
					/** while(distance_sensor > 8){ */
					while(data->distance > 8){
						DesireSpeed_Write(data->speed);
						SteeringServoControl_Write(data->angle);
						printf("Speed : %d", DesireSpeed_Read());
						usleep(50000);
					}
					//tdata.mission_state = PRE_PASSING_OVER;

					// 추월 미션 진입
				case PASSING_OVER_LEFT :
					passing_go_back();
					passing_left();

				case PASSING_OVER_RIGHT :
					passing_go_back();
					passing_right();

				case WAIT : 
					// 규열이의 차선주행
					DesireSpeed_Write(100);
					SteeringServoControl_Write(1500);
					printf("Speed : %d", DesireSpeed_Read());
					usleep(1500000); // 이게 없으면 속력을 계속해서 주기 때문에 매우 낮은 속도로 감

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

		} /// 추월

		else if (data->mission_id == 8) {//[TODO] 튜닝
			printf("traffic mission!!!!\n");
			if(is_Traffic_Light == 1){
				//go left
				DesireSpeed_Write(-200);
				usleep(10000);

				SteeringServoControl_Write(1950);
				DesireSpeed_Write(200);
				usleep(1700000);
				printf("step 1...\n");

				SteeringServoControl_Write(1500);
				usleep(100000);
				printf("step 2...\n");

				printf("traffic light finished..!!!\n");
				DesireSpeed_Write(0); //E-Stop;
				Alarm_Write(ON);
				usleep(1000000);
				Alarm_Write(OFF);
				break;
			}
			else if(is_Traffic_Light == 2){
				//right
				DesireSpeed_Write(-200);
				usleep(10000);

				SteeringServoControl_Write(1050);
				DesireSpeed_Write(100);
				usleep(1700000);
				printf("step 1...\n");

				SteeringServoControl_Write(1500);
				usleep(1000000);
				printf("step 2...\n");

				printf("traffic light finished..!!!\n");
				DesireSpeed_Write(0); //E-Stop;
				Alarm_Write(ON);
				usleep(1000000);
				Alarm_Write(OFF);
				break;
			}
			else{
				printf("ERROR!!!!\n");
			}
		}	/// 신호등
		else { //basic driving 
			angle = 1500-(tdata.angle/50)*500;
			angle = 0.5 * tdata.pre_angle + 0.5 * angle;
			/** printf("tdata.speed = %d\n", data->speed);//error */
			SteeringServoControl_Write(angle); 

			tdata.pre_angle = angle;//???
			
			DesireSpeed_Write(data->speed);
			if(data->speed == 0) usleep(100000);
			usleep(100000); //calibrate IO delay

		} 

		/** printf("data_driving_onoff = %d\n", data->driving_flag_onoff); */
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
		/** speed_ratio = 1; */
		/** is_Traffic_Light = OpenCV_green_Detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H); */
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

		// ---- sensor data input
		data->I_data_1 = DistanceSensor(1);
		data->O_data_1 = DistFunc(data->I_data_1);
		data->I_data_2 = DistanceSensor(2);
		data->O_data_2 = DistFunc(data->I_data_2);
		data->I_data_3 = DistanceSensor(3);
		data->O_data_3 = DistFunc(data->I_data_3);
		data->I_data_4 = DistanceSensor(4);
		data->O_data_4 = DistFunc(data->I_data_4);
		// ---- end

		// ---- mission trigger
		if(data->tunnelSignal == 1 && data->O_data_2 < 30) data->mission_id = 4;//tunnel
		/** if(data->ParkingSignal_2 == 0 && data->ParkingSignal_1 == 0 && data->O_data_2 < 30 && data->O_data_3 > 30) data->mission_id = 5;//parking */
		/** if(data->parParkingSignal_2 == 1 && data->parParkingSignal_1 == 0 && data->O_data_2 < 30 && data->O_data_3 > 30) data->mission_id = 6;//parparking */
		/** if(data->mission_state == BEFORE_PASSING_OVER && data->distance < 20) data->mission_id = 7;//passing master */
		if(is_Traffic_Light != 0) data->mission_id = 8; //traffic light


// -------------------- image process by capt ----------------------------------
		// ---- pky function
		double a, v;
		getSteeringWithLane(vpe->disp, capt, &a, &v);
		data->angle = a;
		data->speed = v;
		/** data->speed = 100; */
		// ---- pky end
		data->speed_ratio = color_detection(vpe->disp, capt);
		/** data->speed_ratio = 1;//test */
		data->speed = data->speed * data->speed_ratio;
		if(data->speed == 0) usleep(150000);
		printf("speedratio : %d\n", data->speed_ratio);
		printf("dataspeed : %d\n", data->speed);

// ----------------------- end of image process ----------------------------------

// -------------------------koo mission trigger---------------------
		// ---- 적외선 센서 ----
		data->distance = distance_sensor();
		printf("distance = 0x%04X(%d) \n", data->distance);

		// -------------------- capt로 이미지 처리 ----------------------------------
		if (stopLine_detect() == 1) data->stop_line = true;
		else data->stop_line = false; /// 여기서 트루 되었다가 바로 false로 바뀌면 chot됨

		// 여기서 data->mission_state로 던져줍니다

		if (data->mission_state == AUTO_DRIVE){
			/** data->angle = getSteeringWithLane(vpe->disp, capt); // 차선인식 <] */
			/** data->speed = color_detection(vpe->disp, capt); */
		}

		// 추월 미션 진입 트리거 원래 else if 라서 에러떴음
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
			/** data->angle = getSteeringWithLane(vpe->disp, capt); // 차선인식  <] */
			/** data->speed = color_detection(vpe->disp, capt); */
			data->yellow_stop_line = stop_line_detection(vpe->disp, capt, &data); // 정지선 인식하면 stop_line_recognition을 1로 return
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
//--------------------------koo mission trigger end-----------

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

void getSteeringWithLane(struct display *disp, struct buffer *cambuf, double *steer, double *speed) //detect lane
{
    double angle, ratio;
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);
        
        laneDetection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, &angle, &ratio); //get angle value from laneDetection()


        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
	*steer = angle;
    /** *speed = 130 *ratio; */
    *speed = 100;//test
}

static char* passing_master(struct display *disp, struct buffer *cambuf, void *arg){
    /** struct thr_data *data = (struct thr_data *)arg; */
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et; 
    /** signed short speed; */
    double distance;
    char* direction; // 갈 방향, left of right!
	/** printf("Koo is Passing master!!!!!!! wow!!!!!!\n"); */

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);
    
        direction = histogram_backprojection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }

    return direction;

}

int stopLine_detect(void) { /// 1 if stopline detected: the car is on white line
    sensor = LineSensor_Read();
    int whitecount = 0;
    for(i=0; i<8; i++){
        if (!sensor) whitecount++;
        sensor = sensor << 1;
    }
    printf("\n");
    printf("whitecount: %d\n", whitecount);
    if (whitecount > 3) return 1; /// white
    else return 0; /// black
}
