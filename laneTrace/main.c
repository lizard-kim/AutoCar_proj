#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>
#include <math.h>
#include <time.h>
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

#define DYNAMIC_OBS_START 30
#define DYNAMIC_OBS_END 40
#define DYNAMIC_OBS_WAIT 40

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
	NEXT_AUTO_DRIVE,
    HISTOGRAM_BACK_PROPAGATION,
    BEFORE_PASSING_OVER, // 추월차로 미션 전의 임의의 미션
    PASSING_OVER_LEFT,
    PASSING_OVER_RIGHT,
    WAIT,
    PASSING_OVER_RETURN_LEFT,
    PASSING_OVER_RETURN_RIGHT,
    STOP,
    BREAK
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
	int forline;
	int ParkingSignal_1;
	int ParkingSignal_2;//after 수직주차 1
	int parParkingSignal_1;
	int parParkingSignal_2;
	int tunnelSignal;
	int tunnelcount;
	int tunnelend;
	/** int highway; // initial = 0, after finishing highway, it returns to 1 */

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
    int after_passing; // to judge whether passing mission
	//end

	// --- lizard
	int is_Traffic_Light;
	int is_Traffic_Light_for_traffic_light;

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
    int stop_line_DY; /// by dy: true 면 정지선 인식된거임
};
//parParkingSignal_2 = 2; ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int passing_where = -1; // 1 is left 2 is right
int passing = 0; // decide the time to passing other car

static int allocate_input_buffers(struct thr_data *data);
static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar);
static void draw_operatingtime(struct display *disp, uint32_t time);

void getSteeringWithLane(struct display *disp, struct buffer *cambuf, double *steer, double *speed);
void * capture_thread(void *arg);
/** void * capture_dump_thread(void *arg); */
/** void* input_thread(void *arg); */
void * sensor_thread(void *arg);
/** void * line_thread(void *arg); */
double distance_calculate(double data); // make sensor input data to real distance data
int distance_sensor(); //get sensor input data
static char* passing_master(struct display *disp, struct buffer *cambuf, void *arg); // 2019.11.16 관형 변경
static char* main_stop_line_detection(struct display *disp, struct buffer *cambuf);
void warmSensorTrigger(); 
static struct thr_data* pexam_data = NULL;
void signal_handler(int sig);
int stopLine_detect(void); // 정지선 인식하는 함수 1이면 정지선 위, 0이면 아님 by Doyeon
int line_trace_sensor();
double DistFunc(double data);
void DistanceTest();
void parparking(void *arg);
void parking(void *arg);
void tunnel_adv(void *arg);
void dynamic_obs_ver2(void *arg); // 
int dynamic_obs_ver3(void *arg);

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


    tdata.dump_state = DUMP_NONE;
    memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data)); // dump data를 0으로 채워서 초기화
	int start_sig = 0; //////////////////////////////////////////////////////////////////////////////////////////////

	//init data struct
	tdata.forline = 1; //0 white	
	tdata.mission_id = 0; // 0 is basic driving 1 is for testing
    tdata.driving_flag_onoff = true; /// by dy: true면 주행중, false면 주행종료
    tdata.pre_angle = 0;
    tdata.speed_ratio = 1; /// by dy: 태영이랑 도연이만 이 변수 건드릴 수 있음. 정지 표지판이나 회전교차로에서 정지해야하면 이 비율을 0으로 두기
	tdata.park = 0;//non
	/** tdata.highway = 0; // before highway, it is 0 and after highway it becomes 1 # highway_change */
	tdata.ParkingSignal_1 = 0;
	tdata.ParkingSignal_2 = 0;
	tdata.parParkingSignal_1 = 0;
	tdata.parParkingSignal_2 = 0;
	tdata.tunnelSignal = 0;
	tdata.tunnelend = 0;
	tdata.tunnelcount = 0;
	tdata.I_data_1 = 0;
	tdata.I_data_2 = 0;
	tdata.I_data_3 = 0;
	tdata.I_data_4 = 0;
	tdata.O_data_1 = 0;
	tdata.O_data_2 = 0;
	tdata.O_data_3 = 0;
	tdata.O_data_4 = 0;
	tdata.after_passing = 0; //0 is initial value, 1 is finished
    tdata.direction = "NONE"; // 추월 차로 진행 방향, left or right
    tdata.yellow_stop_line = "NONE"; // 정지선 인식 변수, 관형 추가
    tdata.white_stop_line = 1; // 정지선 인식 변수, 관형 추가 white = 0 black = 1
	tdata.mission_state = AUTO_DRIVE; // 여기서 mission_state를 HISTOGRAM_BACK_PROPAGATION로 설정함!!!
	tdata.is_Traffic_Light = 0; // 1 is left 2 is right
	tdata.is_Traffic_Light_for_traffic_light = 0; // 1 is red sign
    tdata.stop_line_DY = 0; // DY: stop line detected if first stop line is detected it will change to 1, second 2

	// ---------------- init data structure end -------------------

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

	ret = pthread_create(&tdata.threads[1], NULL, sensor_thread, &tdata);
	if(ret) MSG("Failed creating capture dump thread");
	pthread_detach(tdata.threads[1]);

	/** ret = pthread_create(&tdata.threads[2], NULL, line_thread, &tdata); */
	/** if(ret) MSG("Failed creating input thread"); */
	/** pthread_detach(tdata.threads[2]); */

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

	CarControlInit();
	SpeedControlOnOff_Write(CONTROL);
	PositionControlOnOff_Write(UNCONTROL);

	//speed controller gain set
	//P-gain
	gain = SpeedPIDProportional_Read();        // default value = 10, range : 1~50
	printf("SpeedPIDProportional_Read() = %d \n", gain);
	gain = 50;
	SpeedPIDProportional_Write(gain);

	//I-gain
	gain = SpeedPIDIntegral_Read();        // default value = 10, range : 1~50
	printf("SpeedPIDIntegral_Read() = %d \n", gain);
	gain = 50;
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
	camera_angle = 1680;//1650
	CameraYServoControl_Write(camera_angle);    

	//speed set
	check_speed = DesireSpeed_Read();  
	printf("DesireSpeed_Read() = %d \n", check_speed); 

	//for starting!
	/** DesireSpeed_Write(0); */
	SteeringServoControl_Write(1500);
	usleep(100000);
	int hightime = 0;

	while(start_sig == 0){
		printf("start_sig : %d\n", start_sig);
		if(data->O_data_1 < 10 && start_sig != 2) start_sig = 1;
		while(start_sig == 1){
			printf("O_data_1 : %d\n", data->O_data_1);
			if(data->O_data_1 > 10) start_sig = 2;
		}
		usleep(10000);
	}
	/** printf("start_sig : %d\n", start_sig); */

	while (start_sig == 2){
		/** printf("------------mission_id: %d\n", data->mission_id); */
		if (data->mission_id == 1) {//test driving
			/** Alarm_Write(ON); */
			/** usleep(100000); */
			/** Alarm_Write(OFF); */
			DesireSpeed_Write(0);
			/** usleep(1000000); */
		} 
		else if (data->mission_id == 3 ) { /// DY mission
			dynamic_obs_ver3(&tdata);
			data->tunnelSignal = 1;//[TODO] move it!
			data->mission_id = 0;
			data->stop_line_DY = 2;
		} 
		else if (data->mission_id == 4) {/// 터널
			tunnel_adv(&tdata);
			data->tunnelend = 1;
			data->mission_id = 7;// test driving edit it to 0
			CameraYServoControl_Write(1650);
			DesireSpeed_Write(0);
			usleep(100000);
		}  
		else if (data->mission_id == 5) {//수직
			parking(&tdata);
			/** data->tunnelSignal = 1; */
			data->mission_id = 0;// test driving edit it to 0
		} 
		else if (data->mission_id == 6) {//수평주차
			parparking(&tdata);
			data->mission_id = 0;// test driving edit it to 0
			//CameraYServoControl_Write(1600);data->after_passing
		} 
		else if (data->mission_id == 7 && data->after_passing == 0) {//passing master
			switch(data->mission_state){//[TODO] what is initial mission_state?
				// 기본주행 모드
				case AUTO_DRIVE : //koo_trigger
					DesireSpeed_Write(50);
					SteeringServoControl_Write(1500);
					usleep(2000000);
					SteeringServoControl_Write(1900);
					usleep(3000000);
					SteeringServoControl_Write(1500);
					usleep(100000);
					data->mission_state = NEXT_AUTO_DRIVE;

					/** CameraYServoControl_Write(1500); */
					break;     

				case NEXT_AUTO_DRIVE :
					DesireSpeed_Write(50);
					SteeringServoControl_Write(1500);
					usleep(100000);
					if(data->O_data_1 < 12){
						data->mission_state = PASSING_OVER_LEFT;
					}
				case HISTOGRAM_BACK_PROPAGATION :
					CameraYServoControl_Write(1500);
					SteeringServoControl_Write(1500);
					DesireSpeed_Write(50);
					usleep(100000);
					break;
					// 추월 미션 진입
				case PASSING_OVER_LEFT :
					CameraYServoControl_Write(1700);
					passing_go_back();
					passing_left();
					data->mission_state = WAIT;
					break;
				case PASSING_OVER_RIGHT :
					CameraYServoControl_Write(1700);
					passing_go_back();
					passing_right();
					data->mission_state = WAIT;
					break;
				case WAIT : 
					// 규열이의 차선주행
					DesireSpeed_Write(50);
					SteeringServoControl_Write(1500);
					printf("Speed : %d\n", DesireSpeed_Read());
					//usleep(1500000); // 이게 없으면 속력을 계속해서 주기 때문에 매우 낮은 속도로 감
					break;
				case PASSING_OVER_RETURN_RIGHT :
					//passing_go_back_later();
					passing_right_later();
					/** printf("finishing passing later\n"); */
					data->mission_state = STOP;
					break;
				case PASSING_OVER_RETURN_LEFT :
					//passing_go_back_later();
					passing_left_later();
					data->mission_state = STOP;
					break;
				case STOP :
					CameraYServoControl_Write(1500);
					passing_stop();
					data->mission_state = BREAK;
					data->after_passing = 1;
					break;
			}
		} /// 추월

		else if (data->mission_id == 8) {//[TODO] 튜닝

			/** camera_angle = 1500;//1650 delete it!!! */
			/** CameraYServoControl_Write(camera_angle);     */
			if(data->is_Traffic_Light_for_traffic_light == 0){
				DesireSpeed_Write(0);
				CarLight_Write(FRONT_ON);
				usleep(100000);
			}
			else if(data->is_Traffic_Light_for_traffic_light == 1 && data->is_Traffic_Light == 0){
				Alarm_Write(ON);
				SteeringServoControl_Write(1500); 
				usleep(100000);
				Alarm_Write(OFF);
				DesireSpeed_Write(0);
				usleep(9000000); // detect red sign, wait few time
				DesireSpeed_Write(120);
				usleep(940000);
				DesireSpeed_Write(0);
				usleep(1000000);
				data->is_Traffic_Light_for_traffic_light = 2;
			}

			else if(data->is_Traffic_Light_for_traffic_light >= 1 && data->is_Traffic_Light == 1){
				//go left
				/** printf("go left\n"); */
				DesireSpeed_Write(-100);
				usleep(500000);

				SteeringServoControl_Write(1950);
				DesireSpeed_Write(100);
				usleep(1700000);
				/** printf("step 1...\n"); */

				SteeringServoControl_Write(1500);
				DesireSpeed_Write(100);
				usleep(900000);
				/** printf("step 2...\n"); */

				/** printf("traffic light finished..!!!\n"); */
				DesireSpeed_Write(0); //E-Stop;

				Alarm_Write(ON);
				usleep(1000000);
				Alarm_Write(OFF);
				break;
			}
			else if(data->is_Traffic_Light_for_traffic_light >= 1 && data->is_Traffic_Light == 2){
				//right
				/** printf("go right\n"); */
				DesireSpeed_Write(-100);
				usleep(500000);

				SteeringServoControl_Write(1050);
				DesireSpeed_Write(100);
				usleep(1700000);
				/** printf("step 1...\n"); */

				SteeringServoControl_Write(1500);
				DesireSpeed_Write(100);
				usleep(900000);
				/** usleep(1000000); */
				/** printf("step 2...\n"); */

				/** printf("traffic light finished..!!!\n"); */
				DesireSpeed_Write(0); //E-Stop;
				Alarm_Write(ON);
				usleep(1000000);
				Alarm_Write(OFF);
				break;
			}
			/** else{ */
			/**     printf("ERROR!!!!\n"); */
			/**     break; */
			/** } */

		}	/// 신호등
		else { //basic driving 
			angle = 1500-(tdata.angle/50)*500;
			angle = 0.5 * tdata.pre_angle + 0.5 * angle;
			/** printf("tdata.speed = %d\n", data->speed);//error */
			SteeringServoControl_Write(angle); 
			tdata.pre_angle = angle;//???
			DesireSpeed_Write(data->speed);
			/** DesireSpeed_Write(100); */
			if(data->speed == 0) usleep(500000);
			usleep(50000);
		} 

		/** printf("data_driving_onoff = %d\n", data->driving_flag_onoff); */
		if (data->driving_flag_onoff == false) /// mission end, the car will be stopped.
		{//신호등 이후에 하자
			/// 추후에 이 코드 종료 미션에 맞게 바꿀 것
			DesireSpeed_Write(0);
			/** Alarm_Write(ON); */
			/** usleep(1000000); */
			/** Alarm_Write(OFF); */
			break;

		}
	}
	pause();
	return ret;
}

signed short color_detection(struct display *disp, struct buffer *cambuf)
{
	unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
	uint32_t optime;
	struct timeval st, et;
	signed short speed_ratio = 0;

	unsigned char* cam_pbuf[4];
	if(get_framebuf(cambuf, cam_pbuf) == 0) {
		memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);
		/** gettimeofday(&st, NULL); */
		speed_ratio = OpenCV_red_Detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
		/** gettimeofday(&et, NULL); */
		/** optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000); */
		/** draw_operatingtime(disp, optime); */
	}

	return speed_ratio;
}

int Traffic_mission(struct display *disp, struct buffer *cambuf){
	unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
	uint32_t optime;
	struct timeval st, et;
	int is_Traffic_Light_for_traffic_light = 0;

	unsigned char* cam_pbuf[4];
	if(get_framebuf(cambuf, cam_pbuf) == 0) {
		memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);
		/** gettimeofday(&st, NULL); */
		is_Traffic_Light_for_traffic_light = OpenCV_red_Detection_for_traffic_light(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
		// 1 is red sign
		/** gettimeofday(&et, NULL); */
		/** optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000); */
		/** draw_operatingtime(disp, optime); */
	}
	return is_Traffic_Light_for_traffic_light; 
}

int Traffic_mission_green(struct display *disp, struct buffer *cambuf){
	unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
	uint32_t optime;
	struct timeval st, et;
	int is_Traffic_Light = 0;

	unsigned char* cam_pbuf[4];
	if(get_framebuf(cambuf, cam_pbuf) == 0) {
		memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);
		/** gettimeofday(&st, NULL); */
		is_Traffic_Light = OpenCV_green_Detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
		// 1 is red sign
		/** gettimeofday(&et, NULL); */
		/** optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000); */
		/** draw_operatingtime(disp, optime); */
	}
	return is_Traffic_Light; 
}

void * capture_thread(void *arg)
{
	/** printf("capture_thread\n"); */
	usleep(100000);
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

	double a, v;
	int temp = 1;

	while(1) {
		while(data->mission_id == 4) usleep(100000);
		// printf("capture_thread!\n");
		////////////////////////////do not touch////////////////////////////////////
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
		///////////////////////////////////////////////////////////////////////////

		// ------------- trigger ----------------
		/** if (data->highway == 0) { data->mission_id = 2; } /// #highway_changed : at first, this car start at highway */
		/** else { */
		if( data->stop_line_DY == 1 ) { 
			data->mission_id = 3; /// [TODO] 준호 주차 미션이 끝나면 주행 가능하도록 바꿀 것
			/** data->stop_line_DY = 2; */
		}
		else{
			// 4 tunnel trigger 
			// printf("data->tunnelSignal : %d\n", data->tunnelSignal);
			if(data->tunnelend == 0 && data->tunnelSignal == 1 && data->O_data_2 < 20 && data->O_data_6 < 18){
				data->tunnelSignal = 2;
			}
			if(data->tunnelSignal == 2 && (data->O_data_2 > 20 || data->O_data_6 > 18)){
				data->tunnelSignal = 1;
			}
			if(data->tunnelSignal == 2 && data->O_data_2 < 20 && data->O_data_6 < 18){
				data->tunnelcount +=1;

				if(data->tunnelcount > 1){
					data->tunnelSignal = 3;
				}
			}
			if(data->tunnelSignal == 3 && (data->O_data_2 > 20 || data->O_data_6 > 18)){
				data->tunnelSignal = 1;
				data->tunnelcount = 0;
			}
			if(data->tunnelend == 0 && data->tunnelSignal == 3 && data->O_data_2 < 20 && data->O_data_6 < 18){
				data->mission_id = 4;
			}

			// 5 parking trigger
			if(data->parParkingSignal_2 == 2 && data->ParkingSignal_2 == 0 && data->ParkingSignal_1 == 0 && data->O_data_2 < 30 && data->O_data_3 > 30){
				printf("step1\n");
				data->ParkingSignal_1 = 1;
			}
			if(data->ParkingSignal_1 == 1 && data->O_data_2 > 30 && data->O_data_3 < 30){
				printf("step3\n");
				data->ParkingSignal_1 = 2;
			}
			if(data->ParkingSignal_1 == 2 && data->O_data_2 < 30 && data->O_data_3 > 30) data->ParkingSignal_1 = 0;
			if(data->ParkingSignal_1 == 2 && (data->O_data_2 > 200 || data->O_data_3 > 200)) data->ParkingSignal_1 = 0;
			if(data->ParkingSignal_1 == 2 && data->O_data_2 > 30 && data->O_data_3 > 30){
				printf("step4\n");
				data->ParkingSignal_1 = 3;
			}
			if(data->ParkingSignal_1 == 3 && data->O_data_2 < 30 && data->O_data_3 > 30) data->ParkingSignal_1 = 4;
			if(data->ParkingSignal_2 == 0 && data->ParkingSignal_1 == 4 && data->O_data_3 < 30){
				printf("step5\n");
				data->ParkingSignal_2 = 2;
				data->mission_id = 5;// test driving edit it to 0
			}

			// 6 parparking trigger
			if(data->parParkingSignal_2 == 0 && data->parParkingSignal_1 == 0 && data->O_data_2 < 30 && data->O_data_3 > 30){
				printf("step1\n");
				data->parParkingSignal_1 = 1;
			}
			/*
			   if(data->parParkingSignal_1 == 1 && data->O_data_2 > 60 && data->O_data_3 > 60){
			   printf("step2\n");
			   data->parParkingSignal_1 = 0;
			   }*/
			if(data->parParkingSignal_1 == 1 && data->O_data_2 > 30 && data->O_data_3 < 30){
				printf("step3\n");
				data->parParkingSignal_1 = 2;
			}
			if(data->parParkingSignal_1 == 2 && (data->O_data_2 > 200 || data->O_data_3 > 200)) data->parParkingSignal_1 = 0;
			if(data->parParkingSignal_1 == 2 && data->O_data_2 < 30 && data->O_data_3 > 30) data->parParkingSignal_1 = 0;
			if(data->parParkingSignal_1 == 2 && data->O_data_2 > 30 && data->O_data_3 > 30){
				printf("step4\n");
				data->parParkingSignal_1 = 3;
			}
			if(data->parParkingSignal_1 == 3 && data->O_data_2 < 30 && data->O_data_3 > 30) data->parParkingSignal_1 = 4;
			if(data->parParkingSignal_2 == 0 && data->parParkingSignal_1 == 4 && data->O_data_3 < 30){
				printf("step5\n");
				data->mission_id = 6;// test driving edit it to 0
			}

			if(data->parParkingSignal_2 == 2 && data->tunnelend == 1 && data->O_data_1 < 80) { 
				/** pri\cntf("I love you!!!!!!!!!!!!!!!QQQQQQQQQQQqqqqqqqqqqqqqqqq!\n"); */
				data->mission_id = 7;//passing master /////////////////////////////////////////////////
			}
			// 8 traffic light trigger
			if(data->after_passing == 1 && data->mission_id == 7) data->mission_id = 8; //traffic light

			// -------------------- image process by capt ----------------------------------
			// ---- pky function
			getSteeringWithLane(vpe->disp, capt, &a, &v);
			data->angle = a;
			// data->speed = v;
			// ---- pky end

			if(data->mission_id != 8) {
				data->speed_ratio = color_detection(vpe->disp, capt);
				if(data->speed_ratio == 0){
					temp = 0;		
				}
				else if(temp < data->speed_ratio){
					CameraYServoControl_Write(1750);
					usleep(10000);
				}
			}
			else{
				if(data->is_Traffic_Light_for_traffic_light == 0){
					data->is_Traffic_Light_for_traffic_light = Traffic_mission(vpe->disp, capt);//red sign
					/** data->red = 1; */
					usleep(12000000);
				}
				else {
					data->is_Traffic_Light = Traffic_mission_green(vpe->disp, capt); //green sign
				}
				usleep(20000);
				/** printf("capture_thread is_Traffic_Light : %d\n", data->is_Traffic_Light); */
			}
			data->speed = v * data->speed_ratio;
			// if(data->speed == 0) usleep(50000);
			// usleep(10000); 
			// ----------------------- end of image process ----------------------------------

			// -------------------------koo mission trigger---------------------
			if (data->mission_id == 7 && data->after_passing == 0){
				// ---- 적외선 센서 ----

				// -------------------- capt로 이미지 처리 ------------------0x%04Xfalse로 바뀌면 chot됨

				// 여기서 data->mission_state로 던져줍니다
				//	if (data->mission_state == AUTO_DRIVE && data->O_data_1 < 60){
				//		data->mission_state = HISTOGRAM_BACK_PROPAGATION;
				//}

				// 추월 미션 진입 트리거 원래 else if 라서 에러떴음
				if (data->mission_state == HISTOGRAM_BACK_PROPAGATION){
					data->direction = passing_master(vpe->disp, capt, &data);
					if (data->O_data_1< 55){
						data->mission_state = BEFORE_PASSING_OVER;
					}
				}

				else if (data->mission_state == BEFORE_PASSING_OVER && data->O_data_1 < 12){
					if (strcmp(data->direction, "left")==0){
						data->mission_state = PASSING_OVER_LEFT;
					}
					else if (strcmp(data->direction, "right")==0){
						data->mission_state = PASSING_OVER_RIGHT;
					}
					else if (strcmp(data->direction,"fail")==0){
						data->mission_state = PASSING_OVER_RIGHT; // 만일 역히스토그램 투영으로 방향을 도출해내지 못하면 왼쪽으로 간다고 설정
					}
				}    

				// 추월 이후 정지선을 인식할 때까지 차선 인식
				if (data->mission_state == WAIT){ // WAIT은 불가피하게 main에서 바꾸어준다
					data->yellow_stop_line = main_stop_line_detection(vpe->disp, capt); // 정지선 인식하면 stop_line_recognition을 1로 return
				}
				// 정지선을 인식하면 다시 차로로 return
				if (data->mission_state == WAIT && strcmp(data->yellow_stop_line, "stop")==0){
					/** printf(" ########### data->direction in capture thread = %s\n", data->direction); */

					// 정지선 인식 신호
					CarLight_Write(ALL_ON);
					Alarm_Write(ON);
					usleep(100000);
					Alarm_Write(OFF);
					CarLight_Write(ALL_OFF);

					if (strcmp(data->direction, "left") == 0 || strcmp(data->direction, "fail") == 0){
						data->mission_state = PASSING_OVER_RETURN_RIGHT;
					}
					else if (strcmp(data->direction, "right") == 0){
						data->mission_state = PASSING_OVER_RETURN_LEFT;
					}
					/** } */
				else if (data->mission_state == PASSING_OVER_RETURN_RIGHT || data->mission_state == PASSING_OVER_RETURN_LEFT){
				}

			}
		}

		//--------------------------koo mission trigger end-----------
	}
	// input video data to disp_buf
	if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height)) {
		error("post buffer failed");
		return NULL;
	}
	update_overlay_disp(vpe->disp); // diplay overay plane                                       
	vpe_output_qbuf(vpe, index); // VPE 출력 큐 처리 권한을 드라이버에게 이전
	index = vpe_input_dqbuf(vpe); // VPE 입력 큐 처리 권한을 어플리케이션이 가지고 옴
	v4l2_qbuf(v4l2, vpe->input_buf_dmafd[index], index); // 영상 큐 처리 권한을 드라이버에게 이전하여 다음 영상 프레임 요청
	}
	MSG("Ok!");
	return NULL;
}


void * sensor_thread(void *arg)
{
	struct thr_data *data = (struct thr_data *)arg;
	/** usleep(10000); */
	while(1) {
		// ---- sensor data input
		usleep(10000);
		/** printf("odata1: %d\n", data->O_data_1); */
		data->I_data_1 = DistanceSensor(1);
		data->O_data_1 = DistFunc(data->I_data_1);
		data->I_data_2 = DistanceSensor(2);
		data->O_data_2 = DistFunc(data->I_data_2);
		data->I_data_3 = DistanceSensor(3);
		data->O_data_3 = DistFunc(data->I_data_3);
		data->I_data_4 = DistanceSensor(4);
		data->O_data_4 = DistFunc(data->I_data_4);
		data->I_data_5 = DistanceSensor(5);
		data->O_data_5 = DistFunc(data->I_data_5);
		data->I_data_6 = DistanceSensor(6);
		data->O_data_6 = DistFunc(data->I_data_6);
		/** if(data->mission_id == 7) break;//no sensor */
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

		DesireSpeed_Write(0);
		usleep(10000);
		SteeringServoControl_Write(1500);
		usleep(10000);
		pthread_cancel(pexam_data->threads[0]);
		pthread_cancel(pexam_data->threads[1]);
		/** pthread_cancel(pexam_data->threads[2]); */

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

        /** printf("-- laneTrace End ... pky --\n"); */
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
	*speed = ratio;
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
		direction = "left";// koo_trigger */
		
		gettimeofday(&et, NULL);
		optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
		draw_operatingtime(disp, optime);
	}

	return direction;

}

void parking(void *arg)
{
	/** struct thr_data* data; */
	struct thr_data *data = (struct thr_data *)arg;
	int meandist;	
	printf("parking!!!!\n");
	/** printf("%d\n", data->O_data_3); */
	DesireSpeed_Write(100);
	if(data->ParkingSignal_2 == 1)
	{
		return;
	}

	CarLight_Write(REAR_ON);

	DesireSpeed_Write(100);
	usleep(200000+(meandist*3000)); //210000
	printf("wtf3\n");
	DesireSpeed_Write(0);
	usleep(500000);
	SteeringServoControl_Write(1050);
	EncoderCounter_Write(0);
	DesireSpeed_Write(-100);
	int posRead = EncoderCounter_Read();
	while(1)
	{
		if(data->O_data_4 < 7 || EncoderCounter_Read() < -1300) // or EncoderCounter_Read == -6000; it means car moved 30cm.
		{
			DesireSpeed_Write(0);
			SteeringServoControl_Write(1500);
			break;
		}
	}
	printf("wtf1\n");
	meandist = data->O_data_3;
	usleep(200000);
	SteeringServoControl_Write(1500);
	while(1)
	{
		int posRead_1 = data->O_data_4;
		printf("DIST4 SENSOR = %d\n", posRead_1);

		if(data->O_data_4 > 13)
		{
			DesireSpeed_Write(-50);
		}
		else
		{
			DesireSpeed_Write(0);
			break;
		}
	}

	/** while(1) */
	/** { */
	/**     if(data->O_data_4 > 13) */
	/**     {//this condition is weird */
	/**         DesireSpeed_Write(-50); */
	/**         usleep(10000); */
	/**     } */
	/**     else */
	/**     { */
	/**         DesireSpeed_Write(0); */
	/**         break; */
	/**     } */
	/**     usleep(500000); */
	/** } */
	printf("wtf2\n");

	DesireSpeed_Write(0);
	usleep(500000); //1000000


	Alarm_Write(ON);
	usleep(500000);
	Alarm_Write(OFF);


	SteeringServoControl_Write(1500);
	DesireSpeed_Write(100);
	usleep(300000);
	EncoderCounter_Write(0);

	SteeringServoControl_Write(1001);
	DesireSpeed_Write(100);
	usleep(1400000);
	SteeringServoControl_Write(1500);

	DesireSpeed_Write(0); //E-Stop;
	data->ParkingSignal_1 = 0;
	CarLight_Write(ALL_OFF);
	data->stop_line_DY = 1;
}


void parparking(void *arg)
{
	/** struct thr_data* data; */
	struct thr_data *data = (struct thr_data *)arg;
	int tmpdist_1 =  0;
	int tmpdist = 0;

	DesireSpeed_Write(100);

	CarLight_Write(REAR_ON);
	tmpdist_1 = data->O_data_2;
	if(tmpdist_1 >= 15)
	{
		tmpdist = tmpdist_1;
	}
	usleep(210000+(tmpdist*5000));

	DesireSpeed_Write(0);
	usleep(500000);

	SteeringServoControl_Write(1000);
	EncoderCounter_Write(0);
	DesireSpeed_Write(-50);
	usleep(1500000+(tmpdist*5000)); //1300000

	int posRead_1 = EncoderCounter_Read();
	//printf("EncoderCounter_Read() = %d\n", posRead_1);

	//printf("-600 reached\n");
	SteeringServoControl_Write(1500);
	usleep(1400000); //1800000
	//printf("-800 reached\n");
	SteeringServoControl_Write(2000);
	usleep(50000); //1800000

	while(1)
	{
		if(data->O_data_4 > 10 && data->O_data_3 > 4)
		{//this condition is weird
			DesireSpeed_Write(-80);
		}
		else
		{
			DesireSpeed_Write(0);
			break;
		}
		usleep(10000);
	}

	SteeringServoControl_Write(1500);

	while(1)
	{
		int posRead_1 = data->O_data_4;
		printf("DIST4 SENSOR = %d\n", posRead_1);

		if(data->O_data_4 > 10)
		{
			DesireSpeed_Write(-50);
		}
		else
		{
			DesireSpeed_Write(0);
			break;
		}
	}

	Alarm_Write(ON);
	usleep(500000);
	Alarm_Write(OFF);

	usleep(500000); //1000000

	SteeringServoControl_Write(1999);
	DesireSpeed_Write(80);
	usleep(300000);
	//printf("step 1...\n");

	SteeringServoControl_Write(1500);
	DesireSpeed_Write(-80);
	usleep(300000);
	//printf("step 1...\n");

	SteeringServoControl_Write(1999);
	DesireSpeed_Write(80);
	usleep(700000);
	//printf("step 1...\n");

	SteeringServoControl_Write(1500);
	usleep(320000);

	SteeringServoControl_Write(1000);
	usleep(1300000);
	//printf("step 2...\n");

	SteeringServoControl_Write(1500);
	usleep(500000);

	//printf("Basic Mode is ready...Parking finished..!!!\n");
	//printf("tmpdist : %d\n", tmpdist);
	DesireSpeed_Write(0); //E-Stop;
	data->parParkingSignal_1 = 0;
	data->parParkingSignal_2 = 2;
	CarLight_Write(ALL_OFF);


	//return 0;
}

double DistFunc(double data)
{
	double I_Dist = data;
	double O_Dist;
	O_Dist = pow(I_Dist/22691.0, -1/1.0706);
	/** printf("%.0lf\n", O_Dist); */
	return O_Dist;
}

void tunnel_adv(void *arg)
{
	struct thr_data *data = (struct thr_data *)arg;
	int angle;
	int k = 0;
	Alarm_Write(ON);
	usleep(100000);
	Alarm_Write(OFF);
	int speed;
	speed = 80;
	while(1)
	{
		/** printf("2: %d, 3: %d, 5: %d\n", data->O_data_2, data->O_data_3, data->O_data_5); */
		DesireSpeed_Write(speed);
		usleep(10000);
		if(k == 0)CarLight_Write(FRONT_ON);
		k++;
		if(data->O_data_2 >= 13)
		{
			angle = -71*data->O_data_2 + 2210;
			if(angle < 1010)
			{
				angle = 1001;
			}
			/** if(data->O_data_2 >= 40) angle = 1500; */
			SteeringServoControl_Write(angle);

			DesireSpeed_Write(speed);
			usleep(10000);
		}
		else if(data->O_data_2 < 13) // 10--> 1500 5 --> 2000            y = -31x + 1900
		{
			angle = -100*data->O_data_2 + 2500;
			if(angle > 1990)
			{
				angle = 1999;
			}
			SteeringServoControl_Write(angle);
			usleep(50000);
		}
		if(data->O_data_6 > 19 && data->O_data_1 > 110){
			angle = 1500;
			SteeringServoControl_Write(angle);
			usleep(50000);
			CarLight_Write(0);
			/** printf("ddddddddddddddddddddddddddddddTunnelofff\n"); */
			data->tunnelend = 1;
			break;
		}
	} 
	/** usleep(100000); */
}

void DistanceTest()
{
	struct thr_data* data;
	CarLight_Write(FRONT_ON);
	/** data->I_data_5 = DistanceSensor(5); */
	/** data->O_data_5 = DistFunc(data->I_data_5); */
	/** data->I_data_6 = DistanceSensor(6); */
	/** data->O_data_6 = DistFunc(data->I_data_6); */

	//Distance between car and wall is each 10cm.
	// 10 --> 1500 , 20 --> 1230            y = -27x + 1770
	/** printf("O_data_6 : %d\n", O_data_6 ); */
	usleep(50000);
}

int dynamic_obs_ver3(void *arg) {
	struct thr_data *data = (struct thr_data *)arg;
	double angle1;
	while (1) { /// go laneTracing before stop line detected
		angle1 = 1500-(data->angle/50)*500;
		angle1 = 0.5 * data->pre_angle + 0.5 * angle1;
		// printf("tdata.speed = %d\n", data->speed); //error
		SteeringServoControl_Write(angle1);
		data->pre_angle = angle1;
		DesireSpeed_Write(data->speed);
		if (line_trace_sensor() == 0) break;
	}
	DesireSpeed_Write(0);
	SteeringServoControl_Write(1500);
	Alarm_Write(ON);
	usleep(500*1000); // mission is over alarm signal */
	Alarm_Write(OFF); //
	printf("I'm here00-------------------------\n ");
	int testing;
	while (1) { /// 앞에서 차가 지나갈 때 까지 기다림
		usleep(1000*300);
		// printf("data O data 1 in dynamic obs : %d\n", data->O_data_1);
		if (data->O_data_1 < DYNAMIC_OBS_START) {
			break;
		}
	}
	printf("I am here0----------------------------ver6 \n");
	while (1) {
		usleep(1000*300);
		//printf("here0 data: %d\n", data->O_data_1);
		if (data->O_data_1 > DYNAMIC_OBS_WAIT) break;
	}
	int a = 0;
	double angle;
	while (a<15) { // go straight with pky function
		/** angle = 1500-(data->angle/50)*500; */
		angle = 1500;
		/** angle = 0.5 * data->pre_angle + 0.5 * angle; */
		/** printf("tdata.speed = %d\n", data->speed);//error */
		SteeringServoControl_Write(angle);
		/** data->pre_angle = angle; */
		DesireSpeed_Write(50);
		usleep(1000*200);
		//SteeringServoControl_Write(data->angle);
		//printf("angle: %i\n", data->angle);
		//usleep(1000*50); // in usleep, 1000 * 1000 is 1 second
		a++; // [TODO] 대회장에서 규열이 함수가 충분히 원형교차로 빠져나올 수 있을 수준으로 a 컨트롤하기
	}
	DesireSpeed_Write(0);
	printf("I am here2---------------------------- \n");
	usleep(1000*1000*19);
	// printf("here3 data: %d\n", data->O_data_4);
	/// 뒤에서 차가 따라옴. Dynamic obs end 수치 이하일때
	/** printf("I am here3-------------------------- \n"); */
	/*
	   int b = 0;
	   double angle2;
	   while (b<10) { // lane tracing part
	   angle2 = 1500-(data->angle/50)*500;
	   angle2 = 0.5 * data->pre_angle + 0.5 * angle2;
	   SteeringServoControl_Write(angle2);
	   data->pre_angle = angle2;//???
	   DesireSpeed_Write(data->speed);
	   usleep(1000*200);
	   b++; // [TODO] 대회장에서 규열이 함수가 충분히 원형교차로 빠져나올 수 있을 수준으로 a 컨트롤하기
	   }
	   printf("I am here4-----------------------------%d\n", a);
	   printf("ver 3\n");
	   usleep(1000*1000); /// [TODO] 원형교차로 빠져나온 뒤 직선주행 잠깐 하기
	   Alarm_Write(ON);
	   usleep(500*1000);
	   Alarm_Write(OFF);
	   */
	data->mission_id = 0;
	return 0;
}

void dynamic_obs_ver2(void *arg) {

    DesireSpeed_Write(0);
    SteeringServoControl_Write(1500);
    Alarm_Write(ON);
	usleep(500*1000); // mission is over alarm signal */
	Alarm_Write(OFF); //
    struct thr_data *data = (struct thr_data *)arg;
    printf("I'm here00-------------------------\n ");
    int testing;
    while (1) { /// 앞에서 차가 지나갈 때 까지 기다림
        usleep(1000*300);
        if (data->O_data_1 < DYNAMIC_OBS_START) {
            break;
        }
        
    }
    printf("I am here0----------------------------ver6 \n");
    
    while (1) {
        usleep(1000*300);
        //printf("here0 data: %d\n", data->O_data_1);
        if (data->O_data_1 > DYNAMIC_OBS_WAIT) break;
        
    }
    usleep(100*1000); // after 1/10 second wait(until front car is go to far enough)
    printf("I am here1---------------------------- \n");
    /// go straight
    DesireSpeed_Write(150);
    usleep(1200*1000);

    DesireSpeed_Write(0);  
    printf("I am here2---------------------------- \n");
    while (1) {
        usleep(1000*300);
        //printf("here3 data: %d\n", data->O_data_4);
        if (data->O_data_4 < DYNAMIC_OBS_END) break;
    }
    /// 뒤에서 차가 따라옴. Dynamic obs end 수치 이하일때
    printf("I am here3-------------------------- \n");

    int a = 0;
    double angle;
    while (a<28) { // lane tracing part
    	angle = 1500-(data->angle/50)*500;
		angle = 0.5 * data->pre_angle + 0.5 * angle;
		/** printf("tdata.speed = %d\n", data->speed);//error */
		SteeringServoControl_Write(angle); 
		data->pre_angle = angle;//???
        DesireSpeed_Write(data->speed);
        usleep(1000*200);
        //SteeringServoControl_Write(data->angle);
        //printf("angle: %i\n", data->angle);
        //usleep(1000*50); // in usleep, 1000 * 1000 is 1 second
        
        a++; // [TODO] 대회장에서 규열이 함수가 충분히 원형교차로 빠져나올 수 있을 수준으로 a 컨트롤하기
    }

    printf("I am here4-----------------------------%d\n", a);

    SteeringServoControl_Write(1300);
    DesireSpeed_Write(100);
    usleep(1000*1000); /// [TODO] 원형교차로 빠져나온 뒤 직선주행 잠깐 하기
    Alarm_Write(ON);
    usleep(500*1000);
    Alarm_Write(OFF);
    Alarm_Write(ON);
    usleep(500*1000);
    Alarm_Write(OFF);
    data->mission_id = 0;
}
static char* main_stop_line_detection(struct display *disp, struct buffer *cambuf) //detect lane
{
    //double angle, ratio;
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;
    char* answer;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);
        gettimeofday(&st, NULL);
        answer = stop_line_detection(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H); //get angle value from laneDetection()
        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
	//*steer = angle;
    /** *speed = 130 *ratio; */
    //*speed = 100;//test
    return answer;
}


int stopLine_detect(void) { /// 1 if stopline detected: the car is on white line
	int i;
	unsigned char sensor;
	sensor = LineSensor_Read();
	int whitecount = 0;
	for(i=0; i<8; i++){
		if (!sensor) whitecount++;
		sensor = sensor << 1;
	}

	// printf("\n");
	// printf("whitecount: %d\n", whitecount);
	if (whitecount <= 5) {
		printf("black\n");
		return 1; /// black
	}
	else return 0; /// white
}
