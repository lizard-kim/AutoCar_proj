
#include "koo_driving.h"


//passing은 추월이란 뜻 입니다!
// 추월에서만 쓰이는 함수들~
unsigned char status;
short speed;
unsigned char gain;
int position, posInit, posDes, posRead;
short angle;
int channel = 1;
int tol;
int data, distance; // distance 센서용
int i, j;
char sensor;
char byte = 0x80;
int line_buffer[8];


double distance_calculate(double data){
    double middle = data / 22691;
    double distance = pow(middle, -(1/1.07));
    return distance;
}

int distance_sensor(){
    printf("distance sensor start in koo_driving.c!!!\n");
    int distance = 1;
    while(1){
        data = DistanceSensor(channel);
        distance = distance_calculate(data);
        printf("channel = %d, distance = %d\n", channel, distance);
        return distance;
    }
}

void passing_go_back(){
    DesireSpeed_Write(-50);
    Winker_Write(LEFT_ON);
    usleep(800000);
    Winker_Write(ALL_OFF);
}

void passing_go_back2(){
    printf("뒤로 가자!\n");
    SpeedControlOnOff_Write(CONTROL);
    speed = -100;

    //control on/off
    PositionControlOnOff_Write(CONTROL);

    //position controller gain set
    gain = 30;
    PositionProportionPoint_Write(gain);

    //position write
    posInit = 0;  //initialize
    EncoderCounter_Write(posInit);
    
    //position set
    posDes = -800;
    position = posInit+posDes;
    DesireEncoderCount_Write(position);

    position=DesireEncoderCount_Read();
    printf("DesireEncoderCount_Read() = %d\n", position);
    
    tol = 100;    // tolerance
    while(abs(posRead-position)>tol)
    {
        posRead=EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        speed = DesireSpeed_Read();
        printf("DesireSpeed_Read() = %d \n", speed);
    }
    sleep(1);
    speed = 0;
    DesireSpeed_Write(speed);
    //mission_count ++;

    // 모드가 바뀔 때 빛과 소리가 납니다
    CarLight_Write(ALL_ON);
    Alarm_Write(ON);
    usleep(100000);
    Alarm_Write(OFF);
    CarLight_Write(ALL_OFF);
}
void passing_left(){
    DesireSpeed_Write(50);
    Winker_Write(LEFT_ON);
    SteeringServoControl_Write(2000);
    usleep(2000000); 
    SteeringServoControl_Write(1100);
    usleep(2700000);
    Winker_Write(ALL_OFF);    
}

void passing_right(){
    DesireSpeed_Write(50);
    Winker_Write(RIGHT_ON);
    SteeringServoControl_Write(1000);
    usleep(1800000); // 20
    SteeringServoControl_Write(1900);
    usleep(2700000); // 25
    Winker_Write(ALL_OFF);    
}

void passing_left_later(){
    printf("passing_left_later in !!!\n");
    DesireSpeed_Write(-50);
    printf("why22\n");
    usleep(2000000);
    DesireSpeed_Write(30);
    SteeringServoControl_Write(1900);
    usleep(3500000); //35
    SteeringServoControl_Write(1000);
    usleep(3500000);
    while(line_trace_sensor() == 1){  // black:1, white:0
        printf("in the while line_trace_sensor\n");
        printf("in the while line_trace_sensor\n");
        SteeringServoControl_Write(1500);
        usleep(10000);
    } 
}

void passing_right_later(){
    printf("passing_right_later in !!!\n");
    DesireSpeed_Write(-50); // -40
    printf("why\n");
    usleep(2000000); //27
    DesireSpeed_Write(30); // 40
    SteeringServoControl_Write(1100);
    usleep(3500000); // 35
    SteeringServoControl_Write(2000);
    usleep(3500000); // 35
    while(line_trace_sensor() == 1){ // black:1, white:0
        printf("in the while line_trace_sensor\n");
        printf("in the while line_trace_sensor\n");
        SteeringServoControl_Write(1500);
        usleep(10000);
    }
}

void passing_left2(){
    printf("왼쪽으로 가자!\n");

    SpeedControlOnOff_Write(CONTROL);

    speed = 70;
    /** DesireSpeed_Wirte(speed); */

    angle = 1850;
    SteeringServoControl_Write(angle);
    
    //control on/off
    PositionControlOnOff_Write(CONTROL);

    //position controller gain set
    gain = 30;
    PositionProportionPoint_Write(gain);

    //position write
    posInit = 0;  //initialize
    EncoderCounter_Write(posInit);
    
    //position set
    posDes = 1000;
    position = posInit+posDes;
    DesireEncoderCount_Write(position);

    position=DesireEncoderCount_Read();
    printf("DesireEncoderCount_Read() = %d\n", position);
    
    tol = 100;    // tolerance
    while(abs(posRead-position)>tol)
    {
        posRead=EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        speed = DesireSpeed_Read();
        printf("DesireSpeed_Read() = %d \n", speed);
    }
    sleep(1); 

    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!! 

    speed = 70;
    DesireSpeed_Write(speed);
    // 다시 원래 주행각도로 돌아오기 위해서
    angle = 1150;
    SteeringServoControl_Write(angle);
    usleep(1500000);    

    angle = 1500;
    SteeringServoControl_Write(angle);

    speed = 70;
    DesireSpeed_Write(speed);
}


void passing_right2(){
    printf("오른쪽으로 가자!\n");

    SpeedControlOnOff_Write(CONTROL);

    speed = 70;
    /** DesireSpeed_Wirte(speed); */

    angle = 1150;
    SteeringServoControl_Write(angle);
    
    //control on/off
    PositionControlOnOff_Write(CONTROL);

    //position controller gain set
    gain = 30;
    PositionProportionPoint_Write(gain);

    //position write
    posInit = 0;  //initialize
    EncoderCounter_Write(posInit);
    
    //position set
    posDes = 1000;
    position = posInit+posDes;
    DesireEncoderCount_Write(position);

    position=DesireEncoderCount_Read();
    printf("DesireEncoderCount_Read() = %d\n", position);
    
    tol = 100;    // tolerance
    while(abs(posRead-position)>tol)
    {
        posRead=EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        speed = DesireSpeed_Read();
        printf("DesireSpeed_Read() = %d \n", speed);
    }
    sleep(1); 

    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!! 

    speed = 70;
    DesireSpeed_Write(speed);
    // 다시 원래 주행각도로 돌아오기 위해서
    angle = 1850;
    SteeringServoControl_Write(angle);
    usleep(1500000);    

    angle = 1500;
    SteeringServoControl_Write(angle);

    speed = 70;
    DesireSpeed_Write(speed);
}

int line_trace_sensor(){
	/** printf("line_trace_sensor\n"); */
    int sum = 0;
    sensor = LineSensor_Read(); // black:1, white:0
    /** printf("LineSensor_Read() = "); */
        for(i=0; i<8; i++){
            if((sensor & byte)) {
				/** printf("1"); */
                line_buffer[i] = 1;
            }
            else {
				/** printf("0"); */
                line_buffer[i] = 0;
            }
            sensor = sensor << 1;
        }
	printf("\n");
    //printf("LineSensor_Read() = %d \n", sensor);
    for(j=0; j<8; j++){
        /** printf("LineSensor_Read() = %d\n", line_buffer[j]); */
        sum = sum + line_buffer[j];
    }
    if (sum > 2) return 1; // black:1, white:0
    else if (sum <= 2) {
		printf("==============black!!!================\n");
		return 0; //0111111이 검은색, 000000이 흰색이었음
		
	}
}

void passing_go_back_later(){
    printf("뒤로 가자!\n");
    SpeedControlOnOff_Write(CONTROL);
    speed = -100;

    //control on/off
    PositionControlOnOff_Write(CONTROL);

    //position controller gain set
    gain = 30;
    PositionProportionPoint_Write(gain);

    //position write
    posInit = 0;  //initialize
    EncoderCounter_Write(posInit);
    
    //position set
    posDes = -500;
    position = posInit+posDes;
    DesireEncoderCount_Write(position);

    position=DesireEncoderCount_Read();
    printf("DesireEncoderCount_Read() = %d\n", position);
    
    printf("passing_right_later in !!!\n");
    while(abs(posRead-position)>tol)
    {
        posRead=EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        speed = DesireSpeed_Read();
        printf("DesireSpeed_Read() = %d \n", speed);
    }
    sleep(1);
    speed = 0;
    /** DesireSpeed_Write(speed); */
    //mission_count ++;

    // 모드가 바뀔 때 빛과 소리가 납니다
    CarLight_Write(ALL_ON);
    Alarm_Write(ON);
    usleep(100000);
    Alarm_Write(OFF);
    CarLight_Write(ALL_OFF);
}

void passing_right_later2(){
    printf("오른쪽으로 가자!\n");

    SpeedControlOnOff_Write(CONTROL);

    speed = 70;
    /** DesireSpeed_Wirte(speed); */

    angle = 1150;
    SteeringServoControl_Write(angle);
    
    //control on/off
    PositionControlOnOff_Write(CONTROL);

    //position controller gain set
    gain = 30;
    PositionProportionPoint_Write(gain);

    //position write
    posInit = 0;  //initialize
    EncoderCounter_Write(posInit);
    
    //position set
    posDes = 800;
    position = posInit+posDes;
    DesireEncoderCount_Write(position);

    position=DesireEncoderCount_Read();
    printf("DesireEncoderCount_Read() = %d\n", position);
    
    tol = 100;    // tolerance
    while(abs(posRead-position)>tol)
    {
        posRead=EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        speed = DesireSpeed_Read();
        printf("DesireSpeed_Read() = %d \n", speed);
    }
    sleep(1); 

    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!! 

    speed = 70;
    DesireSpeed_Write(speed);

    // 다시 원래 주행각도로 돌아오기 위해서
    angle = 1850;
    SteeringServoControl_Write(angle);
    usleep(1500000);    
}


void passing_left_later2(){
    printf("왼쪽으로 가자!\n");

    SpeedControlOnOff_Write(CONTROL);

    speed = 70;
    /** DesireSpeed_Wirte(speed); */

    angle = 1850;
    SteeringServoControl_Write(angle);
    
    //control on/off
    PositionControlOnOff_Write(CONTROL);

    //position controller gain set
    gain = 30;
    PositionProportionPoint_Write(gain);

    //position write
    posInit = 0;  //initialize
    EncoderCounter_Write(posInit);
    
    //position set
    posDes = 800;
    position = posInit+posDes;
    DesireEncoderCount_Write(position);

    position=DesireEncoderCount_Read();
    printf("DesireEncoderCount_Read() = %d\n", position);
    
    tol = 100;    // tolerance
    while(abs(posRead-position)>tol)
    {
        posRead=EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        speed = DesireSpeed_Read();
        printf("DesireSpeed_Read() = %d \n", speed);
    }
    sleep(1); 

    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!! 

    speed = 70;
    DesireSpeed_Write(speed);

    // 다시 원래 주행각도로 돌아오기 위해서
    angle = 1850;
    SteeringServoControl_Write(angle);
    usleep(1500000);    
}

void passing_stop(){
    printf("멈춰!\n");
    angle = 1500;
    SteeringServoControl_Write(angle);
    speed = 0;
    DesireSpeed_Write(speed);

    // 모드가 바뀔 때 빛과 소리가 납니다
    CarLight_Write(ALL_ON);
    Alarm_Write(ON);
    usleep(100000);
    Alarm_Write(OFF);
    CarLight_Write(ALL_OFF);

    usleep(6000000);
}
