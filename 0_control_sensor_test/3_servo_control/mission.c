/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <stdio.h>
#include "car_lib.h"
#include <math.h>

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */
#define LIGHT_BEEP       // to test light and beep
#define POSITION_CONTROL  // to test postion control
#define SPEED_CONTROL     // to test speed control
#define SERVO_CONTROL     // to test servo control(steering & camera position)
#define LINE_TRACE              // to test line trace sensor
#define DISTANCE_SENSOR     // to test distance sensor

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
double distance_calculate(double data){
    double middle = data/22691;
    double distance = pow(middle, -(1/1.07));
    printf("distance : %lf\n", distance);
    return distance;
}

void main(void)
{
    unsigned char status;
    short speed;
    unsigned char gain;
    int position, posInit, posDes, posRead;
    short angle;
    int channel=1;
    int data;
    char sensor;
    int i, j;
    int tol;
    char byte = 0x80;

    CarControlInit();

    // 각도 초기화
    angle = SteeringServoControl_Read();
    printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

    angle = 1500;
    SteeringServoControl_Write(angle);
    sleep(1);
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    
    // 모드가 바뀔 때 빛과 소리가 납니다
    CarLight_Write(ALL_ON);
    Alarm_Write(ON);
    usleep(100000);
    Alarm_Write(OFF);
    CarLight_Write(ALL_OFF);

    int mission_count = 0;
    int distance;
    int speed_check;
    int count = 0;
    while(1){
        //0번은 직진 주행 모드
        while(mission_count == 0){
            printf("1번 미션 진입\n");
            data = DistanceSensor(channel);
            distance = distance_calculate(data);
            //SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
            speed = 150;
            DesireSpeed_Write(speed);
            if(distance > 15){
                while(1){        
                    printf("현재 기본주행 모드 입니다\n");
                    speed_check = DesireSpeed_Read();
                    printf("%d, %d\n", count, speed_check);

                    data = DistanceSensor(channel);
                    distance = distance_calculate(data);
                    if(distance < 15){
                        break;
                    }
                    count ++;
                }
            }
            else if(distance < 15) {
                mission_count ++; 
                speed = 0;
                DesireSpeed_Write(speed);
                break;
                }
            mission_count++;
        }
        // 모드가 바뀔 때 빛과 소리가 납니다
        CarLight_Write(ALL_ON);
        Alarm_Write(ON);
        usleep(100000);
        Alarm_Write(OFF);
        CarLight_Write(ALL_OFF);

        if(mission_count == 1){
            printf("2번 미션 진입\n");
            //jobs to be done beforehand;
            SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
            speed = -200; // speed set     --> speed must be set when using position controller
            DesireSpeed_Write(speed);

            //control on/off
            status = PositionControlOnOff_Read();
            printf("PositionControlOnOff_Read() = %d\n", status);
            PositionControlOnOff_Write(CONTROL);

            //position controller gain set
            gain = PositionProportionPoint_Read();    // default value = 10, range : 1~50
            printf("PositionProportionPoint_Read() = %d\n", gain);
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
            mission_count ++;

            // 모드가 바뀔 때 빛과 소리가 납니다
            CarLight_Write(ALL_ON);
            Alarm_Write(ON);
            usleep(100000);
            Alarm_Write(OFF);
            CarLight_Write(ALL_OFF);
        }

        if(mission_count == 2){
            printf("3번 미션 진입\n");
            //steer servo set
            angle = SteeringServoControl_Read();
            printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

            angle = 1150;
            SteeringServoControl_Write(angle);

            //jobs to be done beforehand;
            SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
            speed = 200; // speed set     --> speed must be set when using position controller
            DesireSpeed_Write(speed);

            //control on/off
            status = PositionControlOnOff_Read();
            printf("PositionControlOnOff_Read() = %d\n", status);
            PositionControlOnOff_Write(CONTROL);

            //position controller gain set
            gain = PositionProportionPoint_Read();    // default value = 10, range : 1~50
            printf("PositionProportionPoint_Read() = %d\n", gain);
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
            speed = 150;
            DesireSpeed_Write(speed);
            // 다시 원래 주행각도로 돌아오기 위해서
            angle = 1850;
            SteeringServoControl_Write(angle);
            usleep(1500000);

            angle = 1500;
            SteeringServoControl_Write(angle);
            mission_count = 0;

            // 모드가 바뀔 때 빛과 소리가 납니다
            CarLight_Write(ALL_ON);
            Alarm_Write(ON);
            usleep(100000);
            Alarm_Write(OFF);
            CarLight_Write(ALL_OFF);
        }
    }

}






