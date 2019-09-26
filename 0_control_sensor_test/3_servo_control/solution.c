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

    int speed_check;
    int count = 0;
    // 각도 초기화
    angle = SteeringServoControl_Read();
    printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

    angle = 1100;
    SteeringServoControl_Write(angle);
    sleep(1);

    angle = 1800;
    SteeringServoControl_Write(angle);
    sleep(1);

    angle = 1500;
    SteeringServoControl_Write(angle);
    sleep(1);

    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
    speed = 50;
    DesireSpeed_Write(speed);
    while(1){        
        printf("속도 150을 주었을 때\n");
        speed_check = DesireSpeed_Read();
        printf("%d, 속도 : %d\n", count, speed_check);
        count ++;
        if(count > 200){
            break;
        }
    }

    count = 0;
    speed = 100;
    DesireSpeed_Write(speed);
    while(1){        
        printf("속도 200을 주었을 때\n");
        speed_check = DesireSpeed_Read();
        printf("%d, 속도 : %d\n", count, speed_check);
        count ++;
        if(count > 200){
            break;
        }
    }
    
    printf("엔코더 진입\n");
     // 1. position control -------------------------------------------------------
    printf("\n\n 1. position control\n");

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
    }
    sleep(1);
}
