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
#define SERVO_CONTROL     // to test servo control(steering & Uamera position)
#define LINE_TRACE           // to test line trace sensor
#define DISTANCE_SENSOR     // to test distance sensor


#define FLAG_DYNAMIC_OBS 1
#define DYNAMIC_OBS_START 25
#define DYNAMIC_OBS_END 15

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */

unsigned char status;
short speed;
unsigned char gain;
int position, posInit, posDes, posRead;
short angle;
int channel;
int data;
char sensor;
int i, j;
int tol;
char byte = 0x80;

double DistFunc(double data)
{
    return pow(data/22691.0, -1/1.0706);
}


void dynamic_obs(void) {

    while (1) {
        int a;
        printf("if you want to exit, enter 5. \n");
        scanf("%d", &a);

        if (a == 5) {
            DesireSpeed_Write(0);
            SteeringServoControl_Write(1500);
            break;
        }

        DesireSpeed_Write(0);
        SteeringServoControl_Write(1500);

        printf("before while loop 1 ver 6\n");
        while (1) {
            double front_dist = DistFunc(DistanceSensor(1));
            if (front_dist < DYNAMIC_OBS_START) break;
        }
        printf("before while loop 2 and break \n");
        
        /// go straight
        DesireSpeed_Write(150);
        usleep(2*1000*1000);

        printf("before while loop 3 \n");
        DesireSpeed_Write(0);
        while (1) {
            double back_dist = DistFunc(DistanceSensor(4));
            if (back_dist < DYNAMIC_OBS_END) break;
        }
        //usleep();
        printf("before while loop 4 and break \n");

        /// rotation
        DesireSpeed_Write(200);
        SteeringServoControl_Write(2000);
        printf("before steering 9 sec\n");
        usleep(65*100*1000);
        printf("steering end\n");
        SteeringServoControl_Write(1500);
        usleep(15*100*1000);
        DesireSpeed_Write(0);

        printf("before while loop 5 \n");

        Alarm_Write(ON);
        usleep(100000);
        Alarm_Write(OFF);
        
    }

}

void main(void)
{    
    CarControlInit();
    PositionControlOnOff_Write(UNCONTROL);
    dynamic_obs();
    // check_linesensors();
    printf("function is terminated \n");
}

void check_linesensors(void) { /// for checking line sensor in while roop
    printf("before while roop");

    while (1) {
        int a;
        printf("Enter 5 if you want to execute linesensor");
        scanf("%d", &a);
        if (a == 5) {
            sensor = LineSensor_Read();        // black:1, white:0
            printf("LineSensor_Read() = ");
            for(i=0; i<8; i++)
            {
                if((i % 4) ==0) printf(" ");
                if((sensor & byte)) printf("1");
                else printf("0");
                sensor = sensor << 1;
                printf("LineSensor_Read() = %d \n", sensor);
            }
        }
        else {
            printf("while roop exit");
            break;
            }
        
    }
    printf("while roop end");
}