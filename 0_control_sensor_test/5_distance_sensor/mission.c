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
    printf("middle : %lf\n", middle);
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
    int channel;
    double data;
    char sensor;
    int i, j;
    int tol;
    char byte = 0x80;

    CarControlInit();

#ifdef DISTANCE_SENSOR
    // 5. distance sensor --------------------------------------------------------
    printf("distance sensor start!!!\n");
    scanf("%d", &channel);
    double distance;
    while(1){
        data = DistanceSensor(channel);
        printf("channel = %d, distance = %lf\n", channel, data);
        distance = distance_calculate(data);
        printf("%lf\n", distance);
        usleep(100000);
    }
    
#endif
}







