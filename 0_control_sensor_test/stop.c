#include <stdio.h>
#include "car_lib.h"

#define LIGHT_BEEP       // to test light and beep
#define POSITION_CONTROL  // to test postion control
#define SPEED_CONTROL     // to test speed control
#define SERVO_CONTROL     // to test servo control(steering & camera position)
#define LINE_TRACE              // to test line trace sensor
#define DISTANCE_SENSOR     // to test distance sensor


void main(void){
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

    CarControlInit();

    #ifdef SPEED_CONTROL
    // 2. speed control ----------------------------------------------------------

    //speed set    
    speed = DesireSpeed_Read();
    printf("DesireSpeed_Read() = %d \n", speed);
    speed = 0;
    DesireSpeed_Write(speed);

#endif
}
