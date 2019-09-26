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

    //speed read  
    speed = DesireSpeed_Read();
    printf("DesireSpeed_Read() = %d \n", speed);

    //speed initalization
    speed = 0;
    DesireSpeed_Write(speed);
    sleep(1);

    //speed set
    printf("speed를 100으로 주겠습니다!");
    speed = 100;
    DesireSpeed_Write(speed);
    signed short speed_check;
    int count = 0;
    FILE *fp;
    fp = fopen("velocity_100.txt", "w");
    char* message = "속력 100으로 주었을 때 test";
    printf("%s\n", message);
    fprintf(fp, "%s\n", message);
    while(1){
        speed_check = DesireSpeed_Read();
        printf("%d, %d\n", count, speed_check);
        fprintf(fp, "%d, %d/n", count, speed_check);
        count ++;
        if (count == 10000) break;
    }
#endif
}
