#include <stdio.h>
#include "car_lib.h"
#include <string.h>
#include <stdlib.h>

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
    sleep(3);

    signed short speed_input;
    scanf("%d", &speed_input);
    //speed set
    printf("speed를 %d으로 주겠습니다!", speed_input);
    //speed = 1000;
    DesireSpeed_Write(speed_input);
    signed short speed_check;
    int count = 0;
    char* speed_string;
    char* txt_name = malloc(sizeof(char)* 20);
    FILE *fp;

    sprintf(speed_string,"%d",speed_input);
    strcat(txt_name, "velocity_");
    strcat(txt_name, speed_string);
    strcat(txt_name, ".txt");

    fp = fopen(txt_name, "w");

    char* message = "속력을 주었을 때 test";


    printf("%s\n", message);
    fprintf(fp, "%s\n", message);
    while(1){
        speed_check = DesireSpeed_Read();
        printf("%d, %d\n", count, speed_check);
        fprintf(fp, "%d, %d/n", count, speed_check);
        count ++;
        if (count == 30000) break;
    }
    printf("다시 speed를 0으로 주겠습니다!");
    speed = 0;
    DesireSpeed_Write(speed);
#endif
}
