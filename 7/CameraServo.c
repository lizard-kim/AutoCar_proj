#include <stdio.h>
#include "car_lib.h"

void main(void)
{

    CarControlInit();
    printf("FUCKYOU\n");
    CameraYServoControl_Write(1600);
}

