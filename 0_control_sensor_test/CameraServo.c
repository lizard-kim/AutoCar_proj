#include <stdio.h>
#include "car_lib.h"

void main(void)
{

    CarControlInit();
    printf("FUCKYOU");
    CameraYServoControl_Write(1000);
}