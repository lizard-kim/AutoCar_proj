#include <stdio.h>
#include "car_lib.h"
#include <math.h>

unsigned char status;
short speed;
unsigned char gain;
int position, posInit, posDes, posRead;
short angle;
int channel;
char sensor;
int i, j;
int tol;
// char byte = 0x80;
double I_Dist;
double O_Dist;
double doubledata;
int I_data_1, I_data_2, I_data_3, I_data_4, I_data_5, I_data_6;
int O_data_1, O_data_2, O_data_3, O_data_4, O_data_5, O_data_6;
///////////////////////////////////////////////////////////////////////////////////////


/*********************************Functions*******************************************/
double DistFunc(double data);                                                        //
void parking();                                                                      //
void tunnel_adv();                   
void parparking(); 
