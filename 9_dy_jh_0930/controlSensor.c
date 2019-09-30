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

/*******************************************************************************
 *  Defines
 * 1 = front
 * 2 = right_front
 * 3 = right_back
 * 4 = back
 * 5 = left_back
 * 6 = left_front
 *******************************************************************************
 */

/********************************HEADERS*********************************************/
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
double I_Dist;
double O_Dist;
double doubledata;
int I_data_1, I_data_2, I_data_3, I_data_4, I_data_5, I_data_6;
int O_data_1, O_data_2, O_data_3, O_data_4, O_data_5, O_data_6;



/*********************************Functions*******************************************/
double DistFunc(double data);                                                        //
void tunnel();                                                                       //
void parking();                                                                      //
void tunnel_adv();                   
void tunnel_real();     
void parparking();                                           //
///////////////////////////////////////////////////////////////////////////////////////


double DistFunc(double data)
{
    return pow(data/22691.0, -1/1.0706);
}


void dynamic_obs(void) { /// this function is for executing moving obstacle in rotary

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

void tunnel() /// this function is for executing tunnel mission by using psd
{
    CarLight_Write(FRONT_ON);
    I_data_2 = DistanceSensor(2);
    O_data_2 = DistFunc(I_data_2);
    //Distance between car and wall is each 10cm.
    // 10 --> 1500 , 20 --> 1230            y = -27x + 1770
    if(O_data_2 >= 10)
    {
        angle = -27*O_data_2 + 1770;
        if(angle < 1200)
        {
            angle = 1200;
        }
        SteeringServoControl_Write(angle);
        //printf("tunnel, O>10 = %d", angle);
    }
    else if(O_data_2 < 10) // 10--> 1500 0 --> 1800            y = -30x + 1800
    {
        angle = -30*O_data_2 + 1800;
        //printf("tunnel, O<10 = %d", angle);
        SteeringServoControl_Write(angle);
    }
}


void tunnel_adv() /// tunnel advanced function: it check front distance
{
    
    I_data_1 = DistanceSensor(1);
    O_data_1 = DistFunc(I_data_1);
    printf("O_data_1 : %d", O_data_1);

    CarLight_Write(FRONT_ON);
    I_data_2 = DistanceSensor(2);
    O_data_2 = DistFunc(I_data_2);

    int I_data_5, O_data_5;
    I_data_5 = DistanceSensor(5);
    O_data_5 = DistFunc(I_data_5);
    printf("O_data_5 : %d", O_data_5);
    //Distance between car and wall is each 10cm.
    // 10 --> 1500 , 20 --> 1230            y = -27x + 1770
    if(O_data_2 >= 10)
    {
        angle = -27*O_data_2 + 1770;
        if(angle < 1200)
        {
            angle = 1200;
        }
        if(O_data_1 < 40)
        {
            //angle = -16*O_data_1 + 2140;
            angle = 1990;
        }
        SteeringServoControl_Write(angle);
        if(O_data_1 < 40)
        {
            usleep(500000);
        }
        printf("tunnel, O>10 = %d", angle);
    }
    else if(O_data_2 < 10) // 10--> 1500 0 --> 1800            y = -30x + 1800
    {
        angle = -30*O_data_2 + 1800;
        if(O_data_1 < 40)
        {
            //angle = -16*O_data_1 + 2140;
            angle = 1990;
        }
        printf("tunnel, O<10 = %d", angle);
        SteeringServoControl_Write(angle);
        if(O_data_1 < 40)
        {
            usleep(500000);
        }
    }
    else if(O_data_1 < 40) // must be edited afterward;;;;;;;;;;
    {
        printf("tunnel, detected front sensor");
        if(O_data_2 <  10) // right curve , looking front
        {
            angle = 1800;
            printf("tunnel, right curve = %d", angle);
            SteeringServoControl_Write(angle);
        }
        else if(O_data_2  - O_data_5 <= 3) // left curve, looking front
        {
            angle = 1200;
            printf("tunnel, left curve = %d", angle);
            SteeringServoControl_Write(angle);
        }
    }
}



void tunnel_real() /// it's for tunnel mission: it use all of psd sensors
{
    
    I_data_1 = DistanceSensor(1);
    O_data_1 = DistFunc(I_data_1);
    printf("O_data_1 : %d", O_data_1);

    CarLight_Write(FRONT_ON);
    I_data_2 = DistanceSensor(2);
    O_data_2 = DistFunc(I_data_2);

    int I_data_5, O_data_5;
    I_data_5 = DistanceSensor(5);
    O_data_5 = DistFunc(I_data_5);
    printf("O_data_5 : %d", O_data_5);
    //Distance between car and wall is each 10cm.
    // 10 --> 1500 , 20 --> 1230            y = -27x + 1770
    if(O_data_2 >= 10)
    {
        angle = -27*O_data_2 + 1770;
        if(angle < 1200)
        {
            angle = 1200;
        }
        if(O_data_1 < 20 && O_data_2 - O_data_6 <= 3) // if right curve
        {
            angle = 16*O_data_1 + 860;
        }
        if(O_data_1 < 2 && O_data_6 - O_data_2 <= 3) // if left curve
        {
            angle = -16*O_data_1 + 2140;
        }
        SteeringServoControl_Write(angle);
        printf("tunnel, O>10 = %d", angle);
    }
    else if(O_data_2 < 10) // 10--> 1500 0 --> 1800            y = -30x + 1800
    {
        angle = -30*O_data_2 + 1800;
        if(O_data_1 < 20 && O_data_2 - O_data_6 <= 3) // if right curve
        {
            angle = 16*O_data_1 + 860;
        }
        if(O_data_1 < 20 && O_data_6 - O_data_2 <= 3)
        {
            angle = -16* O_data_1 + 2140;
        }
        printf("tunnel, O<10 = %d", angle);
        SteeringServoControl_Write(angle);
    }
    else if(O_data_2 > 30 && O_data_5 > 30)
    {
        return 0;
    }
    
}

//1. if sensor 4 distance & sensor 5 distance is the same. (left == right)
//2. if sensor 2 and sensor 3 is the same (two right same)
//3. time control
//4. Position(Encoder) control


int ParkingSignal_1 = 0;
int ParkingSignal_2 = 0;
void parking() /// it's for T parking
{
    DesireSpeed_Write(100);
    if(ParkingSignal_2 == 1)
    {
        return;
    }
    I_data_2 = DistanceSensor(2);
    I_data_3 = DistanceSensor(3);
    int I_data_5 = DistanceSensor(5);
    int O_data_5 = DistFunc(I_data_5);
    O_data_2 = DistFunc(I_data_2);
    O_data_3 = DistFunc(I_data_3);

    CarLight_Write(REAR_ON);
    
    if(O_data_2 > 10 && O_data_3 < 10)
    {
        ParkingSignal_1 = 1;
        printf("Parking Area nearby\n");
        //continue;
        /* 
        int PosInit = 0;
        EncoderCounter_Write(PosInit);
        DesireSpeed_Write(50);
        */
    }
    if(O_data_2 > 10 && O_data_3 > 10)
    {
        printf("Parking Area here\n");
        //continue;
    }
    if(O_data_2 < 10 && O_data_3 > 10)
    {
        printf("Parking Point Detected...\n");
       // continue;
    }
    if(ParkingSignal_1 == 1 && O_data_3 < 10 && O_data_2 < 10)
    {
        printf("ParkingSignal = %d\n", ParkingSignal_1);
        printf("O_data_2 : %d\n", O_data_2);
        printf("O_data_3 : %d\n", O_data_3);
        DesireSpeed_Write(0);
        //printf("Parking Start!!\n");
        usleep(500000);
        SteeringServoControl_Write(1050);
        //printf("Steering..\n.");
        EncoderCounter_Write(0);
        DesireSpeed_Write(-200);
        int posRead = EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        while(1)
        {
            if(EncoderCounter_Read() < -1200) // or EncoderCounter_Read == -6000; it means car moved 30cm.
            {
                printf("-4000 reached\n");
                DesireSpeed_Write(0);
                //SteeringServoControl_Write(1500);
                usleep(1000000);
                break;
        }
        SteeringServoControl_Write(1500);
        DesireSpeed_Write(-100);
        usleep(300000);

        SteeringServoControl_Write(1500);
        DesireSpeed_Write(100);
        usleep(300000);
        EncoderCounter_Write(0);
        

        SteeringServoControl_Write(1050);
        DesireSpeed_Write(200);
        usleep(1230000);
        SteeringServoControl_Write(1500);
        /* 
        if(EncoderCounter_Read() == 18000)
        {
            SteeringServoControl_Write(1500);
        }*/

        printf("Basic Mode is ready...");
        DesireSpeed_Write(0); //E-Stop;
        ParkingSignal_2 = 1;
        CarLight_Write(ALL_OFF);
        return 0;
    }
    

}
}


int parParkingSignal_1 = 0;
int parParkingSignal_2 = 0;

void parparking(void) /// parallel parking
{
    DesireSpeed_Write(100);
    if(parParkingSignal_2 == 1)
    {
        return;
    }

    I_data_2 = DistanceSensor(2);
    I_data_3 = DistanceSensor(3);
    int I_data_5 = DistanceSensor(5);
    int O_data_5 = DistFunc(I_data_5);
    O_data_2 = DistFunc(I_data_2);
    O_data_3 = DistFunc(I_data_3);

    CarLight_Write(REAR_ON);
    
    if(O_data_2 > 10 && O_data_3 < 10)
    {
        printf("Parking Area nearby\n");
    }
    if(O_data_2 > 10 && O_data_3 > 10)
    {
        parParkingSignal_1 = 1;
        printf("Parking Area here\n");
        printf("parParkingSignal_1 : %d\n", parParkingSignal_1);
        //continue;
    }
    if(O_data_2 < 10 && O_data_3 > 10)
    {
        printf("Parking Point Detected...\n");
       // continue;
    }
    if(parParkingSignal_1 == 1 && O_data_3 < 10 && O_data_2 < 10)
    {
        printf("ParkingSignal = %d\n", ParkingSignal_1);
        printf("O_data_2 : %d\n", O_data_2);
        printf("O_data_3 : %d\n", O_data_3);
        DesireSpeed_Write(0);
        //printf("Parking Start!!\n");
        usleep(500000);
        DesireSpeed_Write(100);
        usleep(200000);
        DesireSpeed_Write(0);
        SteeringServoControl_Write(1000);
        //printf("Steering..\n.");
        EncoderCounter_Write(0);
        DesireSpeed_Write(-100);
        usleep(1400000);
        while(1)
        {
            int posRead_1 = EncoderCounter_Read();
            printf("EncoderCounter_Read() = %d\n", posRead_1);

            printf("-600 reached\n");
            SteeringServoControl_Write(1500);
            usleep(800000);

            printf("-800 reached\n");
            SteeringServoControl_Write(2000);
            usleep(1400000);

            int posRead_2 = EncoderCounter_Read();
            printf("EncoderCounter_Read() = %d\n", posRead_2);
            printf("-1000 reached\n");
            DesireSpeed_Write(0);
            SteeringServoControl_Write(1500);

            printf("breakthrough while\n");
            DesireSpeed_Write(0);
            usleep(1000000);
            
            break;

        }
        printf("Start exit\n");
        DesireSpeed_Write(-100);
        usleep(300000);

        SteeringServoControl_Write(2000);
        DesireSpeed_Write(100);
        usleep(1700000);
        printf("step 1...\n");

        SteeringServoControl_Write(1500);
        usleep(600000);

        SteeringServoControl_Write(1000);
        usleep(1700000);
        printf("step 2...\n");

        SteeringServoControl_Write(1500);
 
        printf("Basic Mode is ready...Parking finished..!!!\n");
        DesireSpeed_Write(0); //E-Stop;
        parParkingSignal_2 = 1;
        CarLight_Write(ALL_OFF);


        return 0;
    }
    
}
