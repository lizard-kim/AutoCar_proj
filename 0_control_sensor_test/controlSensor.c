#include <stdio.h>
#include "car_lib.h"
#include <math.h>

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
char sensor;
int i, j;
int tol;
char byte = 0x80;
double I_Dist;
double O_Dist;
double doubledata;
int I_data_1, I_data_2, I_data_3, I_data_4, I_data_5, I_data_6;
int O_data_1, O_data_2, O_data_3, O_data_4, O_data_5, O_data_6;
///////////////////////////////////////////////////////////////////////////////////////


/*********************************Functions*******************************************/
double DistFunc(double data);                                                        //
void tunnel();                                                                       //
void parking();                                                                      //
void tunnel_adv();                   
void tunnel_real();     
void parparking();                                           //
///////////////////////////////////////////////////////////////////////////////////////


void main(void)
{
    double data;
    CarControlInit();
    PositionControlOnOff_Write(UNCONTROL);
    // 5. distance sensor --------------------------------------------------------
    printf("INPUT Number\n");
    scanf("%d", &channel);
    while(1)
    {
        //DesireSpeed_Write(300);
        data = DistanceSensor(channel);
        printf("channel = %d, distance = %.0lf\n", channel, data);
        //DistFunc(data);
        //tunnel(); // Mission trigger needed. basic tunnel code without front sensor
        //tunnel_adv(); // Mission trigger needed. advanced tunnel algorithm with front sensor
        //tunnel_real();
        //CarLight_Write(ALL_OFF);
        //parking();
        parparking();
        usleep(100000);
    }
}



double DistFunc(double data)
{
    I_Dist = data;
    O_Dist = pow(I_Dist/22691.0, -1/1.0706);
    printf("%.0lf\n", O_Dist);
    return O_Dist;
}


void tunnel()
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


void tunnel_adv()
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
            printf("FUCK");
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
            printf("FUCK2222");
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
            printf("FUCK");
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



void tunnel_real()
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
            printf("FUCK");
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
            printf("FUCK2222");
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
void parking()
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
                /* 
                if (O_data_2 < 10)
                {
                    angle = -30*O_data_2 + 1800;
                    SteeringServoControl_Write(angle);
                }
                if(O_data_2 >= 10)
                {
                    angle = -27*O_data_2 + 1800;
                    SteeringServoControl_Write(angle);
                }*/
            }
            /* 
            if(EncoderCounter_Read() < -8000) // or EncoderCounter_Read == -8000; it means car moved around 37cm total.
            {
                DesireSpeed_Write(0);
                ParkingSignal_2 = 1;
                printf("Reverse Finished\n");
                printf("ParkingSignal_2 : %d\n", ParkingSignal_2);
                break;
            }*/
        }
        printf("FUCK0");
        SteeringServoControl_Write(1500);
        usleep(1000000);
        EncoderCounter_Write(0);
        
        
        printf("FUCK0_1");
        SteeringServoControl_Write(1050);
        DesireSpeed_Write(100);
        usleep(4000000);
        SteeringServoControl_Write(1500);
        /* 
        if(EncoderCounter_Read() == 18000)
        {
            printf("FUCK2");
            SteeringServoControl_Write(1500);
        }*/

        printf("Basic Mode is ready...");
        DesireSpeed_Write(0); //E-Stop;
        ParkingSignal_2 = 1;
        CarLight_Write(ALL_OFF);
        return 0;

        /* 
        if(EncoderCounter_Read() == 600) //edit afterward
        {
            DesireSpeed_Write(0);
            EncoderCounter_Write(0);
            printf("steering...");
            SteeringServoControl_Write(1050);
            DesireEncoderCount_Write(600); //31.415 cm after steering 0
            if(DesireEncoderCount_Read() >= 600)
            {
                SteeringServoControl_Write(0);
                printf("Steering completed");
                int I_data_1 = DistanceSensor(1);
                int O_data_1 = DistFunc(I_data_1);
                if(O_data_1 < 10) // distance of front is under 10cm
                {
                    DesireSpeed_Write(0);
                    printf("parking finished...!");
                }
                else if(O_data_2 < 10)
                {
                    angle = -30*O_data_2 + 1800;
                    SteeringServoControl_Write(angle);
                }
                else if(O_data_2 >= 10)
                {
                    angle = -27*O_data_2 + 1770;
                    if(angle < 1200)
                        angle = 1200;
                    SteeringServoControl_Write(angle);    
                }

            }        
        }*/

    }
    

}


int parParkingSignal_1 = 0;
int parParkingSignal_2 = 0;
void parparking()
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
        parParkingSignal_1 = 1;
        printf("Parking Area nearby\n");
    }
    if(O_data_2 > 10 && O_data_3 > 10)
    {
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
        SteeringServoControl_Write(1050);
        //printf("Steering..\n.");
        EncoderCounter_Write(0);
        DesireSpeed_Write(-200);
        int posRead = EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        while(1)
        {
            if(EncoderCounter_Read() < -600) // or EncoderCounter_Read == -6000; it means car moved 30cm.
            {
                printf("-4000 reached\n");
                SteeringServoControl_Write(1500);
                if(EncoderCounter_Read() < -800)
                {
                    SteeringServoControl_Write(1950);
                    if(EncoderCounter_Read() < -1000)
                    {
                        DesireSpeed_Write(0);
                        SteeringServoControl_Write(1500);
                    }
                }
                DesireSpeed_Write(0);
                //SteeringServoControl_Write(1500);
                usleep(1000000);
                printf("breakthrough while");
                break;
            }
        }
        printf("Start exit");
        DesireSpeed_Write(-100);
        usleep(30000);

        SteeringServoControl_Write(1950);
        DesireSpeed_Write(100);
        usleep(2000000);
        printf("step 1...\n")

        SteeringServoControl_Write(1050);
        usleep(1000000);
        printf("step 2...\n");

        SteeringServoControl_Write(1500);
 
        printf("Basic Mode is ready...Parking finished..!!!\n");
        DesireSpeed_Write(0); //E-Stop;
        parParkingSignal_2 = 1;
        CarLight_Write(ALL_OFF);
        

        return 0;
    }
}


