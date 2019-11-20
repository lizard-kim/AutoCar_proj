/** #include <stdio.h> */
/** #include "car_lib.h" */
/** #include <math.h> */
/**  */
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
/** unsigned char status; */
/** short speed; */
/** unsigned char gain; */
/** int position, posInit, posDes, posRead; */
/** short angle; */
/** int channel; */
/** char sensor; */
/** int i, j; */
/** int tol; */
/** char byte = 0x80; */
/** double I_Dist; */
/** double O_Dist; */
/** double doubledata; */
/** int I_data_1, I_data_2, I_data_3, I_data_4, I_data_5, I_data_6; */
/** int O_data_1, O_data_2, O_data_3, O_data_4, O_data_5, O_data_6; */
/** /////////////////////////////////////////////////////////////////////////////////////// */
/**  */
/**  */
/** [>*******************************Functions******************************************<] */
/** double DistFunc(double data);                                                        // */
/** void parking();                                                                      // */
/** void tunnel_adv();                    */
/** void parparking();                                           // */
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
        if(channel == 1)
        {
            /** tunnel(); */
           tunnel_adv();
        }
        else if(channel == 2)
        {
            /** tunnel_real(); */
        }
        else if(channel == 3)
        {
            /** tunnel(); */
        }
        else if(channel==4)
        {
            parking();
        }
        else if(channel == 5)
        {
            parparking();
        }
        else if(channel == 6)
        {
            I_data_1 = DistanceSensor(3);
            O_data_1 = DistFunc(I_data_1);
            usleep(50000);
        }
        //printf("channel = %d, distance = %.0lf\n", channel, data);
        //DistFunc(data);
        //tunnel(); // Mission trigger needed. basic tunnel code without front sensor
        //tunnel_adv(); // Mission trigger needed. advanced tunnel algorithm with front sensor
        //tunnel_real();
        //CarLight_Write(ALL_OFF);
        //parking();
        //parparking();
        //usleep(100000);
    }
}



double DistFunc(double data)
{
    I_Dist = data;
    O_Dist = pow(I_Dist/22691.0, -1/1.0706);
    printf("%.0lf\n", O_Dist);
    return O_Dist;
}

void DistanceTest()
{
    CarLight_Write(FRONT_ON);
    I_data_5 = DistanceSensor(5);
    O_data_5 = DistFunc(I_data_5);
    I_data_6 = DistanceSensor(6);
    O_data_6 = DistFunc(I_data_6);
    //Distance between car and wall is each 10cm.
    // 10 --> 1500 , 20 --> 1230            y = -27x + 1770

    printf("O_data_5 : %d\n", O_data_5 );
    printf("O_data_6 : %d\n", O_data_6 );
    usleep(50000);
}


void tunnel_adv()
{
    DesireSpeed_Write(400);
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
    // 10 --> 1500 , 20 --> 1000            y = -50x + 2000
    if(O_data_2 >= 10)
    {
        angle = -50*O_data_2 + 2000;
        if(angle < 1200)
        {
            angle = 1200;
        }
        if(O_data_1 < 40)
        {
            printf("FUCK");
            DesireSpeed_Write(100);
            //angle = -16*O_data_1 + 2140;
            angle = 1990;
        }
        SteeringServoControl_Write(angle);
        if(O_data_1 < 40)
        {
            usleep(500000);
        }
        DesireSpeed_Write(400);
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
            DesireSpeed_Write(100);
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

//1. if sensor 4 distance & sensor 5 distance is the same. (left == right)
//2. if sensor 2 and sensor 3 is the same (two right same)
//3. time control
//4. Position(Encoder) control


int ParkingSignal_1 = 0;
int ParkingSignal_2 = 0;
void parking()
{//수직
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
    
    if(ParkingSignal_1 == 0 && O_data_2 < 30 && O_data_3 > 30)
    {
        ParkingSignal_1 = 1;
        printf("Parking Point Detected...\n");
       // continue;
    }
    if(ParkingSignal_1 == 1 && O_data_2 > 30 && O_data_3 < 30)
    {
        ParkingSignal_1 = 2;
        printf("Parking Area here\n");
        //continue;
    }
    if(ParkingSignal_1 == 2 && O_data_2 > 30 && O_data_3 > 30)
    {
        ParkingSignal_1 = 3;
        printf("Parking fucking\n");
        //continue;
    }
    if(ParkingSignal_1 == 3 && O_data_3 < 30)
    {
        printf("fuck succeess\n");
        int meandist = DistFunc(DistanceSensor(3));
        printf("ParkingSignal = %d\n", ParkingSignal_1);
        printf("O_data_2 : %d\n", O_data_2);
        printf("O_data_3 : %d\n", O_data_3);
        DesireSpeed_Write(100);
        usleep(600000+(meandist*3000));
        printf("MISSION ONE\n");
        DesireSpeed_Write(0);
        //printf("Parking Start!!\n");
        usleep(500000);
        printf("MISSION TWO\n");
        SteeringServoControl_Write(1050);
        //printf("Steering..\n.");
        EncoderCounter_Write(0);
        DesireSpeed_Write(-100);
        printf("MISSION THREE\n");
        int posRead = EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
        while(1)
        {
            if(EncoderCounter_Read() < -1300 || DistFunc(DistanceSensor(4)) < 7) // or EncoderCounter_Read == -6000; it means car moved 30cm.
            {
                printf("-4000 reached\n");
                DesireSpeed_Write(0);
                SteeringServoControl_Write(1500);
                //SteeringServoControl_Write(1500);
                
                break;
            }
        }
        usleep(200000);
        SteeringServoControl_Write(1500);
        while(1)
        {
            if(DistFunc(DistanceSensor(4)) > 8)
            {
                DesireSpeed_Write(-100);
            }
            else
            {
                break;
            }
        }
    
        DesireSpeed_Write(0);
        usleep(1000000);
        

        printf("FUCK0");
        SteeringServoControl_Write(1500);
        DesireSpeed_Write(100);
        usleep(620000);
        EncoderCounter_Write(0);
        
        printf("FUCK0_1");
        SteeringServoControl_Write(1001);
        DesireSpeed_Write(100);
        usleep(3300000);
        SteeringServoControl_Write(1500);

        printf("Basic Mode is ready...");
        DesireSpeed_Write(0); //E-Stop;
        ParkingSignal_2 = 1;
        CarLight_Write(ALL_OFF);
        return 0;
    }
}




int parParkingSignal_1 = 0;
int parParkingSignal_2 = 0;
void parparking()
{//수평
    DesireSpeed_Write(100);
    /*  After parking(), trgging parparking()...
    if(parParkingSignal_2 != 1)
    {
        return;
    }*/

    I_data_2 = DistanceSensor(2);
    I_data_3 = DistanceSensor(3);
    int I_data_5 = DistanceSensor(5);
    int O_data_5 = DistFunc(I_data_5);
    O_data_2 = DistFunc(I_data_2);
    O_data_3 = DistFunc(I_data_3);

    CarLight_Write(REAR_ON);
    if(parParkingSignal_2 == 1)
    {
        return;
    }
    if(parParkingSignal_1 == 0 && O_data_2 < 30 && O_data_3 > 30)
    {
        printf("Parking Area nearby\n");
        parParkingSignal_1 = 1;
    }
    if(parParkingSignal_1 == 1 && O_data_2 > 30 && O_data_3 < 30)
    {
        parParkingSignal_1 = 2;
        printf("Parking Area here\n");
        printf("parParkingSignal_1 : %d\n", parParkingSignal_1);
        //continue;
    }
    if(parParkingSignal_1 == 2 && O_data_2 > 30 && O_data_3 > 30)
    {
        parParkingSignal_1 = 3;
        printf("Parking Point Detected...\n");
       // continue;
    }
    if(parParkingSignal_1 == 3 && O_data_3 < 30)
    {
        printf("Fucking PARARREL\n");
        //printf("ParkingSignal = %d\n", ParkingSignal_1);
        printf("O_data_2 : %d\n", O_data_2);
        printf("O_data_3 : %d\n", O_data_3);
        usleep(170000);
        DesireSpeed_Write(0);
        //printf("Parking Start!!\n");
        usleep(500000);

        SteeringServoControl_Write(1000);
        //printf("Steering..\n.");
        EncoderCounter_Write(0);
        DesireSpeed_Write(-100);
        usleep(1300000);
        /* 
        while(1)
        {
            int posRead_1 = EncoderCounter_Read();
            printf("EncoderCounter_Read() = %d\n", posRead_1);

            printf("-600 reached\n");
            SteeringServoControl_Write(1500);
            usleep(700000);

            printf("-800 reached\n");
            SteeringServoControl_Write(2000);
            usleep(1500000);

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
        */
        int posRead_1 = EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead_1);

        printf("-600 reached\n");
        SteeringServoControl_Write(1500);
        usleep(500000);

        printf("-800 reached\n");
        SteeringServoControl_Write(2000);

        while(1)
        {
            if(DistFunc(DistanceSensor(4)) > 8)
            {
                DesireSpeed_Write(-100);
            }
            else
            {
                DesireSpeed_Write(0);
                break;
            }
        }

        SteeringServoControl_Write(1500);
        while(1)
        {
            int posRead_1 = DistFunc(DistanceSensor(4));
            printf("DIST4 SENSOR = %d\n", posRead_1);
            
            if(DistFunc(DistanceSensor(4)) <= 5 && DistFunc(DistanceSensor(4)) >= 4 )
            {
                DesireSpeed_Write(-50);
            }
            else
            {
                DesireSpeed_Write(0);
                break;
            }
        }



        /* 
        printf("Start exit\n");
        DesireSpeed_Write(-100);
        usleep(300000);
        */

        SteeringServoControl_Write(1999);
        DesireSpeed_Write(100);
        usleep(1500000);
        printf("step 1...\n");

        SteeringServoControl_Write(1500);
        usleep(700000);

        SteeringServoControl_Write(1000);
        usleep(1900000);
        printf("step 2...\n");

        SteeringServoControl_Write(1500);
 
        printf("Basic Mode is ready...Parking finished..!!!\n");
        DesireSpeed_Write(0); //E-Stop;
        parParkingSignal_2 = 1;
        CarLight_Write(ALL_OFF);


        return 0;
    }
}
