#include "DynamicObs.h"

void dynamic_obs_ver2(double angle, signed short speed, signed short speedratio) {
    DesireSpeed_Write(0);
    SteeringServoControl_Write(1500);

    while (1) {
            double front_dist = DistFunc(DistanceSensor(1));
            if (front_dist < DYNAMIC_OBS_START) break;
    }
    /// go straight
    DesireSpeed_Write(150);
    usleep(2*1000*1000);

    while (DistFunc(DistanceSensor(4)) > DYNAMIC_OBS_END) {} /// 뒤에서 차가 따라옴. Dynamic obs end 수치 이하일때

    DesireSpeed_Write(speed*speedratio);
    SteeringServoControl_Write(angle);

    usleep(2*1000*1000);

    // lane tracing
    // lane tracing
}
