#ifndef WHEEL_PWM_H
#define WHEEL_PWM_H

#include "main.h"
#include "tim.h"

volatile struct WheelPWM
{
    double kp;
    double ki;
    double kd;
    double speed;
    double target;
    double error;
    double lastError;
    double lastError2;
    double minPWM;
    double maxPWM;
    double pwm;
    double dis;
};


extern volatile struct WheelPWM leftPWM;
extern volatile struct WheelPWM rightPWM;


void PID_Init();
void PID_Tick();




#endif