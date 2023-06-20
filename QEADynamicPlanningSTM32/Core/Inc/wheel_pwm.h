#ifndef WHEEL_PWM_H
#define WHEEL_PWM_H

#include "main.h"
#include "tim.h"
#include "pid.h"

volatile struct WheelPWM
{
    volatile struct PIDHanldeDef pidHanldeDef; //pid算法句柄
    double speed; //电机速度
    double dis; //路程
    double pwm; //最终输出pwm
};


extern volatile struct WheelPWM leftPWM;
extern volatile struct WheelPWM rightPWM;


void Wheel_PID_Init();
void Wheel_PID_Tick();




#endif