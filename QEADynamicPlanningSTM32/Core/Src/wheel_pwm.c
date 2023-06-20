#include "wheel_pwm.h"

volatile struct WheelPWM leftPWM;
volatile struct WheelPWM rightPWM;


void PID_Init()
{
    leftPWM.kp = 960;
    leftPWM.ki = 240;
    leftPWM.kd = 0;
    leftPWM.target = 0;
    leftPWM.minPWM = -1200;
    leftPWM.maxPWM = 1200;

    rightPWM.kp = 960;
    rightPWM.ki = 240;
    rightPWM.kd = 0;
    rightPWM.target = 0;
    rightPWM.minPWM = -1200;
    rightPWM.maxPWM = 1200;
}

void Tick(volatile struct WheelPWM* wheelPwm)
{
    wheelPwm->lastError2 = wheelPwm->lastError;
    wheelPwm->lastError = wheelPwm->error;
    wheelPwm->error = wheelPwm->target - wheelPwm->speed;

    double pwm = wheelPwm->kp * (wheelPwm->error - wheelPwm->lastError) +
                 wheelPwm->ki * wheelPwm->error +
                 wheelPwm->kd * (wheelPwm->error - 2 * wheelPwm->lastError + wheelPwm->lastError2);
    wheelPwm->pwm += pwm;
    if (wheelPwm->pwm > wheelPwm->maxPWM) wheelPwm->pwm = wheelPwm->maxPWM;
    if (wheelPwm->pwm < wheelPwm->minPWM) wheelPwm->pwm = wheelPwm->minPWM;
}

void PID_Tick()
{
    //获取左右轮速度
    leftPWM.speed = (short) __HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    rightPWM.speed = (short) __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    //通过速度积分计算行驶过的路程
    leftPWM.dis += leftPWM.speed;
    rightPWM.dis += rightPWM.speed;

    //计算一次pid
    Tick(&leftPWM);
    Tick(&rightPWM);

    //更新左轮pwm
    if (leftPWM.pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, leftPWM.pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (leftPWM.pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -leftPWM.pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }

    //更新右轮pwm
    if (rightPWM.pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, rightPWM.pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (rightPWM.pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -rightPWM.pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}