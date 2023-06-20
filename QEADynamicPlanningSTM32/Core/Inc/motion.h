#ifndef MOTION_H
#define MOTION_H

#include "pid.h"
#include "main.h"
#include "wheel_pwm.h"
#include "imu.h"
#include "math.h"


volatile struct RobotMotion
{
    double VL;
    double VR;
};

extern volatile struct RobotMotion robotMotion;

void ClearSpeed();
void CommitSpeed();
void MoveForward(double speed);
void MoveForwardWithDis(double speed, double dis);
void MoveBackward(double speed);
void MoveBackwardWithDis(double speed, double dis);
void SpinLeft(double speed);
void SpinRight(double speed);
void SpinTo(double radian);




#endif