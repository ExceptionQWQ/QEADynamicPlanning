#include "motion.h"


volatile struct RobotMotion robotMotion;

void ClearSpeed()
{
    robotMotion.VL = 0;
    robotMotion.VR = 0;
}

void CommitSpeed()
{
    leftPWM.pidHanldeDef.target = robotMotion.VL;
    rightPWM.pidHanldeDef.target = -robotMotion.VR;
}

void MoveForward(double speed)
{
    robotMotion.VL += speed;
    robotMotion.VR += speed;
}

void MoveBackward(double speed)
{
    robotMotion.VL -= speed;
    robotMotion.VR -= speed;
}

void SpinLeft(double speed)
{
    robotMotion.VL -= speed;
    robotMotion.VR += speed;
}

void SpinRight(double speed)
{
    robotMotion.VL += speed;
    robotMotion.VR -= speed;
}

void MoveForwardWithDis(double speed, double dis)
{

}

void MoveBackwardWithDis(double speed, double dis)
{

}

void SpinTo(double radian)
{
    while (1) {
        double dr = fabs(radian - robotIMU.heading);
        if (dr < 0.01) break;
        double maxSpeed = 60;
        double speed = dr * 140;
        if (speed > maxSpeed) speed = maxSpeed;
        if (radian - robotIMU.heading > 0) {
            if (radian - robotIMU.heading > PI) {
                ClearSpeed();
                SpinRight(speed);
                CommitSpeed();
            } else {
                ClearSpeed();
                SpinLeft(speed);
                CommitSpeed();
            }
        } else {
            if (radian - robotIMU.heading > -PI) {
                ClearSpeed();
                SpinRight(speed);
                CommitSpeed();
            } else {
                ClearSpeed();
                SpinLeft(speed);
                CommitSpeed();
            }
        }
    }
    ClearSpeed();
    CommitSpeed();
}