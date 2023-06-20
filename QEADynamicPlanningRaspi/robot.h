#ifndef ROBOT_H
#define ROBOT_H

#include "qea.h"

#define ROBOT_DEV_PATH "/dev/ttyACM0"
#define ROBOT_BAUD 115200

struct RobotInfo
{
    double xPos;
    double yPos;
    double roll;
    double pitch;
    double heading;
};

extern struct RobotInfo robotInfo;

extern int robotSerial;
extern pthread_t robotThreadHandle;
extern char robotRecvBuff[1024];
extern int robotRecvOffset;

void Robot_Init();
void Robot_Start();

void Robot_MoveForward(double speed, double dis);
void Robot_MoveBackward(double speed, double dis);
void Robot_SpinTo(double speed, double radian);



#endif