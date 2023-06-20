#ifndef ROBOT_H
#define ROBOT_H

#include "qea.h"

#define ROBOT_DEV_PATH "/dev/ttyACM0"
#define ROBOT_BAUD 115200
#define ROBOT_MM_TO_PULSE 438 //1毫米对应的脉冲数

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

bool Robot_MoveForward(double speed, double dis);
bool Robot_MoveBackward(double speed, double dis);
bool Robot_SpinTo(double speed, double radian);



#endif