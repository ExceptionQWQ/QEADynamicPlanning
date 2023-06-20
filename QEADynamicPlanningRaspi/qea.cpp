#include "qea.h"
#include "robot.h"


int main(int argc, char** argv)
{
	Robot_Init(); //机器人初始化
	Robot_Start(); //启动机器人线程

	sleep(1);
	Robot_MoveForward(30, 30000);
	Robot_MoveBackward(30, 30000);
	Robot_SpinTo(60, robotInfo.heading + 0.5);
	sleep(99999);
	return 0;
}