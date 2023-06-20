#include "qea.h"
#include "robot.h"
#include "lidar.h"

int main(int argc, char** argv)
{
	Robot_Init(); //机器人初始化
	Robot_Start(); //启动机器人线程
	Lidar_Init();
	Lidar_Start();


	sleep(1);

	while (true) {
		sleep(1);
		auto points = GetPointData();
		for (size_t i = 0; i != points.size(); ++i) {
			std::cout << points[i].angle << " " << (int)points[i].intensity << std::endl;
		}
	}
	
	sleep(99999);
	return 0;
}