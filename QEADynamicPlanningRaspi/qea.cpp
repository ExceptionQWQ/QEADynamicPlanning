#include "qea.h"
#include "robot.h"
#include "lidar.h"
#include "view.h"

int main(int argc, char** argv)
{
	Robot_Init(); //机器人初始化
	Robot_Start(); //启动机器人线程
	Lidar_Init();
	Lidar_Start();


	sleep(1);

	while (true) {
		// sleep(1);
		auto points = GetPointData();
		cv::Mat view = GetCloudView(points, cv::Size(512, 512), 0.3);
		cv::imshow("view", view);
		cv::waitKey(100);
	}
	
	sleep(99999);
	return 0;
}