#include "qea.h"
#include "robot.h"
#include "lidar.h"
#include "view.h"
#include "mapping.h"
#include "planning.h"

int main(int argc, char** argv)
{
	Robot_Init(); //机器人初始化
	Robot_Start(); //启动机器人线程
	Lidar_Init();
	Lidar_Start();

	sleep(1);

	Planning_Init();
	Planning_Start();


	Mapping mapping(cv::Size(800, 800), {700, 700}, 0.4);
	mapping.SetColor({0, 255, 0});

	while (true) {
		char ch;
		std::cin >> ch;
		double radian = robotInfo.heading;
		switch (ch) {
			case 'w':
				if (Robot_MoveForward(30, 50)) {
					robotInfo.xPos += 50 * std::cos(robotInfo.heading);
					robotInfo.yPos += 50 * std::sin(robotInfo.heading);
				}
			break;
			case 's':
				if (Robot_MoveBackward(30, 50)) {
					robotInfo.xPos += -50 * std::cos(robotInfo.heading);
					robotInfo.yPos += -50 * std::sin(robotInfo.heading);
				}
			break;
			case 'a':
				radian += 0.2;
				radian = std::fmod(radian + 2 * PI, 2 * PI);
				Robot_SpinTo(60, radian);
			break;
			case 'd':
				radian -= 0.2;
				radian = std::fmod(radian + 2 * PI, 2 * PI);
				Robot_SpinTo(60, radian);
			break;
			case 'm':
				mapping.SetRobotPos(robotInfo.xPos, robotInfo.yPos);
				mapping.SetRobotHeading(robotInfo.heading);
				auto points = GetPointData();
				mapping.UpdateLidarPoints(points);
				
				cv::Mat mappingView = mapping.GetMapView();
				cv::imshow("mappingView", mappingView);
				cv::waitKey(500);
			break;
		}
	}
	return 0;
}