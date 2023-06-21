#include "planning.h"

pthread_t planningThreadHandle;


QOBJ startObj;
QOBJ endObj;

void Planning_Init()
{
    startObj.SetQ(200);
    startObj.SetPos(0, 0);

    double endX = 1000 * std::cos(-PI / 180 * 45 + robotInfo.heading);
    double endY = 1000 * std::sin(-PI / 180 * 45 + robotInfo.heading);
    endObj.SetQ(-9000);
    endObj.SetPos(endX, endY);
}

void* PlanningThread(void*)
{
    Mapping mapping(cv::Size(800, 800), {700, 700}, 0.4);
	mapping.SetColor({0, 255, 0});

    while (true) {
        //检测是否达到目标点附近
        double len = std::sqrt(std::pow(robotInfo.xPos - endObj.GetX(), 2) + std::pow(robotInfo.yPos - endObj.GetY(), 2));
        if (len < 100) { //小于10cm
            break;
        }


        std::vector<QOBJ> qobjs;
        qobjs.push_back(startObj);
        qobjs.push_back(endObj);

        auto points = GetPointData();
        for (size_t index = 0; index != points.size(); ++index) {
            QOBJ qobj;
            qobj.SetQ(10);
            qobj.SetPos(robotInfo.xPos + points[index].distance * std::cos(PI / 180 * points[index].angle + robotInfo.heading),
                        robotInfo.yPos + points[index].distance * std::sin(PI / 180 * points[index].angle + robotInfo.heading));
            qobjs.push_back(qobj);
        }

        mapping.SetRobotPos(robotInfo.xPos, robotInfo.yPos);
		mapping.SetRobotHeading(robotInfo.heading);
		mapping.UpdateLidarPoints(points);
		cv::Mat mappingView = mapping.GetMapView();
		cv::imshow("mappingView", mappingView);


        double scale = 2;
        cv::Point offset = {robotInfo.xPos / scale - 400, robotInfo.yPos / scale - 400};
        cv::Mat gradientView = GetGradientViewFromQOBJ(qobjs, cv::Size(800, 800), cv::Scalar{0, 255, 0}, scale, offset);

        //绘制小车朝向
        cv::Point directionEnd;
        directionEnd.x = 400 + 100 * cos(robotInfo.heading);
        directionEnd.y = 400 + 100 * sin(robotInfo.heading);
        cv::line(gradientView, cv::Point(400, 400), directionEnd, cv::Scalar(255, 0, 0), 3);

        cv::Point start(robotInfo.xPos, robotInfo.yPos);
        for (int i = 0; i < 100; ++i) {
            auto grad = CalcQOBJSGradient(qobjs, start.x, start.y);
            double len = std::sqrt(std::pow(grad.first, 2) + std::pow(grad.second, 2));
            double stlScale = 50 / len;
            grad.first *= stlScale; grad.second *= stlScale;
            cv::Point end(start.x + grad.first, start.y + grad.second);
            cv::line(gradientView, cv::Point(start.x / scale - offset.x, start.y / scale - offset.y), cv::Point(end.x / scale - offset.x, end.y / scale - offset.y), cv::Scalar(0, 0, 255), 3);
            start = end;
        }

        cv::flip(gradientView, gradientView, 0);
        cv::imshow("gradientView", gradientView);
        cv::waitKey(1);

        auto [dx, dy] = CalcQOBJSGradient(qobjs, robotInfo.xPos, robotInfo.yPos);
        double radian = std::atan(dy / dx);
        
        if (dx > 0 && dy > 0) {
            radian += 0;
        } else if (dx < 0 && dy > 0) {
            radian += PI;
        } else if (dx < 0 && dy < 0) {
            radian += PI;
        } else if (dx > 0 && dy < 0) {
            radian += 2 * PI;
        }
        radian = std::fmod(radian, 2 * PI);

        if (Robot_SpinTo(60, radian)) { //成功转弯
            //向前走5cm
            if (Robot_MoveForward(30, 20)) {
                robotInfo.xPos += 20 * std::cos(robotInfo.heading);
                robotInfo.yPos += 20 * std::sin(robotInfo.heading);
            }
        }

    }
    
    return nullptr;
}

void Planning_Start()
{
    pthread_create(&planningThreadHandle, nullptr, PlanningThread, nullptr);
}