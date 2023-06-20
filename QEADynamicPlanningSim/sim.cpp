#include "sim.h"
#include "ransac.h"
#include "gradient.h"
#include "qobj.h"

void Sim()
{
    std::vector<QOBJ> qobjs;

    cv::Mat view = cv::Mat::zeros(cv::Size(1024, 1024), CV_8UC3);

    //绘制地图上边界
    for (int i = 0; i < 1024; i += 5) {
        cv::circle(view, cv::Point(i, 10), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, i, 10);
    }
    //绘制地图下边界
    for (int i = 0; i < 1024; i += 5) {
        cv::circle(view, cv::Point(i, 1000), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, i, 1000);
    }
    //绘制地图左边界
    for (int i = 0; i < 1024; i += 5) {
        cv::circle(view, cv::Point(10, i), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, 10, i);
    }
    //绘制地图右边界
    for (int i = 0; i < 1024; i += 5) {
        cv::circle(view, cv::Point(1000, i), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, 1000, i);
    }

    //绘制方形垃圾桶1 上边界
    for (int i = 300; i < 400; i += 5) {
        cv::circle(view, cv::Point(i, 300), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, i, 300);
    }
    //绘制方形垃圾桶1 下边界
    for (int i = 300; i < 400; i += 5) {
        cv::circle(view, cv::Point(i, 400), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, i, 400);
    }
    //绘制方形垃圾桶1 左边界
    for (int i = 300; i < 400; i += 5) {
        cv::circle(view, cv::Point(300, i), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, 300, i);
    }
    //绘制方形垃圾桶1 右边界
    for (int i = 300; i < 400; i += 5) {
        cv::circle(view, cv::Point(400, i), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, 400, i);
    }

    //绘制方形垃圾桶2 上边界
    for (int i = 700; i < 800; i += 5) {
        cv::circle(view, cv::Point(i, 300), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, i, 300);
    }
    //绘制方形垃圾桶2 下边界
    for (int i = 700; i < 800; i += 5) {
        cv::circle(view, cv::Point(i, 400), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, i, 400);
    }
    //绘制方形垃圾桶2 左边界
    for (int i = 300; i < 400; i += 5) {
        cv::circle(view, cv::Point(700, i), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, 700, i);
    }
    //绘制方形垃圾桶2 右边界
    for (int i = 300; i < 400; i += 5) {
        cv::circle(view, cv::Point(800, i), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, 800, i);
    }

    //绘制方形垃圾桶3 上边界
    for (int i = 700; i < 800; i += 5) {
        cv::circle(view, cv::Point(i, 700), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, i, 700);
    }
    //绘制方形垃圾桶3 下边界
    for (int i = 700; i < 800; i += 5) {
        cv::circle(view, cv::Point(i, 800), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, i, 800);
    }
    //绘制方形垃圾桶3 左边界
    for (int i = 700; i < 800; i += 5) {
        cv::circle(view, cv::Point(700, i), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, 700, i);
    }
    //绘制方形垃圾桶3 右边界
    for (int i = 700; i < 800; i += 5) {
        cv::circle(view, cv::Point(800, i), 5, cv::Scalar(0, 255, 0), -1);
        qobjs.emplace_back(5, 800, i);
    }
    
    //起点
    cv::circle(view, cv::Point(50, 980), 20, cv::Scalar(255, 0, 0), -1);
    qobjs.emplace_back(50, 50, 980);

    //地图右上角绘制圆形垃圾桶
    cv::circle(view, cv::Point(950, 50), 20, cv::Scalar(0, 0, 255), -1);
    qobjs.emplace_back(-6000, 950, 50);


    cv::Mat fieldView = GetGradientViewFromQOBJ(qobjs, cv::Size(1024, 1024), cv::Scalar(255, 255, 0));

    cv::Point start(50, 980);
    for (int i = 0; i < 100; ++i) {
        auto grad = CalcQOBJSGradient(qobjs, start.x, start.y);
        double len = std::sqrt(std::pow(grad.first, 2) + std::pow(grad.second, 2));
        double scale = 20 / len;
        grad.first *= scale; grad.second *= scale;
        std::cout << grad.first << " " << grad.second << std::endl;
        cv::Point end(start.x + grad.first, start.y + grad.second);
        cv::line(view, start, end, cv::Scalar(0, 0, 255), 8);
        cv::line(fieldView, start, end, cv::Scalar(0, 0, 255), 8);
        start = end;
    }

    cv::imshow("view", view);
    cv::imshow("fieldView", fieldView);
    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    srand(time(0));


    Sim();

    return 0;
}