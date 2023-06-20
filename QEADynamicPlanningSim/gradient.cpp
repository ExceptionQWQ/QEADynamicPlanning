#include "gradient.h"

/*
 * @brief 绘制向量
 * @param view 要绘制的图像
 * @param start 起点坐标
 * @param vector 向量
 * @param color 颜色
 * @param scale 缩放系数
 */
void DrawVector(cv::Mat view, cv::Point start, cv::Vec2f vector, cv::Scalar color, double scale)
{
    double len = std::sqrt(std::pow(vector[0], 2) + std::pow(vector[1], 2));
    double stlScale = 20 / len;
    vector[0] *= stlScale; vector[1] *= stlScale; //将向量长度统一成20
    vector[0] *= scale; vector[1] *= scale; //缩放
    cv::Point end = cv::Point(start.x + vector[0], start.y + vector[1]); //终点坐标
    cv::line(view, start, end, color, 1); 
    cv::circle(view, end, 2, color, -1); //用圆标记向量方向
}

/*
 * @brief 获取向量场的图像
 * @param vecField 向量场
 * @param color 颜色
 */
cv::Mat GetVectorFieldView(cv::Mat vecField, cv::Scalar color)
{
    cv::Mat view = cv::Mat::zeros(vecField.size(), CV_8UC3);
    int stepX = view.cols / 20, stepY = view.rows / 20;

    for (int x = 0; x < view.cols - 1; x += stepX) {
        for (int y = 0; y < view.rows - 1; y += stepY) {
            double dx = vecField.at<float>(y, x + 1) - vecField.at<float>(y, x);
            double dy = vecField.at<float>(y + 1, x) - vecField.at<float>(y, x);
            DrawVector(view, cv::Point(x, y), cv::Vec2f(dx, dy), color, 1);
        }
    }

    return view;
}