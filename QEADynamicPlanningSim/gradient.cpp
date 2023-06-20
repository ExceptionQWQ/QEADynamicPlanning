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
 * @brief 由qobj生成标量场
 * @param qobjs 电荷对象
 * @param fieldSz 场大小
 * @param scale 缩放
 * @param offset 偏移
 */
cv::Mat GenerateScalarField(const std::vector<QOBJ>& qobjs, cv::Size fieldSz, double scale, cv::Point offset)
{
    cv::Mat field = cv::Mat::zeros(fieldSz, CV_32FC1);
    for (int x = 0; x < field.cols; ++x) {
        for (int y = 0; y < field.rows; ++y) {
            int tx = x * scale + offset.x;
            int ty = y * scale + offset.y;
            float phi = 0;
            for (size_t qindex = 0; qindex != qobjs.size(); ++qindex) {
                phi += qobjs[qindex].GetValue(tx, ty);
            }
            field.at<float>(y, x) = phi;
        }
    }
    return field;
}

/*
 * @brief 获取梯度图像
 * @param scalarField 标量场
 * @param color 颜色
 */
cv::Mat GetGradientViewFromScalarField(cv::Mat scalarField, cv::Scalar color)
{
    cv::Mat view = cv::Mat::zeros(scalarField.size(), CV_8UC3);
    int stepX = view.cols / 30, stepY = view.rows / 30;

    for (int x = 0; x < view.cols - 1; x += stepX) {
        for (int y = 0; y < view.rows - 1; y += stepY) {
            double dx = scalarField.at<float>(y, x + 1) - scalarField.at<float>(y, x);
            double dy = scalarField.at<float>(y + 1, x) - scalarField.at<float>(y, x);
            DrawVector(view, cv::Point(x, y), cv::Vec2f(dx, dy), color, 1);
        }
    }

    return view;
}