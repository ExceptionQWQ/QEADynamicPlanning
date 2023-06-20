#include "view.h"

/*
 * @brief 获取点云图像
 * @param points 激光雷达点的数据
 * @param viewSz 图像大小
 * @param scale 缩放比例系数
 */
cv::Mat GetCloudView(const std::vector<LidarPointData>& points, cv::Size viewSz, double scale)
{
    cv::Mat view = cv::Mat::zeros(viewSz, CV_8UC3);
    int cx = viewSz.width / 2, cy = viewSz.height / 2; //图像中心坐标
    for (size_t index = 0; index != points.size(); ++index) {
        int x = cx + points[index].distance * std::cos(PI / 180 * points[index].angle) * scale;
        int y = cy + points[index].distance * std::sin(PI / 180 * points[index].angle) * scale;
        cv::circle(view, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
    }
    cv::flip(view, view, 0); //图像上下翻转
    return view;
}