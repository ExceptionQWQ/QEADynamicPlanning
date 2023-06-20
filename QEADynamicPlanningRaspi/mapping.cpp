#include "mapping.h"


void Mapping::SetColor(cv::Scalar color)
{
    this->color = color;
}

void Mapping::SetRobotPos(double x, double y)
{
    this->xPos = x;
    this->yPos = y;
}

void Mapping::SetRobotHeading(double heading)
{
    this->heading = heading;
}

cv::Mat Mapping::GetMapView()
{
    cv::Mat mapView = view.clone();
    cv::flip(mapView, mapView, 0);
    return mapView;
}

void Mapping::UpdateLidarPoints(const std::vector<LidarPointData>& points)
{
    for (size_t index = 0; index != points.size(); ++index) {
        double x = (xPos + points[index].distance * std::cos(PI / 180 * points[index].angle + heading)) * scale + offset.x;
        double y = (yPos + points[index].distance * std::sin(PI / 180 * points[index].angle + heading)) * scale + offset.y;
        cv::circle(view, cv::Point(x, y), 3, color, -1);
    }
}