#ifndef MAPPING_H
#define MAPPING_H

#include "qea.h"
#include "lidar.h"

class Mapping
{
private:
    cv::Mat view;
    cv::Size sz;
    cv::Point offset;
    double scale;
    cv::Scalar color;
    double xPos = 0;
    double yPos = 0;
    double heading = 0;
public:

    Mapping(cv::Size viewSz, cv::Point mapOffset = {0, 0}, double mapScale = 1.0) : sz{viewSz}, offset{mapOffset}, scale{mapScale} 
    {
        view = cv::Mat::zeros(sz, CV_8UC3);
    }
    void SetColor(cv::Scalar color);
    void SetRobotPos(double x, double y);
    void SetRobotHeading(double heading);
    void UpdateLidarPoints(const std::vector<LidarPointData>& points);
    cv::Mat GetMapView();
};

#endif