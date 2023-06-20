#ifndef VIEW_H
#define VIEW_H

#include "qea.h"
#include "lidar.h"

cv::Mat GetCloudView(const std::vector<LidarPointData>& points, cv::Size viewSz, double scale = 1.0);


#endif