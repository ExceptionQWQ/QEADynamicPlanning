#ifndef RANSAC_H
#define RANSAC_H

#include "qea.h"

void DrawLine(cv::Mat view, double k, double z, cv::Scalar color = {255, 255, 255});
void DrawCircle(cv::Mat view, double x, double y, double r, cv::Scalar color = {255, 255, 255});
cv::Mat GenerateLineView();
cv::Mat GenerateLineSegmentView();
cv::Mat GenerateCircleView();
cv::Mat MakeNoise(cv::Mat view);
std::pair<cv::Point, double> CalcCircleFromThreePoints(cv::Point p1, cv::Point p2, cv::Point p3);
std::vector<cv::Point2d> ConvertViewToPoints(cv::Mat view);
std::optional<std::pair<double, double>> FindLineRansac(const std::vector<cv::Point2d>& points);
std::optional<std::pair<cv::Point, cv::Point>> FindLineSegmentRansac(const cv::Mat& view, double minLen = 10, double maxLen = 999999);
std::optional<std::pair<cv::Point, double>> FindCircleRansac(const std::vector<cv::Point2d>& points, double minR = 0, double maxR = 999999);



#endif