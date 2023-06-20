#ifndef GRADIENT_H
#define GRADIENT_H

#include "sim.h"
#include "qobj.h"

void DrawVector(cv::Mat view, cv::Point start, cv::Vec2f vector, cv::Scalar color = {255, 255, 255}, double scale = 1.0);


double CalcQOBJSValue(const std::vector<QOBJ>& qobjs, int x, int y);
std::pair<double, double> CalcQOBJSGradient(const std::vector<QOBJ>& qobjs, int x, int y);

cv::Mat GenerateScalarField(const std::vector<QOBJ>& qobjs, cv::Size fieldSz, double scale = 1.0, cv::Point offset = {0, 0});
cv::Mat GenerateGradientField(const std::vector<QOBJ>& qobjs, cv::Size fieldSz, double scale = 1.0, cv::Point offset = {0, 0});
cv::Mat GetGradientViewFromScalarField(cv::Mat scalarField, cv::Scalar color = {255, 255, 255});
cv::Mat GetGradientViewFromQOBJ(const std::vector<QOBJ>& qobjs, cv::Size viewSz,cv::Scalar color = {255, 255, 255});
cv::Mat GetViewFromVectorsField(cv::Mat vectorField, cv::Scalar color = {255, 255, 255});



#endif