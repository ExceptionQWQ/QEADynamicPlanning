#ifndef GRADIENT_H
#define GRADIENT_H

#include "sim.h"


void DrawVector(cv::Mat view, cv::Point start, cv::Vec2f vector, cv::Scalar color = {255, 255, 255}, double scale = 1.0);

cv::Mat GetVectorFieldView(cv::Mat vecField, cv::Scalar color = {255, 255, 255});

#endif