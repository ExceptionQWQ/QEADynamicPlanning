#include "qobj.h"

/*
 * @brief 获取x，y坐标处的值
 */
double QOBJ::GetValue(double x, double y)
{
    double r = std::sqrt(std::pow(x - xPos, 2) + std::pow(y - yPos, 2) + 0.0001);
    return -k * q / r;
}

double QOBJ::GetValue(double x, double y) const
{
    double r = std::sqrt(std::pow(x - xPos, 2) + std::pow(y - yPos, 2) + 0.0001);
    return -k * q / r;
}

std::pair<double, double> QOBJ::GetGradient(double x, double y)
{
    
    return std::make_pair(0, 0);
}

std::pair<double, double> QOBJ::GetGradient(double x, double y) const
{

    return std::make_pair(0, 0);
}