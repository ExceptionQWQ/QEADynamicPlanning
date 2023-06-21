#include "qobj.h"


double QOBJ::GetX()
{
    return this->xPos;
}

double QOBJ::GetY()
{
    return this->yPos;
}


void QOBJ::SetQ(double q)
{
    this->q = q;
}

void QOBJ::SetPos(double x, double y)
{
    this->xPos = x;
    this->yPos = y;
}

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
    double tx = x - xPos, ty = y - yPos;
    double sb = std::pow(std::sqrt(std::pow(tx, 2) + std::pow(ty, 2) + 0.0001), 3);
    double partialQOverpartialX = k * q * tx / sb;
    double partialQOverpartialY = k * q * ty / sb;
    return std::make_pair(partialQOverpartialX, partialQOverpartialY);
}

std::pair<double, double> QOBJ::GetGradient(double x, double y) const
{
    double tx = x - xPos, ty = y - yPos;
    double sb = std::pow(std::sqrt(std::pow(tx, 2) + std::pow(ty, 2) + 0.0001), 3);
    double partialQOverpartialX = k * q * tx / sb;
    double partialQOverpartialY = k * q * ty / sb;
    return std::make_pair(partialQOverpartialX, partialQOverpartialY);
}