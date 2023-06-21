#ifndef QOBJ_H
#define QOBJ_H

#include "qea.h"

/*
 * 数学形式：Q = -k * (q / r)
 */
class QOBJ
{
private:
    double k = 1; //比例系数
    double q; //电量
    double xPos; //在空间中的x坐标
    double yPos; //在空间中的y坐标
public:
    QOBJ() : q{0}, xPos{0}, yPos{0} {}
    QOBJ(double qValue) : q{qValue}, xPos{0}, yPos{0} {}
    QOBJ(double qValue, double x, double y) : q{qValue}, xPos{x}, yPos{y} {}
    ~QOBJ() {}

    double GetX();
    double GetY();
    void SetQ(double q);
    void SetPos(double x, double y);
    double GetValue(double x, double y);
    double GetValue(double x, double y) const;
    std::pair<double, double> GetGradient(double x, double y);
    std::pair<double, double> GetGradient(double x, double y) const;
};


#endif