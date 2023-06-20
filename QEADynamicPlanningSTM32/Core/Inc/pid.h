#ifndef PID_H
#define PID_H


enum PIDType
{
    PIDType_Pos = 1, //位置式
    PIDType_Inc = 2, //增量式
};

volatile struct PIDHanldeDef
{
    double kp; //p比例系数
    double ki; //i比例系数
    double kd; //d比例系数
    double value; //实际测量值
    double target; //目标值
    double error; //误差
    double totalError; //总误差
    double lastError; //上一次误差
    double lastError2; //上上一次误差
    double minPWM; //最小输出PWM
    double maxPWM; //最大输出PWM
    double pwm; //最终输出PWM
    enum PIDType pidType; //PID算法类型
};

void CreatePID(volatile struct PIDHanldeDef* def, enum PIDType pidType, double kp, double ki, double kd, double minPWM, double maxPWM);
double PIDTick(volatile struct PIDHanldeDef* def, double newValue);

#endif