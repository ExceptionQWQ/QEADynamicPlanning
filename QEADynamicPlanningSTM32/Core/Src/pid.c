#include "pid.h"

/*
 * @brief 创建PID
 * @param def pid句柄指针
 * @param pidType pid类型（位置式=1/增量式=2）
 * @param kp p比例系数
 * @param ki i比例系数
 * @param kd d比例系数
 * @param minPWM 最小输出PWM
 * @param maxPWM 最大输出PWM
 */
void CreatePID(volatile struct PIDHanldeDef* def, enum PIDType pidType, double kp, double ki, double kd, double minPWM, double maxPWM)
{
    def->kp = kp;
    def->ki = ki;
    def->kd = kd;
    def->value = 0;
    def->target = 0;
    def->error = 0;
    def->totalError = 0;
    def->lastError = 0;
    def->lastError2 = 0;
    def->minPWM = minPWM;
    def->maxPWM = maxPWM;
    def->pwm = 0;
    def->pidType = pidType;
}


void PosTick(volatile struct PIDHanldeDef* def)
{
    def->lastError = def->error;
    def->error = def->target - def->value;
    def->totalError += def->error;

    double pwm = def->kp * def->error +
                 def->ki * def->totalError +
                 def->kd * (def->error - def->lastError);
    def->pwm += pwm;
    if (def->pwm > def->maxPWM) def->pwm = def->maxPWM;
    if (def->pwm < def->minPWM) def->pwm = def->minPWM;
}

void IncTick(volatile struct PIDHanldeDef* def)
{
    def->lastError2 = def->lastError;
    def->lastError = def->error;
    def->error = def->target - def->value;

    double pwm = def->kp * (def->error - def->lastError) +
                 def->ki * def->error +
                 def->kd * (def->error - 2 * def->lastError + def->lastError2);
    def->pwm += pwm;
    if (def->pwm > def->maxPWM) def->pwm = def->maxPWM;
    if (def->pwm < def->minPWM) def->pwm = def->minPWM;
}

/*
 * @brief 计算一次pid，并返回pwm
 * @param def pid句柄指针
 * @param newValue 更新后的值(速度/位置等)
 */
double PIDTick(volatile struct PIDHanldeDef* def, double newValue)
{
    def->value = newValue;
    switch (def->pidType) {
        case PIDType_Pos: //位置式PID
            PosTick(def);
            break;
        case PIDType_Inc: //增量式PID
            IncTick(def);
            break;
    }
    return def->pwm;
}