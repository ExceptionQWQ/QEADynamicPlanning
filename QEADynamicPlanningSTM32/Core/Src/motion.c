#include "motion.h"


volatile struct RobotMotion robotMotion;

/*
 * @brief 清除左右轮速度
 */
void ClearSpeed()
{
    robotMotion.VL = 0;
    robotMotion.VR = 0;
}

/*
 * @brief 提交左右轮速度，会实际更新到小车上
 */
void CommitSpeed()
{
    leftPWM.pidHanldeDef.target = robotMotion.VL;
    rightPWM.pidHanldeDef.target = -robotMotion.VR;
}

/*
 * @brief 向前移动，不会实际更新到小车上
 * @param speed 移动速度
 */
void MoveForward(double speed)
{
    robotMotion.VL += speed;
    robotMotion.VR += speed;
}

/*
 * @brief 向后移动，不会实际更新到小车上
 * @param speed 移动速度
 */
void MoveBackward(double speed)
{
    robotMotion.VL -= speed;
    robotMotion.VR -= speed;
}

/*
 * @brief 向左转，不会实际更新到小车上
 * @param speed 移动速度
 */
void SpinLeft(double speed)
{
    robotMotion.VL -= speed;
    robotMotion.VR += speed;
}

/*
 * @brief 向右转，不会实际更新到小车上
 * @param speed 移动速度
 */
void SpinRight(double speed)
{
    robotMotion.VL += speed;
    robotMotion.VR -= speed;
}

/*
 * @brief 向前移动一定距离, 调用此函数会一直阻塞到移动结束
 * @param speed 移动速度
 * @param dis 移动距离
 */
void MoveForwardWithDis(double speed, double dis)
{
    volatile struct PIDHanldeDef defL; //定义pid控制句柄
    volatile struct PIDHanldeDef defR; //定义pid控制句柄
    CreatePID(&defL, PIDType_Pos, 0.05, 0, 2, -speed, speed); //创建句柄
    CreatePID(&defR, PIDType_Pos, 0.05, 0, 2, -speed, speed); //创建句柄
    leftPWM.dis = 0; //清除距离
    rightPWM.dis = 0; //清除距离
    defL.target = dis; //目标距离
    defR.target = dis; //目标距离
    while (1) {
        if (fabs(leftPWM.dis - dis) < 10 && fabs(rightPWM.dis - dis) < 10) break; //左右轮同时到达目标位置后才break
        double retVL = PIDTick(&defL, leftPWM.dis);
        double retVR = PIDTick(&defR, rightPWM.dis);
        ClearSpeed();
        robotMotion.VL += retVL;
        robotMotion.VR += retVR;
        CommitSpeed();
        osDelay(1);
    }
    //停止运动
    ClearSpeed();
    CommitSpeed();
}

/*
 * @brief 向后移动一定距离, 调用此函数会一直阻塞到移动结束
 * @param speed 移动速度
 * @param dis 移动距离
 */
void MoveBackwardWithDis(double speed, double dis)
{
    MoveForwardWithDis(speed, -dis);
}

/*
 * @brief 实现小车转弯到指定角度, 调用此函数会一直阻塞到移动结束
 * @param speed 转弯的速度
 * @param radian 要转到的弧度
 */
void SpinTo(double speed, double radian)
{
    while (1) {
        double dr = fabs(radian - robotIMU.heading);
        if (dr < 0.001) break;
        double maxSpeed = speed;
        double speed = dr * 140;
        if (speed > maxSpeed) speed = maxSpeed;
        if (radian - robotIMU.heading > 0) {
            if (radian - robotIMU.heading > PI) {
                ClearSpeed();
                SpinRight(speed);
                CommitSpeed();
            } else {
                ClearSpeed();
                SpinLeft(speed);
                CommitSpeed();
            }
        } else {
            if (radian - robotIMU.heading > -PI) {
                ClearSpeed();
                SpinRight(speed);
                CommitSpeed();
            } else {
                ClearSpeed();
                SpinLeft(speed);
                CommitSpeed();
            }
        }
    }
    ClearSpeed();
    CommitSpeed();
}