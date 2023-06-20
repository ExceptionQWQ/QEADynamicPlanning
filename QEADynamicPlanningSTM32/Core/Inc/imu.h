#ifndef IMU_H
#define IMU_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"



extern uint8_t imuRecvBuff[128];
extern volatile int imuRecvStatus;
extern volatile uint8_t* imuPackage; //等待处理的数据包
extern uint8_t imuBuff[1024];
extern volatile uint32_t imuBuffOffset;

struct MSG_IMU
{
    float Gyroscope_X; //机体系X轴角速度
    float Gyroscope_Y; //机体系Y轴角速度
    float Gyroscope_Z; //机体系Z轴角速度
    float Accelerometer_X; //机体系X轴加速度(未分离重力加速度)
    float Accelerometer_Y; //机体系Y轴加速度(未分离重力加速度)
    float Accelerometer_Z; //机体系Z轴加速度(未分离重力加速度)
    float Magnetometer_X; //机体系X轴磁感应强度
    float Magnetometer_Y; //机体系Y轴磁感应强度
    float Magnetometer_Z; //机体系Z轴磁感应强度
    float IMU_Temperature; //如果IMU数据由多个传感器组成则该值为这些传感器的平均温度
    float Pressure; //气压值
    float Pressure_Temperature; //气压计的温度值
    int64_t Timestamp; //时间戳
};

struct MSG_AHRS
{
    float RollSpeed; //横滚角速度
    float PitchSpeed; //俯仰角速度
    float HeadingSpeed; //偏航角速度
    float Roll; //横滚
    float Pitch; //俯仰
    float Heading; //偏航
    float Q1; //四元数Q1
    float Q2; //四元数Q2
    float Q3; //四元数Q3
    float Q4; //四元数Q4
    int64_t Timestamp; //时间戳
};

volatile struct RobotIMU
{
    double roll;
    double pitch;
    double heading;
};

extern volatile struct RobotIMU robotIMU;
extern osThreadId_t imuTaskHandle;

void IMU_RxCpltCallback();
void IMU_RxHalfCpltCallback();
int DecodeIMUPackage();

#endif