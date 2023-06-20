#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "math.h"
#include "imu.h"
#include "message_buffer.h"
#include "motion.h"


extern uint8_t robotRecvBuff[1024];
extern int robotRecvOffset;

extern MessageBufferHandle_t messageBufferHandle;


void Robot_RxCpltCallback();

#endif