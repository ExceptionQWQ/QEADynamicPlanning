/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "imu.h"
#include "motion.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuDecoder */
osThreadId_t imuDecoderHandle;
const osThreadAttr_t imuDecoder_attributes = {
  .name = "imuDecoder",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for robotController */
osThreadId_t robotControllerHandle;
const osThreadAttr_t robotController_attributes = {
  .name = "robotController",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMUDecoder(void *argument);
void RobotController(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of imuDecoder */
  imuDecoderHandle = osThreadNew(IMUDecoder, NULL, &imuDecoder_attributes);

  /* creation of robotController */
  robotControllerHandle = osThreadNew(RobotController, NULL, &robotController_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
      char message[128] = {0};
      snprintf(message, 128, "[imu]heading:%.4f pitch:%.4f roll:%.4f\r\n", robotIMU.heading, robotIMU.pitch, robotIMU.roll);
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMUDecoder */
/**
* @brief Function implementing the imuDecoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUDecoder */
void IMUDecoder(void *argument)
{
  /* USER CODE BEGIN IMUDecoder */
    imuTaskHandle = xTaskGetCurrentTaskHandle(); //将当前任务句柄送给imu
    /* Infinite loop */
    for(uint32_t cnt = 0; ; ++cnt)
    {
        if (HAL_UART_STATE_READY == HAL_UART_GetState(&huart2)) {
            HAL_UART_Receive_DMA(&huart2, imuRecvBuff, 128); //开启imu串口通信
        }

        uint32_t value = 0;
        if (pdTRUE == xTaskNotifyWait(0, 0xffffffff, &value, pdMS_TO_TICKS(100)) && value == 0x12345678) {
            while (!DecodeIMUPackage()) {} //解析imu数据包
        }
        osDelay(1);
    }

  /* USER CODE END IMUDecoder */
}

/* USER CODE BEGIN Header_RobotController */
/**
* @brief Function implementing the robotController thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RobotController */
void RobotController(void *argument)
{
  /* USER CODE BEGIN RobotController */
  /* Infinite loop */

    osDelay(1000);

    ClearSpeed();
    MoveForward(10);
    CommitSpeed();

    osDelay(3000);


    ClearSpeed();
    CommitSpeed();



  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RobotController */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

