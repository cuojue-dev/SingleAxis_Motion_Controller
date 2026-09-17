/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motion.h"
/* USER CODE END Includes */

/* Definitions for Motion_Task_ */
osThreadId_t Motion_Task_Handle;
const osThreadAttr_t Motion_Task__attributes = {
  .name = "Motion_Task_",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void Motion_Task(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
	// 在创建任务前完成运动模块的外设启动与状态清零
	Motion_Init();

  /* USER CODE END Init */

  /* Create the thread(s) */
  /* creation of Motion_Task_ */
  Motion_Task_Handle = osThreadNew(Motion_Task, NULL, &Motion_Task__attributes);
}

/* USER CODE BEGIN Header_Motion_Task */
/**
  * @brief  Function implementing the Motion_Task_ thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Motion_Task */
void Motion_Task(void *argument)
{
  /* USER CODE BEGIN Motion_Task */
	// osDelayUntil 使用绝对 tick，避免循环执行耗时逐步累积成周期漂移
	uint32_t next_wake_tick = osKernelGetTickCount();

	for(;;)
	{
		// 单一任务统一拥有运动状态、控制计算和 PWM 输出
		Motion_Update();

		next_wake_tick += 20;
		osDelayUntil(next_wake_tick);
	}
  /* USER CODE END Motion_Task */
}
