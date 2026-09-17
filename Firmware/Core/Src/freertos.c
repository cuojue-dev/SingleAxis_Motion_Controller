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
	uint32_t next_wake_tick = osKernelGetTickCount();

	for(;;)
	{
		Motion_Update();

		next_wake_tick += 100;
		osDelayUntil(next_wake_tick);
	}
  /* USER CODE END Motion_Task */
}
