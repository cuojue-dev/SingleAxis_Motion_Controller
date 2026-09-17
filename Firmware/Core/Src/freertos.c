/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include <stdio.h>

#include "tim.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	MOTION_STATE_INIT = 0,
	MOTION_STATE_IDLE,
	MOTION_STATE_READY,
	MOTION_STATE_RUN_SPEED,
	MOTION_STATE_RUN_POSITION,
	MOTION_STATE_HOMING,
	MOTION_STATE_FAULT
}motion_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTEGRAL_MIN  -1000
#define INTEGRAL_MAX  1000
#define HOMING_TIMEOUT_MS 10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile UBaseType_t motion_task_hwm_words = 0;
static float s_speed_p_term = 0;
static float s_speed_integral = 0;
static float s_trajectory_rpm = 0.0f;

/* USER CODE END Variables */
/* Definitions for Motion_Task_ */
osThreadId_t Motion_Task_Handle;
const osThreadAttr_t Motion_Task__attributes = {
  .name = "Motion_Task_",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Motor_Set_Signed_PWM(int32_t pwm);
static float Motor_Calculate_Feedforward(float target_rpm);
static float Speed_Controller_Update(float target_rpm, float motor_rpm,
									 float Kp, float Ki);
static void Speed_Controller_Reset(void);
static float Position_Controller_Update(int32_t target_position,
										int32_t current_position,
										float Kp_position,
										float max_position_rpm,
										int32_t position_tolerance);
static float Trajectory_Update(float position_target_rpm, float max_acceleration_rpm_s);
static void Trajectory_Reset(void);
/* USER CODE END FunctionPrototypes */

void Motion_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim4, 0);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

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
  /* creation of Motion_Task_ */
  Motion_Task_Handle = osThreadNew(Motion_Task, NULL, &Motion_Task__attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

	char message[50];
	uint16_t length = 0;

	uint16_t current_count = 0;
	uint16_t last_count = 0;
	int16_t delta_count = 0;

	float motor_rpm;

	int32_t pwm_output;

	float Kp = 5.0f;
	float Ki = 10.0f;

	uint32_t next_wake_tick = osKernelGetTickCount();

	motion_state_t motion_state = MOTION_STATE_INIT;
	uint8_t position_command_pending = 1;

	int32_t position_count = 0;
	int32_t target_position = 2800;
	int32_t position_tolerance = 100;

	float Kp_position = 0.05f;
	float max_position_rpm = 60.0f;

	float position_target_rpm;
	float trajectory_rpm;
	float max_acceleration_rpm_s = 100.0f;

	uint8_t home_command_pending = 0;
	float home_rpm = -40.0f;

	uint32_t homing_start_tick = 0;

  /* Infinite loop */
  for(;;)
  {
	  switch(motion_state)
	  {
	  	  case MOTION_STATE_INIT:
	  		  Motor_Set_Signed_PWM(0);
	  		  motion_state = MOTION_STATE_IDLE;
	  		  break;

	  	  case MOTION_STATE_IDLE:
	  		  Motor_Set_Signed_PWM(0);
	  		  motion_state = MOTION_STATE_READY;
	  		  break;

	  	  case MOTION_STATE_READY:
	  		  Trajectory_Reset();
	  		  Speed_Controller_Reset();
	  		  Motor_Set_Signed_PWM(0);

	  		  if(home_command_pending)
	  		  {
	  			  home_command_pending = 0;
	  			  homing_start_tick = osKernelGetTickCount();
	  			  motion_state = MOTION_STATE_HOMING;
	  		  }
	  		  else if(position_command_pending)
	  		  {
	  			  position_command_pending = 0;
	  			  motion_state = MOTION_STATE_RUN_POSITION;
	  		  }
	  		  break;

	  	  case MOTION_STATE_RUN_POSITION:
	  		  current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

	  		  delta_count = (int16_t)(uint16_t)(current_count - last_count);
	  		  last_count = current_count;

	  		  position_count += delta_count;

	  		  motor_rpm = (float)delta_count * 60.0f / (2800.0f * 0.1f);

	  		  position_target_rpm = Position_Controller_Update(target_position, position_count, Kp_position, max_position_rpm, position_tolerance);

	  		  trajectory_rpm = Trajectory_Update(position_target_rpm, max_acceleration_rpm_s);

	  		  pwm_output = Speed_Controller_Update(trajectory_rpm , motor_rpm, Kp, Ki);

	  		  Motor_Set_Signed_PWM(pwm_output);

	  		  if(position_count >= target_position - position_tolerance &&
	  			 position_count <= target_position + position_tolerance &&
				 trajectory_rpm == 0.0f &&
				 motor_rpm >= -5.0f &&
				 motor_rpm <= 5.0f)
	  		  {
	  			  motion_state = MOTION_STATE_READY;
	  		  }
	  		  break;

	  	  case MOTION_STATE_RUN_SPEED:
	  		  Motor_Set_Signed_PWM(0);
	  		  break;

	  	  case MOTION_STATE_HOMING:
	  		  current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

	  		  delta_count = (int16_t)(uint16_t)(current_count - last_count);
	  		  last_count = current_count;

	  		  position_count += delta_count;

	  		  motor_rpm = (float)delta_count * 60.0f / (2800.0f * 0.1f);

	  		  pwm_output = Speed_Controller_Update(home_rpm, motor_rpm, Kp, Ki);

	  		  Motor_Set_Signed_PWM(pwm_output);

	  		  //仅用于日志显示当前homing目标
	  		  position_target_rpm = home_rpm;
	  		  trajectory_rpm = home_rpm;

	  		  if(HAL_GPIO_ReadPin(HOME_SIM_GPIO_Port, HOME_SIM_Pin) == GPIO_PIN_SET)
	  		  {
	  			  Motor_Set_Signed_PWM(0);

	  			  __HAL_TIM_SET_COUNTER(&htim4, 0);

	  			  last_count = 0;
	  			  position_count = 0;

	  			  Trajectory_Reset();
	  			  Speed_Controller_Reset();

	  			  motion_state = MOTION_STATE_READY;
	  		  }
	  		  else if((osKernelGetTickCount() - homing_start_tick) >= HOMING_TIMEOUT_MS)
	  		  {
	  			  Motor_Set_Signed_PWM(0);

	  			  Trajectory_Reset();
	  			  Speed_Controller_Reset();

	  			  motion_state = MOTION_STATE_FAULT;
	  		  }
	  		  break;

	  	  case MOTION_STATE_FAULT:
	  		  Motor_Set_Signed_PWM(0);
	  		  break;

	  	  default:
	  		  Motor_Set_Signed_PWM(0);
	  		  motion_state = MOTION_STATE_FAULT;
	  		  break;
	  }

	  length = snprintf(message, sizeof(message),
	  				  "axis:%ld,%ld\r\n",
					  (long)target_position,
	  				  (long)position_count);

	  HAL_UART_Transmit(&huart1, (uint8_t *)message, (uint16_t)length, 100);

	  motion_task_hwm_words = uxTaskGetStackHighWaterMark(NULL);

	  next_wake_tick += 100;
	  osDelayUntil(next_wake_tick);
  }
  /* USER CODE END Motion_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void Motor_Set_Signed_PWM(int32_t pwm)
{
	if(pwm > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
	else if(pwm < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -pwm);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}

static float Motor_Calculate_Feedforward(float target_rpm)
{
	float pwm_magnitude;
	float target_rpm_abs;

	if(target_rpm == 0)
	{
		return 0.0f;
	}
	else if(target_rpm < 0)
	{
		target_rpm_abs = -target_rpm;
	}
	else
	{
		target_rpm_abs = target_rpm;
	}

	pwm_magnitude = 2642.0f + 12.7f * (target_rpm_abs - 60.0f);

	if(pwm_magnitude > 3599.0f)
	{
		pwm_magnitude = 3599.0f;
	}
	else if(pwm_magnitude < 0.0f)
	{
		pwm_magnitude = 0.0f;
	}

	if(target_rpm < 0)
	{
		return -pwm_magnitude;
	}
	else
	{
		return pwm_magnitude;
	}
}

static float Speed_Controller_Update(float target_rpm, float motor_rpm,
									 float Kp, float Ki)
{
	float feedforward;
	float speed_error;
	float raw_output;
	float raw_before;
	uint8_t allow_integral;

	if(target_rpm == 0.0f)
	{
		Speed_Controller_Reset();
		return 0.0f;
	}

	feedforward = Motor_Calculate_Feedforward(target_rpm);

	speed_error = target_rpm - motor_rpm;

	s_speed_p_term = Kp * speed_error;

	raw_before = feedforward + s_speed_p_term +s_speed_integral;

	allow_integral = 0;

	if(raw_before > -3599.0f && raw_before < 3599.0f)
	{
		allow_integral = 1;
	}
	else if(raw_before >= 3599.0f && speed_error < 0)
	{
		allow_integral = 1;
	}
	else if(raw_before <= -3599.0f && speed_error > 0)
	{
		allow_integral = 1;
	}

	if(allow_integral)
	{
		s_speed_integral += Ki * speed_error * 0.1f;
	}

	if(s_speed_integral < INTEGRAL_MIN)
	{
		s_speed_integral = INTEGRAL_MIN;
	}
	else if(s_speed_integral > INTEGRAL_MAX)
	{
		s_speed_integral = INTEGRAL_MAX;
	}

	raw_output = feedforward + s_speed_p_term + s_speed_integral;

	if(raw_output > 3599.0f)
	{
		raw_output = 3599.0f;
	}
	else if(raw_output < -3599.0f)
	{
		raw_output = -3599.0f;
	}

	return raw_output;
}

static void Speed_Controller_Reset(void)
{
	s_speed_p_term = 0;
	s_speed_integral = 0;
}

static float Position_Controller_Update(int32_t target_position,
										int32_t current_position,
										float Kp_position,
										float max_position_rpm,
										int32_t position_tolerance)
{
	int32_t position_error;
	float target_rpm;

	position_error = target_position - current_position;

	if(position_error >= -position_tolerance &&
	   position_error <= position_tolerance)
	{
		return 0.0f;
	}

	target_rpm = Kp_position * position_error;

	if(target_rpm > max_position_rpm)
	{
		target_rpm = max_position_rpm;
	}
	else if(target_rpm < -max_position_rpm)
	{
		target_rpm = -max_position_rpm;
	}

	return target_rpm;
}

static float Trajectory_Update(float position_target_rpm, float max_acceleration_rpm_s)
{
	float max_delta_rpm;

	max_delta_rpm = max_acceleration_rpm_s * 0.1f;

	if(position_target_rpm > s_trajectory_rpm + max_delta_rpm)
	{
		s_trajectory_rpm += max_delta_rpm;
	}
	else if(position_target_rpm < s_trajectory_rpm - max_delta_rpm)
	{
		s_trajectory_rpm -= max_delta_rpm;
	}
	else
	{
		s_trajectory_rpm = position_target_rpm;
	}

	return s_trajectory_rpm;
}

static void Trajectory_Reset(void)
{
	s_trajectory_rpm = 0.0f;
}
/* USER CODE END Application */

