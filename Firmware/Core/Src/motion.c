#include "motion.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#define HOMING_TIMEOUT_MS 10000

typedef enum
{
	MOTION_STATE_INIT = 0,
	MOTION_STATE_IDLE,
	MOTION_STATE_READY,
	MOTION_STATE_RUN_SPEED,
	MOTION_STATE_RUN_POSITION,
	MOTION_STATE_HOMING,
	MOTION_STATE_FAULT
} motion_state_t;

static uint16_t s_current_count = 0;
static uint16_t s_last_count = 0;
static int16_t s_delta_count = 0;
static float s_motor_rpm = 0.0f;
static int32_t s_pwm_output = 0;

static motion_state_t s_motion_state = MOTION_STATE_INIT;
static uint8_t s_position_command_pending = 1;
static int32_t s_position_count = 0;
static int32_t s_target_position = 2800;
static int32_t s_position_tolerance = 100;
static float s_Kp_position = 0.05f;
static float s_max_position_rpm = 60.0f;
static float s_position_target_rpm = 0.0f;
static float s_trajectory_rpm = 0.0f;
static float s_max_acceleration_rpm_s = 100.0f;

static uint8_t s_home_command_pending = 0;
static float s_home_rpm = -40.0f;
static uint32_t s_homing_start_tick = 0;

static float s_Kp = 5.0f;
static float s_Ki = 10.0f;
static char s_message[50];

volatile UBaseType_t motion_task_hwm_words = 0;

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

static void Motion_Update_Feedback(void)
{
	s_current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
	s_delta_count = (int16_t)(uint16_t)(s_current_count - s_last_count);
	s_last_count = s_current_count;
	s_position_count += s_delta_count;
	s_motor_rpm = (float)s_delta_count * 60.0f / (2800.0f * 0.1f);
}

static void Motion_Send_Debug_Log(void)
{
	uint16_t length;

	length = snprintf(s_message, sizeof(s_message),
					  "axis:%ld,%ld\r\n",
					  (long)s_target_position,
					  (long)s_position_count);

	HAL_UART_Transmit(&huart1, (uint8_t *)s_message, length, 100);
	motion_task_hwm_words = uxTaskGetStackHighWaterMark(NULL);
}

void Motion_Init(void)
{
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	s_current_count = 0;
	s_last_count = 0;
	s_delta_count = 0;
	s_motor_rpm = 0.0f;
	s_pwm_output = 0;
	s_motion_state = MOTION_STATE_INIT;
	s_position_command_pending = 1;
	s_position_count = 0;
	s_target_position = 2800;
	s_position_tolerance = 100;
	s_Kp_position = 0.05f;
	s_max_position_rpm = 60.0f;
	s_position_target_rpm = 0.0f;
	s_trajectory_rpm = 0.0f;
	s_max_acceleration_rpm_s = 100.0f;
	s_home_command_pending = 0;
	s_home_rpm = -40.0f;
	s_homing_start_tick = 0;
	s_Kp = 5.0f;
	s_Ki = 10.0f;

	Controller_Trajectory_Reset();
	Controller_Speed_Reset();
}

void Motion_Update(void)
{
	switch(s_motion_state)
	{
		case MOTION_STATE_INIT:
			Motor_Set_Signed_PWM(0);
			s_motion_state = MOTION_STATE_IDLE;
			break;

		case MOTION_STATE_IDLE:
			Motor_Set_Signed_PWM(0);
			s_motion_state = MOTION_STATE_READY;
			break;

		case MOTION_STATE_READY:
			Controller_Trajectory_Reset();
			Controller_Speed_Reset();
			Motor_Set_Signed_PWM(0);

			if(s_home_command_pending)
			{
				s_home_command_pending = 0;
				s_homing_start_tick = osKernelGetTickCount();
				s_motion_state = MOTION_STATE_HOMING;
			}
			else if(s_position_command_pending)
			{
				s_position_command_pending = 0;
				s_motion_state = MOTION_STATE_RUN_POSITION;
			}
			break;

		case MOTION_STATE_RUN_POSITION:
			Motion_Update_Feedback();
			s_position_target_rpm = Controller_Position_Update(s_target_position,
			                                                   s_position_count,
			                                                   s_Kp_position,
			                                                   s_max_position_rpm,
			                                                   s_position_tolerance);
			s_trajectory_rpm = Controller_Trajectory_Update(s_position_target_rpm,
			                                                 s_max_acceleration_rpm_s);
			s_pwm_output = Controller_Speed_Update(s_trajectory_rpm, s_motor_rpm,
			                                       s_Kp, s_Ki);
			Motor_Set_Signed_PWM(s_pwm_output);

			if(s_position_count >= s_target_position - s_position_tolerance &&
			   s_position_count <= s_target_position + s_position_tolerance &&
			   s_trajectory_rpm == 0.0f &&
			   s_motor_rpm >= -5.0f &&
			   s_motor_rpm <= 5.0f)
			{
				s_motion_state = MOTION_STATE_READY;
			}
			break;

		case MOTION_STATE_RUN_SPEED:
			Motor_Set_Signed_PWM(0);
			break;

		case MOTION_STATE_HOMING:
			Motion_Update_Feedback();
			s_pwm_output = Controller_Speed_Update(s_home_rpm, s_motor_rpm, s_Kp, s_Ki);
			Motor_Set_Signed_PWM(s_pwm_output);

			if(HAL_GPIO_ReadPin(HOME_SIM_GPIO_Port, HOME_SIM_Pin) == GPIO_PIN_SET)
			{
				Motor_Set_Signed_PWM(0);
				__HAL_TIM_SET_COUNTER(&htim4, 0);
				s_last_count = 0;
				s_position_count = 0;
				Controller_Trajectory_Reset();
				Controller_Speed_Reset();
				s_motion_state = MOTION_STATE_READY;
			}
			else if((osKernelGetTickCount() - s_homing_start_tick) >= HOMING_TIMEOUT_MS)
			{
				Motor_Set_Signed_PWM(0);
				Controller_Trajectory_Reset();
				Controller_Speed_Reset();
				s_motion_state = MOTION_STATE_FAULT;
			}
			break;

		case MOTION_STATE_FAULT:
			Motor_Set_Signed_PWM(0);
			break;

		default:
			Motor_Set_Signed_PWM(0);
			s_motion_state = MOTION_STATE_FAULT;
			break;
	}

	Motion_Send_Debug_Log();
}
