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
	// 初始化后依次进入 IDLE 和 READY
	MOTION_STATE_INIT = 0,
	MOTION_STATE_IDLE,
	MOTION_STATE_READY,
	MOTION_STATE_RUN_SPEED,       // 预留的独立速度模式
	MOTION_STATE_RUN_POSITION,
	MOTION_STATE_HOMING,
	MOTION_STATE_FAULT            // 故障后只允许关闭 PWM
} motion_state_t;

static uint16_t s_current_count = 0;       // TIM4 当前 16 位编码器计数
static uint16_t s_last_count = 0;          // 上一周期编码器计数
static int16_t s_delta_count = 0;          // 转为有符号后的本周期位移
static float s_motor_rpm = 0.0f;           // 由位移换算得到的实际速度
static int32_t s_pwm_output = 0;           // 最终写给 DRV8833 的带符号 PWM

static motion_state_t s_motion_state = MOTION_STATE_INIT;
static uint8_t s_position_command_pending = 1;  // 当前用于自动触发一次相对位置测试
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
static uint32_t s_homing_start_tick = 0;   // Homing 超时计时起点

static float s_Kp = 5.0f;
static float s_Ki = 10.0f;
static char s_message[80];

volatile UBaseType_t motion_task_hwm_words = 0;

static void Motor_Set_Signed_PWM(int32_t pwm)
{
	// 正值使用 TIM3 CH1，负值使用 CH2，零值同时关闭两个通道
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
	// 利用 uint16_t 相减再转 int16_t，处理 TIM4 正反向 16 位回绕
	s_current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
	s_delta_count = (int16_t)(uint16_t)(s_current_count - s_last_count);
	s_last_count = s_current_count;
	s_position_count += s_delta_count;
	// 编码器速度按 20 ms 采样周期换算
	s_motor_rpm = (float)s_delta_count * 60.0f / (2800.0f * 0.02f);
}

static void Motion_Send_Debug_Log(void)
{
	uint16_t length;

	// 输出位置、实际速度、位置环速度目标和 Trajectory 速度
	length = snprintf(s_message, sizeof(s_message),
	                  "axis:%ld,%ld,%.1f,%.1f,%.1f\r\n",
	                  (long)s_target_position,
	                  (long)s_position_count,
	                  s_motor_rpm,
	                  s_position_target_rpm,
	                  s_trajectory_rpm);

	// 仅在 snprintf 未截断时发送，避免 UART 读取缓冲区外的数据
	if(length > 0 && length < (int)sizeof(s_message))
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)s_message,
                          (uint16_t)length, 100);
    }

	motion_task_hwm_words = uxTaskGetStackHighWaterMark(NULL);
}

void Motion_Init(void)
{
	// 先启动 Encoder 和双 PWM 通道，之后统一由 Motion_Update 控制输出
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
	// 默认执行一次 +2800 Count 相对位置验证
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
	// 当前轨迹验证先关闭积分项，隔离 Feedforward 与 P 跟踪效果
	s_Ki = 0.0f;

	// 清除跨周期控制状态，保证每次上电从零速开始
	Controller_Trajectory_Reset();
	Controller_Speed_Reset();
}

void Motion_Update(void)
{
	switch(s_motion_state)
	{
		case MOTION_STATE_INIT:
			// 上电第一周期强制关闭输出
			Motor_Set_Signed_PWM(0);
			s_motion_state = MOTION_STATE_IDLE;
			break;

		case MOTION_STATE_IDLE:
			// 保留一个安全空闲周期后再接受命令
			Motor_Set_Signed_PWM(0);
			s_motion_state = MOTION_STATE_READY;
			break;

		case MOTION_STATE_READY:
			// READY 不保留旧轨迹和积分，等待新的 Homing 或位置命令
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
			// Encoder -> Position P -> Trajectory -> Speed Controller -> PWM
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

			// 到位还要满足轨迹归零和实际低速，避免刚进入容差就断电
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
			// 当前尚未接入独立速度命令，进入该状态时保持安全停机
			s_pwm_output = 0;
			Motor_Set_Signed_PWM(0);
			break;

		case MOTION_STATE_HOMING:
			// Homing 仍使用独立低速闭环，不经过位置 Trajectory
			Motion_Update_Feedback();
			s_pwm_output = Controller_Speed_Update(s_home_rpm, s_motor_rpm, s_Kp, s_Ki);
			Motor_Set_Signed_PWM(s_pwm_output);

			if(HAL_GPIO_ReadPin(HOME_SIM_GPIO_Port, HOME_SIM_Pin) == GPIO_PIN_SET)
			{
				// HOME_SIM 触发后建立软件零点并清除控制器状态
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
				// 超时后关闭输出并锁定在 FAULT
				Motor_Set_Signed_PWM(0);
				Controller_Trajectory_Reset();
				Controller_Speed_Reset();
				s_motion_state = MOTION_STATE_FAULT;
			}
			break;

		case MOTION_STATE_FAULT:
			// FAULT 不自动恢复旧命令
			Motor_Set_Signed_PWM(0);
			break;

		default:
			// 未知状态按故障处理
			Motor_Set_Signed_PWM(0);
			s_motion_state = MOTION_STATE_FAULT;
			break;
	}

	Motion_Send_Debug_Log();
}
