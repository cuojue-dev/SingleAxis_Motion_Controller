#include "motion.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "tim.h"
#include "usart.h"



// 运行期间保持不变的运动控制参数
typedef struct
{
	float speed_kp;                 // 速度环比例增益
	float speed_ki;                 // 速度环积分增益
	float position_kp;              // 位置环比例增益
	float max_position_rpm;         // 位置模式最大速度
	float max_acceleration_rpm_s;   // Trajectory 最大加速度
	float home_rpm;                 // Homing 搜索速度

	int32_t position_tolerance;     // 到位允许误差
	int32_t soft_limit_min;         // 当前软件测试下限
	int32_t soft_limit_max;         // 当前软件测试上限
	uint32_t homing_timeout_ms;     // Homing 最大持续时间
} motion_config_t;

// 运动状态机每个周期都会读写的运行状态
typedef struct
{
	motion_state_t state;
	motion_fault_t fault;

	uint8_t homed;                    // 机械坐标是否有效
	uint8_t home_command_pending;      // 等待 Motion_Update 消费的 Home 命令
	uint8_t position_command_pending;  // 等待 Motion_Update 消费的位置命令
	uint8_t fault_reset_pending;       // 等待 Motion_Update 消费的 Fault Reset 命令

	uint16_t current_count;
	uint16_t last_count;
	int16_t delta_count;

	int32_t position_count;
	int32_t target_position;
	int32_t pwm_output;

	float motor_rpm;
	float position_target_rpm;
	float trajectory_rpm;

	uint32_t homing_start_tick;
} motion_context_t;

static const motion_config_t s_config = {
	.speed_kp = 5.0f,
	.speed_ki = 0.0f,
	.position_kp = 0.05f,
	.max_position_rpm = 60.0f,
	.max_acceleration_rpm_s = 100.0f,
	.home_rpm = -40.0f,
	.position_tolerance = 100,
	.soft_limit_min = -3500,
	.soft_limit_max = 3500,
	.homing_timeout_ms = 10000,
};

static motion_context_t s_motion;
static char s_message[80];

volatile UBaseType_t motion_task_hwm_words = 0;

static void Motor_Set_Signed_PWM(int32_t pwm);
static void Motion_Stop_And_Reset_Control(void);
static uint8_t Motion_Is_Target_Within_Soft_Limit(int32_t target_position);
static void Motion_Enter_Fault(motion_fault_t fault_code);
static void Motion_Update_Feedback(void);
static void Motion_Send_Debug_Log(void);

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

static void Motion_Stop_And_Reset_Control(void)
{
	s_motion.pwm_output = 0;
	s_motion.position_target_rpm = 0.0f;
	s_motion.trajectory_rpm = 0.0f;

	Motor_Set_Signed_PWM(0);
	Controller_Trajectory_Reset();
	Controller_Speed_Reset();
}

static uint8_t Motion_Is_Target_Within_Soft_Limit(int32_t target_position)
{
	if(target_position < s_config.soft_limit_min)
	{
		return 0;
	}

	if(target_position > s_config.soft_limit_max)
	{
		return 0;
	}

	return 1;
}

static void Motion_Enter_Fault(motion_fault_t fault_code)
{
	s_motion.fault = fault_code;
	s_motion.position_command_pending = 0;
	s_motion.home_command_pending = 0;

	Motion_Stop_And_Reset_Control();
	s_motion.state = MOTION_STATE_FAULT;

	switch(fault_code)
	{
		case MOTION_FAULT_NOT_HOMED:
			HAL_UART_Transmit(&huart1,
			                  (uint8_t *)"FAULT_NOT_HOMED\r\n",
			                  (uint16_t)(sizeof("FAULT_NOT_HOMED\r\n") - 1U),
			                  100);
			break;

		case MOTION_FAULT_SOFT_LIMIT:
			HAL_UART_Transmit(&huart1,
			                  (uint8_t *)"FAULT_SOFT_LIMIT\r\n",
			                  (uint16_t)(sizeof("FAULT_SOFT_LIMIT\r\n") - 1U),
			                  100);
			break;

		case MOTION_FAULT_HOME_TIMEOUT:
			HAL_UART_Transmit(&huart1,
			                  (uint8_t *)"FAULT_HOME_TIMEOUT\r\n",
			                  (uint16_t)(sizeof("FAULT_HOME_TIMEOUT\r\n") - 1U),
			                  100);
			break;

		default:
			break;
	}
}

static void Motion_Update_Feedback(void)
{
	// uint16_t 相减后再转 int16_t，保留 TIM4 正反向回绕后的有符号增量
	s_motion.current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
	s_motion.delta_count = (int16_t)(uint16_t)(s_motion.current_count -
												  s_motion.last_count);
	s_motion.last_count = s_motion.current_count;
	s_motion.position_count += s_motion.delta_count;

	// 编码器速度按 20 ms 采样周期换算
	s_motion.motor_rpm = (float)s_motion.delta_count * 60.0f /
						 (2800.0f * 0.02f);
}

static void Motion_Send_Debug_Log(void)
{
	int length;

	// 依次输出目标位置、实际位置、实际速度、位置环目标和 Trajectory 速度
	length = snprintf(s_message, sizeof(s_message),
					  "axis:%ld,%ld,%.1f,%.1f,%.1f\r\n",
					  (long)s_motion.target_position,
					  (long)s_motion.position_count,
					  s_motion.motor_rpm,
					  s_motion.position_target_rpm,
					  s_motion.trajectory_rpm);

	if(length > 0 && length < (int)sizeof(s_message))
	{
		HAL_UART_Transmit(&huart1,
		                  (uint8_t *)s_message,
		                  (uint16_t)length,
		                  100);
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

	// 使用一次结构体赋值建立安全上电状态
	s_motion = (motion_context_t){
		.state = MOTION_STATE_INIT,
		.fault = MOTION_FAULT_NONE,
		.homed = 0,
		.home_command_pending = 0,
		.position_command_pending = 0,
		.target_position = 2800,
		.fault_reset_pending = 0,
	};

	Motion_Stop_And_Reset_Control();
}

void Motion_Update(void)
{
	switch(s_motion.state)
	{
		case MOTION_STATE_INIT:
			Motor_Set_Signed_PWM(0);
			s_motion.state = MOTION_STATE_IDLE;
			break;

		case MOTION_STATE_IDLE:
			Motor_Set_Signed_PWM(0);
			s_motion.state = MOTION_STATE_READY;
			break;

		case MOTION_STATE_READY:
			Motion_Stop_And_Reset_Control();

			if(s_motion.home_command_pending)
			{
				s_motion.home_command_pending = 0;
				s_motion.homing_start_tick = osKernelGetTickCount();
				s_motion.state = MOTION_STATE_HOMING;
			}
			else if(s_motion.position_command_pending)
			{
				s_motion.position_command_pending = 0;

				if(!s_motion.homed)
				{
					Motion_Enter_Fault(MOTION_FAULT_NOT_HOMED);
				}
				else if(Motion_Is_Target_Within_Soft_Limit(
						s_motion.target_position))
				{
					s_motion.state = MOTION_STATE_RUN_POSITION;
				}
				else
				{
					Motion_Enter_Fault(MOTION_FAULT_SOFT_LIMIT);
				}
			}
			break;

		case MOTION_STATE_RUN_POSITION:
			// Encoder -> Position P -> Trajectory -> Speed Controller -> PWM
			Motion_Update_Feedback();

			if(!s_motion.homed)
			{
				Motion_Enter_Fault(MOTION_FAULT_NOT_HOMED);
				break;
			}

			if(!Motion_Is_Target_Within_Soft_Limit(s_motion.position_count))
			{
				Motion_Enter_Fault(MOTION_FAULT_SOFT_LIMIT);
				break;
			}

			s_motion.position_target_rpm = Controller_Position_Update(
					s_motion.target_position,
					s_motion.position_count,
					s_config.position_kp,
					s_config.max_position_rpm,
					s_config.position_tolerance);

			s_motion.trajectory_rpm = Controller_Trajectory_Update(
					s_motion.position_target_rpm,
					s_config.max_acceleration_rpm_s);

			s_motion.pwm_output = Controller_Speed_Update(
					s_motion.trajectory_rpm,
					s_motion.motor_rpm,
					s_config.speed_kp,
					s_config.speed_ki);
			Motor_Set_Signed_PWM(s_motion.pwm_output);

			// 到位还要满足轨迹归零和实际低速，避免刚进入容差就断电
			if(s_motion.position_count >= s_motion.target_position -
			   s_config.position_tolerance &&
			   s_motion.position_count <= s_motion.target_position +
			   s_config.position_tolerance &&
			   s_motion.trajectory_rpm == 0.0f &&
			   s_motion.motor_rpm >= -5.0f &&
			   s_motion.motor_rpm <= 5.0f)
			{
				s_motion.state = MOTION_STATE_READY;
			}
			break;

		case MOTION_STATE_RUN_SPEED:
			Motion_Stop_And_Reset_Control();
			break;

		case MOTION_STATE_HOMING:
			// Homing 使用独立低速闭环，不经过位置 Trajectory
			Motion_Update_Feedback();
			s_motion.pwm_output = Controller_Speed_Update(
					s_config.home_rpm,
					s_motion.motor_rpm,
					s_config.speed_kp,
					s_config.speed_ki);
			Motor_Set_Signed_PWM(s_motion.pwm_output);

			if(HAL_GPIO_ReadPin(HOME_SIM_GPIO_Port, HOME_SIM_Pin) == GPIO_PIN_SET)
			{
				__HAL_TIM_SET_COUNTER(&htim4, 0);
				s_motion.last_count = 0;
				s_motion.position_count = 0;

				Motion_Stop_And_Reset_Control();
				s_motion.homed = 1;
				s_motion.state = MOTION_STATE_READY;
			}
			else if((osKernelGetTickCount() - s_motion.homing_start_tick) >=
					s_config.homing_timeout_ms)
			{
				s_motion.homed = 0;
				Motion_Enter_Fault(MOTION_FAULT_HOME_TIMEOUT);
			}
			break;

		case MOTION_STATE_FAULT:
			Motion_Stop_And_Reset_Control();

		    // PA0 只是 Fault Reset 命令的一个临时来源
			if(HAL_GPIO_ReadPin(HOME_SIM_GPIO_Port, HOME_SIM_Pin) == GPIO_PIN_SET)
			{
				s_motion.fault_reset_pending = 1;
			}

			// 无论命令来自 PA0 还是公开 API，都走同一条 Reset 路径
			if(s_motion.fault_reset_pending)
			{
				s_motion.fault_reset_pending = 0;
				s_motion.fault = MOTION_FAULT_NONE;
				s_motion.position_command_pending = 0;
				s_motion.home_command_pending = 0;
				s_motion.state = MOTION_STATE_IDLE;

				HAL_UART_Transmit(&huart1,
						(uint8_t *)"FAULT_RESET_OK\r\n",
						(uint16_t)(sizeof("FAULT_RESET_OK\r\n") - 1U),
						100);
			}
			break;

		default:
			Motion_Stop_And_Reset_Control();
			s_motion.state = MOTION_STATE_FAULT;
			break;
	}

	Motion_Send_Debug_Log();
}

motion_request_result_t Motion_Request_Home(void)
{
	// 只允许 READY 接收新命令，避免外部代码打断正在执行的运动
	if(s_motion.state != MOTION_STATE_READY)
	{
		return MOTION_REQUEST_INVALID_STATE;
	}

	if(s_motion.home_command_pending ||
	   s_motion.position_command_pending)
	{
		return MOTION_REQUEST_BUSY;
	}

	// 实际进入 HOMING 由下一次 Motion_Update 完成
	s_motion.home_command_pending = 1;

	return MOTION_REQUEST_OK;
}

motion_request_result_t Motion_Request_Position(int32_t target_position)
{
	// Request 层只提交目标，Homed 和 Soft Limit 由状态机统一校验
	if(s_motion.state != MOTION_STATE_READY)
	{
		return MOTION_REQUEST_INVALID_STATE;
	}

	if(s_motion.home_command_pending ||
	   s_motion.position_command_pending)
	{
		return MOTION_REQUEST_BUSY;
	}

	s_motion.target_position = target_position;
	s_motion.position_command_pending = 1;

	return MOTION_REQUEST_OK;
}

motion_request_result_t Motion_Request_Fault_Reset(void)
{
	// Reset 只能清除已经进入的 Fault，不能当作普通停止命令
	if(s_motion.state != MOTION_STATE_FAULT)
	{
		return MOTION_REQUEST_INVALID_STATE;
	}

	s_motion.fault_reset_pending = 1;

	return MOTION_REQUEST_OK;
}

void Motion_Get_Status(motion_status_t *status)
{
	if(status == NULL)
	{
		return;
	}

	// 复制必要字段，外部模块无法直接修改内部 Context
	status->state = s_motion.state;
	status->fault = s_motion.fault;
	status->homed = s_motion.homed;
	status->position_count = s_motion.position_count;
	status->motor_rpm = s_motion.motor_rpm;
}
