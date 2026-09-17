#include "controller.h"

#define INTEGRAL_MIN  -1000  // 积分项下限
#define INTEGRAL_MAX  1000   // 积分项上限

static float s_speed_p_term = 0.0f;       // 当前速度环 P 项
static float s_speed_integral = 0.0f;     // 跨周期保存的速度环 I 项
static float s_trajectory_rpm = 0.0f;     // 跨周期保存的平滑速度指令

// 基于当前机械装配实测点的分段 Feedforward
static float Motor_Calculate_Feedforward(float target_rpm)
{
	float pwm_magnitude;
	float target_rpm_abs;

	// 目标为零时不施加维持转动的基础 PWM
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

	// 低速段从零平滑插值到最小起步工作点
	// 标定点为 40/47/54.5/55.5/64.5 RPM
	// 每一段都在相邻实测工作点之间做线性插值
	if(target_rpm_abs <= 40.0f)
	{
		pwm_magnitude = 2150.0f * target_rpm_abs / 40.0f;
	}
	else if(target_rpm_abs <= 47.0f)
	{
		pwm_magnitude = 2150.0f +
				(2400.0f - 2150.0f) * (target_rpm_abs - 40.0f) / 7.0f;
	}
	else if(target_rpm_abs <= 54.5f)
	{
		pwm_magnitude = 2400.0f +
				(2600.0f - 2400.0f) * (target_rpm_abs - 47.0f) / 7.5f;
	}
	else if(target_rpm_abs <= 55.5f)
	{
		pwm_magnitude = 2600.0f +
				(2642.0f - 2600.0f) * (target_rpm_abs - 54.5f);
	}
	else if(target_rpm_abs <= 64.5f)
	{
		pwm_magnitude = 2642.0f +
				(2800.0f - 2642.0f) * (target_rpm_abs - 55.5f) / 9.0f;
	}
	else
	{
		pwm_magnitude = 2800.0f + 12.7f * (target_rpm_abs - 64.5f);
	}

	// Feedforward 也必须落在 TIM3 ARR 的可执行范围内
	if(pwm_magnitude > 3599.0f)
	{
		pwm_magnitude = 3599.0f;
	}
	else if(pwm_magnitude < 0.0f)
	{
		pwm_magnitude = 0.0f;
	}

	// 标定只计算幅值，最后按目标方向恢复符号
	if(target_rpm < 0)
	{
		return -pwm_magnitude;
	}
	else
	{
		return pwm_magnitude;
	}
}

float Controller_Speed_Update(float target_rpm, float motor_rpm, float Kp, float Ki)
{
	float feedforward;
	float speed_error;
	float raw_output;
	float raw_before;
	uint8_t allow_integral;

	// 速度目标归零时清除控制器记忆，避免旧积分重新推动电机
	if(target_rpm == 0.0f)
	{
		Controller_Speed_Reset();
		return 0.0f;
	}

	// 前馈提供当前目标速度附近的基础 PWM
	feedforward = Motor_Calculate_Feedforward(target_rpm);

	// 正误差表示实际速度偏低，需要增加正向控制量
	speed_error = target_rpm - motor_rpm;
	s_speed_p_term = Kp * speed_error;

	// 先用旧积分估计输出，用于决定本周期是否允许继续积分
	raw_before = feedforward + s_speed_p_term + s_speed_integral;
	allow_integral = 0;

	// 未饱和时正常积分
	// 正饱和时只允许负误差减小积分
	// 负饱和时只允许正误差减小积分
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
		// 控制周期为 20 ms
		s_speed_integral += Ki * speed_error * 0.02f;
	}

	// 对内部积分状态限幅，防止长期积累形成 windup
	if(s_speed_integral < INTEGRAL_MIN)
	{
		s_speed_integral = INTEGRAL_MIN;
	}
	else if(s_speed_integral > INTEGRAL_MAX)
	{
		s_speed_integral = INTEGRAL_MAX;
	}

	// 用更新后的积分重新组合本周期 PWM 指令
	raw_output = feedforward + s_speed_p_term + s_speed_integral;

	// 最终 PWM 输出限幅，保护 TIM3 比较寄存器范围
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

void Controller_Speed_Reset(void)
{
	// 停机、到位或 Homing 完成后统一清除速度控制器状态
	s_speed_p_term = 0.0f;
	s_speed_integral = 0.0f;
}

float Controller_Position_Update(int32_t target_position,
                                 int32_t current_position,
                                 float Kp_position,
                                 float max_position_rpm,
                                 int32_t position_tolerance)
{
	int32_t position_error;
	float target_rpm;

	// 位置误差的正负号同时表示需要运动的方向
	position_error = target_position - current_position;

	// 进入位置容差后，位置环不再要求继续运动
	if(position_error >= -position_tolerance &&
	   position_error <= position_tolerance)
	{
		return 0.0f;
	}

	// Position P 将剩余 Count 转换为期望速度
	target_rpm = Kp_position * position_error;

	// 限制位置环直接给速度环的最大速度
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

float Controller_Trajectory_Update(float position_target_rpm, float max_acceleration_rpm_s)
{
	float max_delta_rpm;

	// 每个 20 ms 周期允许的最大速度变化
	max_delta_rpm = max_acceleration_rpm_s * 0.02f;

	// 目标变化过快时只走一个最大步长
	// 目标已在本周期可到达范围内时直接对齐
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

void Controller_Trajectory_Reset(void)
{
	// 每次新运动开始前从零速重新规划
	s_trajectory_rpm = 0.0f;
}
