#include "controller.h"

#define INTEGRAL_MIN  -1000
#define INTEGRAL_MAX  1000

static float s_speed_p_term = 0.0f;
static float s_speed_integral = 0.0f;
static float s_trajectory_rpm = 0.0f;

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

float Controller_Speed_Update(float target_rpm, float motor_rpm, float Kp, float Ki)
{
	float feedforward;
	float speed_error;
	float raw_output;
	float raw_before;
	uint8_t allow_integral;

	if(target_rpm == 0.0f)
	{
		Controller_Speed_Reset();
		return 0.0f;
	}

	feedforward = Motor_Calculate_Feedforward(target_rpm);
	speed_error = target_rpm - motor_rpm;
	s_speed_p_term = Kp * speed_error;
	raw_before = feedforward + s_speed_p_term + s_speed_integral;
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

void Controller_Speed_Reset(void)
{
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

float Controller_Trajectory_Update(float position_target_rpm, float max_acceleration_rpm_s)
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

void Controller_Trajectory_Reset(void)
{
	s_trajectory_rpm = 0.0f;
}
