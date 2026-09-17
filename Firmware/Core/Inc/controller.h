#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>

float Controller_Speed_Update(float target_rpm, float motor_rpm, float Kp, float Ki);
void Controller_Speed_Reset(void);

float Controller_Position_Update(int32_t target_position,
                                 int32_t current_position,
                                 float Kp_position,
                                 float max_position_rpm,
                                 int32_t position_tolerance);

float Controller_Trajectory_Update(float position_target_rpm, float max_acceleration_rpm_s);
void Controller_Trajectory_Reset(void);

#endif /* CONTROLLER_H */
