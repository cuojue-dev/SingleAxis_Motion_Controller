#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>


// Motion_Update 内部使用的运行状态
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

// 对外公开的故障原因
typedef enum
{
	MOTION_FAULT_NONE = 0,
	MOTION_FAULT_NOT_HOMED,
	MOTION_FAULT_SOFT_LIMIT,
	MOTION_FAULT_HOME_TIMEOUT
} motion_fault_t;

// Request API 只提交命令，不直接执行运动
typedef enum
{
	MOTION_REQUEST_OK = 0,
	MOTION_REQUEST_BUSY,
	MOTION_REQUEST_INVALID_STATE
} motion_request_result_t;

// 供通信、调试或上层应用读取的只读状态快照
typedef struct
{
	motion_state_t state;
	motion_fault_t fault;

	uint8_t homed;
	int32_t position_count;
	float motor_rpm;
} motion_status_t;

void Motion_Init(void);
void Motion_Update(void);

// 以下 Request API 只能提交命令，状态迁移和 PWM 仍由 Motion_Update 统一执行
motion_request_result_t Motion_Request_Home(void);
motion_request_result_t Motion_Request_Position(int32_t target_position);
motion_request_result_t Motion_Request_Fault_Reset(void);

// status 为 NULL 时直接返回
void Motion_Get_Status(motion_status_t *status);

#endif /* MOTION_H */
