
#ifndef __RBT_H_
#define __RBT_H_

#include "mrl_type.h"

// types for RBT_KERNEL mode
enum RBT_KERNEL_MODE_ENUM {
    RBT_KERNEL_MODE_MANUAL = 1,
	RBT_KERNEL_MODE_AUTO = 2,
	RBT_KERNEL_MODE_MDI = 3
};

// types for RBT_KERNEL state
enum RBT_KERNEL_STATE_ENUM {
    RBT_KERNEL_STATE_ESTOP = 1,
	RBT_KERNEL_STATE_READY = 2
};

// types for motion control
enum RBT_TRAJ_MODE_ENUM {
    RBT_TRAJ_MODE_FREE = 1,		// independent-axis motion,
	RBT_TRAJ_MODE_COORD = 2,	// coordinated-axis motion,
	RBT_TRAJ_MODE_TELEOP = 3	// velocity based world coordinates motion,
};


// 操作面板消息
#define RBT_OPERATOR_ERROR_TYPE                      ((MRLTYPE) 11)
#define RBT_OPERATOR_TEXT_TYPE                       ((MRLTYPE) 12)
#define RBT_OPERATOR_DISPLAY_TYPE                    ((MRLTYPE) 13)

#define RBT_AXIS_SET_MIN_POSITION_LIMIT_TYPE         ((MRLTYPE) 107)
#define RBT_AXIS_SET_MAX_POSITION_LIMIT_TYPE         ((MRLTYPE) 108)

// 轨迹
#define RBT_TRAJ_LINEAR_MOVE_TYPE                    ((MRLTYPE) 220)

// MRL for RBT_KERNEL
#define RBT_KERNEL_SET_MODE_TYPE                     ((MRLTYPE) 504)
#define RBT_KERNEL_SET_STATE_TYPE                    ((MRLTYPE) 505)

// RBT_IO aggregate class type declaration
#define RBT_IO_INIT_TYPE                             ((MRLTYPE) 1601)
#define RBT_IO_HALT_TYPE                             ((MRLTYPE) 1602)
#define RBT_IO_ABORT_TYPE                            ((MRLTYPE) 1603)
#define RBT_IO_STAT_TYPE                             ((MRLTYPE) 1699)

// RBT_USER
#define RBT_USER_IMU_STAT_TYPE												((MRLTYPE) 1150)

// digital IO
#define RBT_IO_GET_DI_TYPE							 ((MRLTYPE) 1000)
#define RBT_IO_SET_DO_BIT_TYPE					     ((MRLTYPE) 1001)
#define RBT_IO_SET_DO_TYPE						     ((MRLTYPE) 1002)
#define RBT_IO_GET_DO_TYPE						     ((MRLTYPE) 1003)

// analog IO
#define RBT_IO_GET_AI_TYPE                           ((MRLTYPE) 1100)


// single axis
#define RBT_AXIS_ENABLE_TYPE                         ((MRLTYPE) 121)
#define RBT_AXIS_DISABLE_TYPE                        ((MRLTYPE) 122)
#define RBT_AXIS_HOME_TYPE                           ((MRLTYPE) 123)
#define RBT_AXIS_UNHOME_TYPE                         ((MRLTYPE) 135)
#define RBT_AXIS_JOG_TYPE                            ((MRLTYPE) 124)
#define RBT_AXIS_INCR_JOG_TYPE                       ((MRLTYPE) 125)
#define RBT_AXIS_ABS_JOG_TYPE                        ((MRLTYPE) 126)
#define RBT_AXIS_SET_PID_TYPE			     ((MRLTYPE) 127)

// milti axes
#define RBT_MULTI_AXIS_JOG_TYPE                      ((MRLTYPE) 130)

// message of task
#define RBT_TASK_ENABLE_TASK_TYPE					 ((MRLTYPE) 500)
#define RBT_TASK_DISABLE_TASK_TYPE					 ((MRLTYPE) 501)

#define RBT_TASK_START_FORCE_THREAD_TYPE            ((MRLTYPE) 502)
#define RBT_TASK_TERMINATE_FORCE_THREAD_TYPE             ((MRLTYPE) 503)


// [append] append message step 1


#endif //end of __RBT_H_
