/*! \file mot_type.h
 *  \brief Define mot_type struct type.
*/

#if !defined(_MOT_TYPE_H_)
#define _MOT_TYPE_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/*! 指令类型
*/
typedef enum {
	KERMOT_ABORT = 1,	/* abort all motion */
	KERMOT_AXIS_ABORT,	/* abort one axis */ //FIXME-AJ: replace command name to EMCMOT_JOINT_ABORT
	KERMOT_ENABLE,		/* enable servos for active joints */
	KERMOT_DISABLE,		/* disable servos for active joints */
	KERMOT_ENABLE_AMPLIFIER,	/* enable amp outputs */
	KERMOT_DISABLE_AMPLIFIER,	/* disable amp outputs */
	KERMOT_ACTIVATE_JOINT,		/* make joint active */
	KERMOT_DEACTIVATE_JOINT,	/* make joint inactive */

	KERMOT_PAUSE,		/* pause motion */
	KERMOT_RESUME,		/* resume motion */
	KERMOT_FREE,		/* set mode to free (joint) motion */

	KERMOT_SET_PID,		/* control PID parameters */

	KERMOT_HOME,		/* home a joint or all joints */
	KERMOT_UNHOME,		/* unhome a joint or all joints*/
	KERMOT_JOG_CONT,	/* continuous jog */
	KERMOT_JOG_INCR,	/* incremental jog */
	KERMOT_JOG_ABS,		/* absolute jog */
	KERMOT_SET_LINE,	/* queue up a linear move */

	KERMOT_SET_POSITION_LIMITS,	/* set the joint position +/- limits */
	KERMOT_SET_VEL,		/* set the velocity for subsequent moves */
	KERMOT_SET_VEL_LIMIT,	/* set the max vel for all moves (tooltip) */
	KERMOT_SET_JOINT_VEL_LIMIT,	/* set the max joint vel */
	KERMOT_SET_JOINT_ACC_LIMIT,	/* set the max joint accel */
	KERMOT_SET_ACC,		/* set the max accel for moves (tooltip) */
	KERMOT_SET_DOUT,        /* sets or unsets a DIO, this can be imediate or synched with motion */
	KERMOT_SET_AOUT,	/* sets or unsets a AIO, this can be imediate or synched with motion */

	KERMOT_SPINDLE_BRAKE_ENGAGE,	/* engage the spindle brake */
	KERMOT_SPINDLE_BRAKE_RELEASE,	/* release the spindle brake */

	// IO 部分
	KERMOT_GET_DI,
	KERMOT_SET_DO,
	KERMOT_SET_DO_BIT,
	KERMOT_GET_DO

} cmd_code_t;

/*! 描述指令执行的状态
*/
typedef enum {
	KERMOT_COMMAND_OK = 0,	/* cmd honored */
	KERMOT_COMMAND_UNKNOWN_COMMAND,	/* cmd not understood */
	KERMOT_COMMAND_INVALID_COMMAND,	/* cmd can't be handled now */
	KERMOT_COMMAND_INVALID_PARAMS,	/* bad cmd params */
	KERMOT_COMMAND_BAD_EXEC	/* error trying to initiate */
} cmd_status_t;

//* motion controller states */
typedef enum {
	TASK_STATE_DISABLED = 0,
	TASK_STATE_ERROR,
	TASK_STATE_RUNNING,
	TASK_STATE_IDLE
} task_state_t;


#endif // !defined(_MOT_TYPE_H_)
