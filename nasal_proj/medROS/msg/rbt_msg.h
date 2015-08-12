
#ifndef __RBT_MSG_H_
#define __RBT_MSG_H_

#include "cmd_msg.h"
#include "stat_msg.h"
#include "pos.h"
#include "mot_type.h"
#include "rbt.h"
#include "../config.h"

//
enum TASK_TYPE_ENUM{
	TASK_TYPE_NULL = 0x00,
	TASK_TYPE_MULTI_AXIS = 0x01,
	TASK_TYPE_IO = 0x02,
};


// RBT_KERNEL status base class
class RBT_KERNEL_STAT_MSG:public RBT_STAT_MSG {
public:
    RBT_KERNEL_STAT_MSG(MRLTYPE t, size_t s):RBT_STAT_MSG(t, s) {
		heartbeat = 0;
    };

    unsigned long int heartbeat;
};


// RBT_TRAJ status base class
class RBT_TRAJ_STAT_MSG:public RBT_STAT_MSG {
public:
    RBT_TRAJ_STAT_MSG(MRLTYPE t, size_t s):RBT_STAT_MSG(t, s) {
    };
};

class RBT_TRAJ_STAT:public RBT_TRAJ_STAT_MSG {
public:
    RBT_TRAJ_STAT();

    double linearUnits;		// units per mm
    double angularUnits;	// units per degree
    int axes;				// maximum axis number
    int axis_mask;			// mask of axes actually present
    enum RBT_TRAJ_MODE_ENUM mode;	// RBT_TRAJ_MODE_FREE,
    // RBT_TRAJ_MODE_COORD
    bool enabled;			// non-zero means enabled

    bool inpos;				// non-zero means in position
    int queue;				// number of pending motions, counting
    // current
    int activeQueue;		// number of motions blending
    bool queueFull;			// non-zero means can't accept another motion
    int id;					// id of the currently executing motion
    bool paused;			// non-zero means motion paused

    Position position;		// current commanded position
    Position actualPosition;	// current actual position, from forward kins
    double velocity;		// system velocity, for subsequent motions
    double acceleration;	// system acceleration, for subsequent
							// motions
    double maxVelocity;		// max system velocity
    double maxAcceleration;	// system acceleration

    int kinematics_type;	// identity=1,serial=2,parallel=3,custom=4
    int motion_type;
    double distance_to_go;	// in current move
    Position dtg;
    double current_vel;		// in current move
};


// AXIS status base class
class RBT_AXIS_STAT_MSG:public RBT_STAT_MSG {
public:
    RBT_AXIS_STAT_MSG(MRLTYPE t, size_t s):RBT_STAT_MSG(t, s) {
    };

    unsigned int mark;
};

class RBT_AXIS_STAT:public RBT_AXIS_STAT_MSG {
public:
    RBT_AXIS_STAT();

    // configuration parameters
    unsigned char axisType;	// RBT_AXIS_LINEAR, RBT_AXIS_ANGULAR
    double units;		// units per mm, deg for linear, angular
    double backlash;
    double minPositionLimit;
    double maxPositionLimit;
    double maxFerror;
    double minFerror;

    // dynamic status
    double ferrorCurrent;	// current following error
    double ferrorHighMark;	// magnitude of max following error
    /*! \todo FIXME - is this really position, or the DAC output? */
    double output;		// commanded output position
    double input;		// current input position
    double velocity;		// current velocity
    unsigned char inpos;	// non-zero means in position
    unsigned char fault;	// non-zero means axis amp fault
    unsigned char enabled;	// non-zero means enabled
    unsigned char minSoftLimit;	// non-zero means min soft limit exceeded
    unsigned char maxSoftLimit;	// non-zero means max soft limit exceeded
    unsigned char minHardLimit;	// non-zero means min hard limit exceeded
    unsigned char maxHardLimit;	// non-zero means max hard limit exceeded
};


class RBT_MOTION_STAT:public RBT_STAT_MSG {
public:
    RBT_MOTION_STAT();
    // aggregate of motion-related status classes
    RBT_TRAJ_STAT traj; // 整理
    RBT_AXIS_STAT axis[RBT_AXIS_MAX]; // 整理
};

// RBT_TASK status base class
class RBT_TASK_STAT_MSG:public RBT_STAT_MSG {
public:
    RBT_TASK_STAT_MSG(MRLTYPE t, size_t s):RBT_STAT_MSG(t, s) {
    };
};

class RBT_TASK_STAT:public RBT_TASK_STAT_MSG {
public:
    RBT_TASK_STAT();
// id
	unsigned int		axisMark;
	TASK_TYPE_ENUM		taskType;

	// command
	cmd_code_t			command;		///<	指令码
	int					commandNum;		///<	加1表示有新指令
	// state
	cmd_code_t			commandEcho;	///<	输入指令码的响应
	int					commandNumEcho;	///<	输入指令号的响应
	cmd_status_t		commandStatus;	///<	最后执行指令的状态
	task_state_t		task_state;		///<	任务运行状态

	// return code
	int					rtn_code;			///<	返回码
	cmd_code_t			rtn_commandEcho;	///<	输入指令码的响应
	int					rtn_commandNumEcho;	///<	输入指令号的响应
};


class RBT_KERNEL_STAT:public RBT_KERNEL_STAT_MSG {
public:
    RBT_KERNEL_STAT();

	// 模式是为以后留出来的扩展的接口，目前暂时不用。
    enum RBT_KERNEL_MODE_ENUM  mode;		// RBT_KERNEL_MODE_MANUAL, etc.
    enum RBT_KERNEL_STATE_ENUM state;	// RBT_KERNEL_STATE_ESTOP, etc.

    //	RBT_TASK_STAT	taskState[MAX_TASK_NUMBER];
};

// RBT_IO status base class
class RBT_IO_STAT_MSG:public RBT_STAT_MSG {
public:
    RBT_IO_STAT_MSG(MRLTYPE t, size_t s):RBT_STAT_MSG(t, s) {
		heartbeat = 0;
    };
    unsigned long int heartbeat;
};

// RBT_USER IMU data class
class RBT_USER_IMU_STAT_MSG:public RBT_STAT_MSG{
public:
		RBT_USER_IMU_STAT_MSG(MRLTYPE t, size_t s):RBT_STAT_MSG(t, s){
		};
    unsigned long int heartbeat;
};

class RBT_USER_IMU_STAT:public RBT_USER_IMU_STAT_MSG {
public:
    RBT_USER_IMU_STAT():RBT_USER_IMU_STAT_MSG(RBT_USER_IMU_STAT_TYPE, sizeof(RBT_USER_IMU_STAT)) {
    };
    float current_YPR[3];
    float reference_YPR[3];
    float delta_YPR[3];
int state;
int act_jnt;
};

// RBT_AUX status base class
class RBT_AUX_STAT_MSG:public RBT_STAT_MSG {
public:
    RBT_AUX_STAT_MSG(MRLTYPE t, size_t s):RBT_STAT_MSG(t, s) {
    };
};

class RBT_AUX_STAT:public RBT_AUX_STAT_MSG {
public:
    RBT_AUX_STAT();
	
    int estop;			// non-zero means estopped
};

class RBT_IO_STAT:public RBT_IO_STAT_MSG {
public:
    RBT_IO_STAT():RBT_IO_STAT_MSG(RBT_IO_STAT_TYPE, sizeof(RBT_IO_STAT)) {
    };
	
    // aggregate of IO-related status classes
    RBT_AUX_STAT aux;
	
    unsigned int synch_di[RBT_MAX_DIO];  // motion inputs queried by interp
    unsigned int synch_do[RBT_MAX_DIO];  // motion outputs queried by interp
};


class RBT_STAT:public RBT_STAT_MSG {
public:
    RBT_STAT();
	
    // the top-level RBT_KERNEL status class
    RBT_KERNEL_STAT kernel;
	
    // subordinate status classes
    RBT_MOTION_STAT motion;
    RBT_IO_STAT io;
		RBT_USER_IMU_STAT imu;
};

// declarations for RBT_TRAJ classes

// RBT_TRAJ command base class
class RBT_TRAJ_CMD_MSG:public RBT_CMD_MSG {
public:
    RBT_TRAJ_CMD_MSG(MRLTYPE t, size_t s):RBT_CMD_MSG(t, s) {
    };
	
	unsigned int mark; // 联动轴标号  lwy add
};

class RBT_TRAJ_LINEAR_MOVE:public RBT_TRAJ_CMD_MSG {
public:
    RBT_TRAJ_LINEAR_MOVE():RBT_TRAJ_CMD_MSG(RBT_TRAJ_LINEAR_MOVE_TYPE,
		sizeof(RBT_TRAJ_LINEAR_MOVE)) {
    };

    int type;
    Position end;		// end point
    double vel, ini_maxvel, acc;
    int feed_mode;
};

// MRL for RBT_KERNEL

// declarations for RBT_KERNEL classes

// RBT_KERNEL command base class
class RBT_KERNEL_CMD_MSG:public RBT_CMD_MSG {
public:
    RBT_KERNEL_CMD_MSG(MRLTYPE t, size_t s):RBT_CMD_MSG(t, s) {
    };
};

class RBT_KERNEL_SET_MODE:public RBT_KERNEL_CMD_MSG {
public:
    RBT_KERNEL_SET_MODE():RBT_KERNEL_CMD_MSG(RBT_KERNEL_SET_MODE_TYPE,
		sizeof(RBT_KERNEL_SET_MODE)) {
    };
    enum RBT_KERNEL_MODE_ENUM mode;
};

class RBT_KERNEL_SET_STATE:public RBT_KERNEL_CMD_MSG {
public:
    RBT_KERNEL_SET_STATE():RBT_KERNEL_CMD_MSG(RBT_KERNEL_SET_STATE_TYPE,
		sizeof(RBT_KERNEL_SET_STATE)) {
    };
	
    enum RBT_KERNEL_STATE_ENUM state;
};


// declarations for RBT_AXIS classes

/*
* AXIS command base class.
* This is the base class for all commands that operate on a single axis.
* The axis parameter specifies which axis the command affects.
* These commands are sent to the RBTCommand buffer to be read by the
* TASK program that will then pass along corresponding messages to the
* motion system.
*/
class RBT_AXIS_CMD_MSG:public RBT_CMD_MSG {
public:
    RBT_AXIS_CMD_MSG(MRLTYPE t, size_t s):RBT_CMD_MSG(t, s) {
    };
    // 0x01 = X, 0x02 = Y, 0x04 = Z, etc.
    unsigned int mark;
};

class RBT_AXIS_HOME:public RBT_AXIS_CMD_MSG {
public:
    RBT_AXIS_HOME():RBT_AXIS_CMD_MSG(RBT_AXIS_HOME_TYPE,
		sizeof(RBT_AXIS_HOME)) {
    };
};

class RBT_AXIS_UNHOME:public RBT_AXIS_CMD_MSG {
public:
    RBT_AXIS_UNHOME():RBT_AXIS_CMD_MSG(RBT_AXIS_UNHOME_TYPE,
		sizeof(RBT_AXIS_UNHOME)) {
    };
};

class RBT_AXIS_JOG:public RBT_AXIS_CMD_MSG {
public:
    RBT_AXIS_JOG():RBT_AXIS_CMD_MSG(RBT_AXIS_JOG_TYPE,
				    sizeof(RBT_AXIS_JOG)) {
    };
    double vel;
};


class RBT_AXIS_INCR_JOG:public RBT_AXIS_CMD_MSG {
public:
    RBT_AXIS_INCR_JOG():RBT_AXIS_CMD_MSG(RBT_AXIS_INCR_JOG_TYPE,
		sizeof(RBT_AXIS_INCR_JOG)) {
    };
    double incr;
    double vel;
};


class RBT_AXIS_ABS_JOG:public RBT_AXIS_CMD_MSG {
public:
    RBT_AXIS_ABS_JOG():RBT_AXIS_CMD_MSG(RBT_AXIS_ABS_JOG_TYPE,
		sizeof(RBT_AXIS_ABS_JOG)) {
    };
    double pos;
    double vel;
};


class RBT_AXIS_SET_PID:public RBT_AXIS_CMD_MSG {
public:
    RBT_AXIS_SET_PID():RBT_AXIS_CMD_MSG(RBT_AXIS_SET_PID_TYPE,
		sizeof(RBT_AXIS_SET_PID)) {
    };
    double kp, ki, kd;
};

class RBT_AXIS_ENABLE:public RBT_AXIS_CMD_MSG {
public:
    RBT_AXIS_ENABLE():RBT_AXIS_CMD_MSG(RBT_AXIS_ENABLE_TYPE,
				    sizeof(RBT_AXIS_ENABLE)) {
    };
};

class RBT_AXIS_DISABLE:public RBT_AXIS_CMD_MSG {
public:
    RBT_AXIS_DISABLE():RBT_AXIS_CMD_MSG(RBT_AXIS_DISABLE_TYPE,
				    sizeof(RBT_AXIS_DISABLE)) {
    };
};

// declarations for multi axes jog
class RBT_MULTI_AXIS_JOG:public RBT_AXIS_CMD_MSG {
public:
    RBT_MULTI_AXIS_JOG():RBT_AXIS_CMD_MSG(RBT_MULTI_AXIS_JOG_TYPE,
				    sizeof(RBT_MULTI_AXIS_JOG)) {
    };
  double vel[8]; // the maximam axes is 8 at one time
};



// declarations for RBT_TASK classes

/*
* TASK command base class.
* This is the base class for all commands that operate on a task.
*/
class RBT_TASK_CMD_MSG:public RBT_CMD_MSG {
public:
    RBT_TASK_CMD_MSG(MRLTYPE t, size_t s):RBT_CMD_MSG(t, s) {
    };
    // 0x01 = X, 0x02 = Y, 0x04 = Z, etc.
    unsigned int mark;
};

class RBT_TASK_ENABLE_TASK:public RBT_TASK_CMD_MSG {
public:
    RBT_TASK_ENABLE_TASK():RBT_TASK_CMD_MSG(RBT_TASK_ENABLE_TASK_TYPE,
				    sizeof(RBT_TASK_ENABLE_TASK)) {
    };
};

class RBT_TASK_DISABLE_TASK:public RBT_TASK_CMD_MSG {
public:
    RBT_TASK_DISABLE_TASK():RBT_TASK_CMD_MSG(RBT_TASK_DISABLE_TASK_TYPE,
				    sizeof(RBT_TASK_DISABLE_TASK)) {
    };
};

class RBT_TASK_START_FORCE_THREAD:public RBT_TASK_CMD_MSG {
public:
    RBT_TASK_START_FORCE_THREAD():RBT_TASK_CMD_MSG(RBT_TASK_START_FORCE_THREAD_TYPE,
                                             sizeof(RBT_TASK_DISABLE_TASK)) {
    };
    
};

class RBT_TASK_TERMINATE_FORCE_THREAD:public RBT_TASK_CMD_MSG {
public:
    RBT_TASK_TERMINATE_FORCE_THREAD():RBT_TASK_CMD_MSG(RBT_TASK_TERMINATE_FORCE_THREAD_TYPE,
                                             sizeof(RBT_TASK_DISABLE_TASK)) {
    };
};

// declarations for RBT_IO classes

/*
* IO command base class.
* This is the base class for all commands that operate on the IO.
*/
class RBT_IO_CMD_MSG:public RBT_CMD_MSG {
public:
    RBT_IO_CMD_MSG(MRLTYPE t, size_t s):RBT_CMD_MSG(t, s) {
    };
    unsigned int mark; // 暂时没有意义，以后用来区分卡的IO。
};

class RBT_IO_GET_DI:public RBT_IO_CMD_MSG {
public:
    RBT_IO_GET_DI():RBT_IO_CMD_MSG(RBT_IO_GET_DI_TYPE,
				    sizeof(RBT_IO_GET_DI)) {
    };
	int DiType;
};

class RBT_IO_SET_DO_BIT:public RBT_IO_CMD_MSG {
public:
    RBT_IO_SET_DO_BIT():RBT_IO_CMD_MSG(RBT_IO_SET_DO_BIT_TYPE,
				    sizeof(RBT_IO_SET_DO_BIT)) {
    };

	int DoIndex;
	unsigned int value;
};

class RBT_IO_SET_DO:public RBT_IO_CMD_MSG {
public:
    RBT_IO_SET_DO():RBT_IO_CMD_MSG(RBT_IO_SET_DO_TYPE,
				    sizeof(RBT_IO_SET_DO)) {
    };

	int DoType;
	unsigned int value;
};

class RBT_IO_GET_DO:public RBT_IO_CMD_MSG {
public:
    RBT_IO_GET_DO():RBT_IO_CMD_MSG(RBT_IO_GET_DO_TYPE,
				    sizeof(RBT_IO_GET_DO)) {
    };

	int DoType;
};

class RBT_IO_GET_AI:public RBT_IO_CMD_MSG {
public:
    RBT_IO_GET_AI():RBT_IO_CMD_MSG(RBT_IO_GET_AI_TYPE,
                    sizeof(RBT_IO_GET_AI)){

    };
    
    unsigned int ai_mark;
};

//[append] append message step 2

#endif //end of __RBT_MSG_H_
