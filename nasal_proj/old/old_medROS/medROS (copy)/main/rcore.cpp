
#include <stdio.h>
#include <unistd.h>
#include <Galil.h>
#include <pthread.h> // force thread,

#include "../config.h"
#include "../msg/return_code.h"
#include "../msg/rbt_msg.h"

//#include "../sensor/ati_mini40.h" // force_card,

// ----- [begin] force sensor  ------------

// ati_mini40  force_card;
  
static pthread_t thread_user; // threads' pointer
static int force_thread_active = 0;
 
void *force_thread(void *ptr)
{
    force_thread_active = 1;
    // force_card.init(0);
    
    while (force_thread_active) {
        // force_card.getvalue();
        // printf("force : %f \t\n", force_card.magnitude);
       usleep(1);
    }
    printf("force_thread_exit\n");
    
}


// ----- [end] force sensor  ------------


static RBT_TRAJ_LINEAR_MOVE*   p_traj_linear_move;
static RBT_AXIS_JOG*           p_axis_jog;
static RBT_IO_GET_AI*          p_io_get_ai;
static RBT_TASK_ENABLE_TASK*   p_task_enable_task;
static RBT_TASK_DISABLE_TASK*  p_task_disable_task;

// force thread
static RBT_TASK_START_FORCE_THREAD*  p_task_start_force_thread;
static RBT_TASK_TERMINATE_FORCE_THREAD*   p_task_terminate_force_thread;

RBT_CMD_CHANNEL*	pCoreCommandBuffer;
RBT_CMD_MSG*        pCoreCommand;
RBT_STAT_CHANNEL*	pCoreStatusBuffer;
RBT_STAT*           pCoreStatus;


Galil g("/dev/galilpci0");


int check_event();

int start_galildmc()
{
    //g.programDownloadFile("./embedded.dmc");
    g.programDownloadFile("./nasal_robot.dmc");
    g.command("XQ#BEGIN");
    printf("start gailidmc\n");
    return RTN_OK;
}

int stop_galildmc()
{
    g.command("ST");
    g.command("XQ#STOP");
    printf("stop galildmc\n");
    return RTN_OK;
}


// command handling function
int traj_linear_move_cmd(double x, double y, double z)
{
    printf("traj_linear_move_cmd x = %f, y = %f, z = %f\n", x, y, z);
    return RTN_OK;
}

int axis_jog_cmd(unsigned int mark, double vel)
{
    char str[256], st[64];
    int i, n=0;
    str[0] = 0;
    for (i = 0; i < 4; i++) {
        if (mark & (1<<i)) {
            sprintf(st, "v%d=%f;", i+1, vel);
            n += sprintf(&str[n], "%s", st);
        }
    }
    if (n == 0) {
        printf("axis_jog_cmd : the axis number error \n");
        return RTN_ERROR;
    }
    str[n]=0;
    printf("axis_jog_cmd : %s\n", str);
    g.command(str);
    return RTN_OK;
}

int io_get_ai_cmd(unsigned int ai_mark, double* ai_val)
{
    // NI function read ai

    return RTN_OK;

}

int task_start_force_thread_cmd()
{
    int rtn;
    if (force_thread_active != 0) {
        printf ("Force thread is running\n");
        return RTN_ERROR;
    }

    // create host's different routines
    if ((rtn = pthread_create(&thread_user, NULL, force_thread, NULL)))
        printf("Force thread creation failed: %d\n", rtn);

    //pthread_join(thread_user, NULL);

    printf("thread create success\n");
    return RTN_OK;
}

int task_terminate_force_thread_cmd()
{
    if ( force_thread_active == 0 ) {
        printf ("Force thread is not running\n");
        return RTN_ERROR;
    }

    force_thread_active = 0;
    return RTN_OK;
}
//[append] append message step 4



int handle_event();

int init()
{
    // initial
    // status of ROS
    pCoreStatusBuffer = new RBT_STAT_CHANNEL(CORE_HMI_STATUS_SHARE_MEM_ID, sizeof(RBT_STAT));
    pCoreStatus = (RBT_STAT *)pCoreStatusBuffer->get_address();

    // command channel of ROS
    pCoreCommandBuffer = new RBT_CMD_CHANNEL(CORE_HMI_COMMAND_SHARE_MEM_ID, 4096);
    pCoreCommand = pCoreCommandBuffer->get_address();

    pCoreCommand->serial_number = pCoreStatus->echo_serial_number = 0;

    printf("create share memory and channel \n");
    return RTN_OK;
}

int check_event()
{
    if (pCoreStatus->kernel.state == RBT_KERNEL_STATE_ESTOP) return RTN_ERROR;
    if (pCoreCommand->serial_number != pCoreStatus->echo_serial_number) return RTN_OK;
    return RTN_ERROR;
}

int handle_event()
{
  int rtn = RTN_OK;
  if (pCoreCommand->serial_number == pCoreStatus->echo_serial_number) return RTN_ERROR;

  switch (pCoreCommand->type) {
      // task
      case RBT_TASK_ENABLE_TASK_TYPE:
          rtn = start_galildmc();
          break;

      case RBT_TASK_DISABLE_TASK_TYPE:
          rtn = stop_galildmc();
          break;

      case RBT_TASK_START_FORCE_THREAD_TYPE:
          p_task_start_force_thread = (RBT_TASK_START_FORCE_THREAD*)pCoreCommand;
          rtn = task_start_force_thread_cmd();
          break;

      case RBT_TASK_TERMINATE_FORCE_THREAD_TYPE:
          p_task_terminate_force_thread = (RBT_TASK_TERMINATE_FORCE_THREAD*)pCoreCommand;
          rtn = task_terminate_force_thread_cmd();
          break;

      // motion
      case RBT_TRAJ_LINEAR_MOVE_TYPE:
          p_traj_linear_move = (RBT_TRAJ_LINEAR_MOVE*) pCoreCommand;
          rtn = traj_linear_move_cmd(p_traj_linear_move->end.s[0], p_traj_linear_move->end.s[1], p_traj_linear_move->end.s[2]);
          break;

      case RBT_AXIS_JOG_TYPE:
          p_axis_jog = (RBT_AXIS_JOG*) pCoreCommand;
          rtn = axis_jog_cmd(p_axis_jog->mark, p_axis_jog->vel);
          break;

      case RBT_IO_GET_AI_TYPE:
          p_io_get_ai = (RBT_IO_GET_AI*) pCoreCommand;
          double val[8];
          rtn = io_get_ai_cmd(p_io_get_ai->mark, val);
          break;

      //[append] append message step 3

      default:
          printf("command type is not defined : %ld\n", pCoreCommand->type);
          break;
    }

  pCoreStatus->echo_serial_number = pCoreCommand->serial_number; // clear the command

  return rtn;
}

int main(int argc, char* argv[]) {

    if (init() != RTN_OK) return 0;

    while(1) {
        if (check_event() == RTN_OK) {
            handle_event();
        }
        usleep(1);
    }
    return 1;
}

