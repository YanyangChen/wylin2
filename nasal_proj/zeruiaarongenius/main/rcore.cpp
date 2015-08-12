#include <stdio.h>
#include <unistd.h>
#include <Galil.h>
#include <pthread.h> // force thread,
#include <sys/time.h>

#include "../config.h"
#include "../msg/return_code.h"
#include "../msg/rbt_msg.h"

//#include "../sensor/ati_mini40.h" // force_card,

static RBT_TRAJ_LINEAR_MOVE*   p_traj_linear_move;
static RBT_AXIS_JOG*           p_axis_jog;
static RBT_MULTI_AXIS_JOG*     p_multi_axis_jog;
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


//Galil g("/dev/galilpci0");
//Galil g("137.189.100.100");
//Galil g("137.189.100.128");
Galil g("192.168.1.110");

// ----- [begin] force sensor  ------------

// ati_mini40  force_card;

static pthread_t thread_user; // threads' pointer
static int force_thread_active = 0;
static timeval start, end;

int initial_record_data()
{
  gettimeofday(&start, NULL);
  return 1;
}

int record_data(FILE* file)
{
  int i;
  char cmd[256];
  long mtime, seconds, useconds;
  static int position[4]={0,0,0,0};
  static double velocity[4]={0,0,0,0};
  static double current[4]={0,0,0,0};

  gettimeofday(&end, NULL);
  seconds = end.tv_sec - start.tv_sec;
  useconds = end.tv_usec - start.tv_usec;
  mtime = long((seconds*1000 + useconds/1000.0) + 0.5);

  for (i = 0; i < 4; i++){
    sprintf(cmd, "TP%c", 'A'+i);
    position[i] = g.commandValue(cmd);
    sprintf(cmd, "TV%c", 'A'+i);
    velocity[i] = g.commandValue(cmd);
    sprintf(cmd, "TT%c", 'A'+i);
    current[i] = g.commandValue(cmd);
  }

  if (file < 0) return RTN_ERROR;
  fprintf(file, "%ld %d %d %d %d %d %d %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g\n", mtime, pCoreStatus->imu.state,
pCoreStatus->imu.act_jnt,
          position[0], position[1], position[2], position[3],
          velocity[0], velocity[1], velocity[2], velocity[3],
          current[0], current[1], current[2], current[3],
					pCoreStatus->imu.current_YPR[0], pCoreStatus->imu.current_YPR[1], pCoreStatus->imu.current_YPR[2],
					pCoreStatus->imu.reference_YPR[0], pCoreStatus->imu.reference_YPR[1], pCoreStatus->imu.reference_YPR[2],
					pCoreStatus->imu.delta_YPR[0], pCoreStatus->imu.delta_YPR[1], pCoreStatus->imu.delta_YPR[2]);

  return RTN_OK;
}

void *force_thread(void *ptr)
{
  FILE* wf = NULL;
  force_thread_active = 1;
  // force_card.init(0);
  wf = fopen("dat.txt", "wt+");
  initial_record_data();

  while (force_thread_active) {
    // force_card.getvalue();
    // printf("force : %f \t\n", force_card.magnitude);
    record_data(wf);
    usleep(100*1000);
  }
  printf("force_thread_exit\n");
  fclose(wf);
}

// ----- [end] force sensor  ------------

int check_event();

int start_galildmc()
{
    g.programDownloadFile("./nasal_robot.dmc");
    g.command("XQ#BEGIN");
    printf("start gailidmc\n");
    return RTN_OK;
}

int stop_galildmc()
{
    g.command("v1=0;v2=0;v3=0;v4=0");
    g.command("OF 0,0,0,0");
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

static char ostr[256]={0};

int axis_jog_cmd(unsigned int mark, double vel)
{
    char str[256], st[64];
    int i, n=0;
    str[0] = 0;
    for (i = 0; i < 4; i++) {
        if (mark & (1<<i)) {
            sprintf(st, "v%d=%.02f;", i+1, vel);
            n += sprintf(&str[n], "%s", st);
        }
    }
    if (n == 0) {
        printf("axis_jog_cmd : the axis number error \n");
        return RTN_ERROR;
    }
    str[n]=0;
    if (memcmp(ostr, str, n) != 0){
        printf("axis_jog_cmd : %s\n", str);
        strcpy(ostr, str);
    }

    g.command(str);
    return RTN_OK;
}

int axis_multi_jog_cmd(unsigned int mark, double* vel)
{
    char str[256], st[64];
    int i, n=0;
    str[0] = 0;
    for (i = 0; i < 4; i++) {
        if (mark & (1<<i)) {
            sprintf(st, "v%d=%.02f;", i+1, vel[i]);
            n += sprintf(&str[n], "%s", st);
        }
    }
    if (n == 0) {
        printf("axis_jog_cmd : the axis number error \n");
        return RTN_ERROR;
    }
    str[n]=0;

    if (memcmp(ostr, str, n) != 0){
        printf("axis_jog_cmd : %s\n", str);
        strcpy(ostr, str);
    }

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

      case RBT_MULTI_AXIS_JOG_TYPE:
          p_multi_axis_jog = (RBT_MULTI_AXIS_JOG*) pCoreCommand;
          rtn = axis_multi_jog_cmd(p_multi_axis_jog->mark, p_multi_axis_jog->vel);
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

