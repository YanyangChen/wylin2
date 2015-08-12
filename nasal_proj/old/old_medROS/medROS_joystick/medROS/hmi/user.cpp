

#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include "../msg/return_code.h"
#include "../msg/rbt_msg.h"
#include "../config.h"
#include "keyboard_hit.h"

using namespace std;
//#define CORE_HMI_STATUS_SHARE_MEM_ID  1000
//#define CORE_HMI_COMMAND_SHARE_MEM_ID 1010


static int counter = 0;
static RBT_TRAJ_LINEAR_MOVE   traj_linear_move;
static RBT_AXIS_JOG           axis_jog;
static RBT_TASK_ENABLE_TASK   task_enable_task;
static RBT_TASK_DISABLE_TASK  task_disable_task;
static RBT_IO_GET_AI          io_get_ai;
static RBT_TASK_START_FORCE_THREAD  task_start_force_thread;
static RBT_TASK_TERMINATE_FORCE_THREAD   task_terminate_force_thread;


RBT_CMD_CHANNEL*	pCoreCommandBuffer;
RBT_CMD_MSG*        pCoreCommand;
RBT_STAT_CHANNEL*	pCoreStatusBuffer;
RBT_STAT*           pCoreStatus;


int check_event();

int init()
{
    // initial
    pCoreStatusBuffer = new RBT_STAT_CHANNEL(CORE_HMI_STATUS_SHARE_MEM_ID, sizeof(RBT_STAT));
    pCoreStatus = (RBT_STAT *)pCoreStatusBuffer->get_address();
    pCoreCommandBuffer = new RBT_CMD_CHANNEL(CORE_HMI_COMMAND_SHARE_MEM_ID, 4096);
    pCoreCommand = pCoreCommandBuffer->get_address();
    pCoreCommand->serial_number = pCoreStatus->echo_serial_number = 0;

    printf("create share memory and channel \n");
    return RTN_OK;
}

int check_event()
{
    if (pCoreCommand->serial_number != pCoreStatus->echo_serial_number) return RTN_OK;
    return RTN_ERROR;
}

int manual_velocity = 1000;

double ctrl_var[4]={40000, 6500, 2.5, 3000};
double ctrl_var_offset[4] = {0, 0, 0, 0};

int nasal_motion_key(int key)
{
    unsigned int mark = 0, idx = 0;
    int bstop = 0;
    double dir = 0;

    switch (key) {
        case '/': mark = 0x01; idx = 0; dir = 1; break;
        case '*': mark = 0x01; idx = 0; dir = -1; break;
        case '-': mark = 0x01; idx = 0; bstop = 1; break;
        case '8': mark = 0x02; idx = 1; dir = 1; break;
        case '9': mark = 0x02; idx = 1; dir = -1; break;
        case '7': mark = 0x02; idx = 0; bstop = 1; break;
        case '5': mark = 0x04; idx = 2; dir = 1; break;
        case '6': mark = 0x04; idx = 2; dir = -1; break;
        case '4': mark = 0x04; idx = 0; bstop = 1; break;
        case '2': mark = 0x08; idx = 3; dir = 1; break;
        case '3': mark = 0x08; idx = 3; dir = -1; break;
        case '1': mark = 0x08; idx = 0; bstop = 1; break;
        default: return RTN_ERROR;
    }

    axis_jog.serial_number = ++counter;
    if (bstop)
         axis_jog.vel = ctrl_var_offset[idx];
    else
         axis_jog.vel = dir * ctrl_var[idx] + ctrl_var_offset[idx];

    axis_jog.mark = mark;
    pCoreCommandBuffer->write(axis_jog);
    return RTN_OK;
}

int nasal_motion_key_help()
{
    cout << "/: jonit 1 +jog" <<endl
         << "*: joint 1 -jog" <<endl
         << "-: joint 1 stop" <<endl
         << "8: joint 2 +jog" <<endl
         << "9: joint 2 -jog" <<endl
         << "7: joint 2 stop" <<endl
         << "5: joint 3 +jog" <<endl
         << "6: joint 3 -jog" <<endl
         << "4: joint 3 stop" <<endl
         << "2: joint 4 +jog" <<endl
         << "3: joint 4 -jog" <<endl
         << "1: joint 4 stop" <<endl;
    return RTN_OK;
}



int user_loop()
{
    int pressed_key;
    int STOP = 0;

    while(!STOP) {
        //  if a key is pressed
        if( kbhit() ) {
            pressed_key = readch(); // get pressed key
            switch (pressed_key) {
                case 'h':
                    cout << "a: get ai" << endl
                         << "b: galil start" << endl
                         << "g: galil stop" << endl
                         << "i: jont 1,2,3,4 set jog 0" << endl
                         << "l: linear motion" << endl
                         << "s: start record sensor value" << endl
                         << "t: stop record sensor value" << endl;
                    // nasal robot key help
                    nasal_motion_key_help();
                    cout << "Esc: exit" <<endl;
                    break;
                case 27:
                    task_disable_task.serial_number = ++counter;
                    pCoreCommandBuffer->write(task_disable_task);
                    STOP = 1;
                    printf("exit the user program\n");
                    break;

                case 'a':
                    io_get_ai.serial_number = ++counter;
                    pCoreCommandBuffer->write(io_get_ai);
                    break;

                case 's':
                    task_start_force_thread.serial_number = ++counter;
                    pCoreCommandBuffer->write(task_start_force_thread);
                    break;

                case 't':
                    task_terminate_force_thread.serial_number = ++counter;
                    pCoreCommandBuffer->write(task_terminate_force_thread);
                    break;

                case 'l':
                    traj_linear_move.serial_number = ++counter;
                    traj_linear_move.end.s[0] += 0.1;
                    traj_linear_move.end.s[1] += 0.1;
                    traj_linear_move.end.s[2] += 0.1;
                    pCoreCommandBuffer->write(traj_linear_move);
                    break;

                case 'b':
                    task_enable_task.serial_number = ++counter;
                    pCoreCommandBuffer->write(task_enable_task);
                    break;

                case 'g':
                    task_disable_task.serial_number = ++counter;
                    pCoreCommandBuffer->write(task_disable_task);
                    break;

                case 'i': //offset??
                    axis_jog.serial_number = ++counter;
                    axis_jog.mark = 0xf;
                    axis_jog.vel = 0;
                    pCoreCommandBuffer->write(axis_jog);
                    break;

                default:
                    nasal_motion_key(pressed_key);
                    break;
            }
        }
        usleep(1);
    }
    return RTN_OK;
}

int main(int argc, char* argv[]) {
    if (init() != RTN_OK) return 0;
    init_keyboard(); // start keyboard reading
    user_loop();
    close_keyboard();
    return 1;
}

