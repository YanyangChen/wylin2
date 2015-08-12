

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

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

int tty_fd;
float YPR[3];
float ref_YPR[3];
int init_serialport(char* port)
{
    struct termios tio;
	tty_fd = open(port, O_RDWR | O_NONBLOCK);
    if (tty_fd < 0) {
        printf("Open com port fail\n");
        return RTN_ERROR;
    }else{
        printf("Open com port success\n");
    }

    memset(YPR, 0, sizeof(float)*3);
    memset(ref_YPR, 0, sizeof(float)*3);

    memset(&tio, 0, sizeof(tio));
	tio.c_iflag=0;
	tio.c_oflag=0;
	tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
	tio.c_lflag=0;
	tio.c_cc[VMIN]=1;
	tio.c_cc[VTIME]=5;
    cfsetospeed(&tio,B57600);            // 57600 baud
	cfsetispeed(&tio,B57600);            // 57600 baud
	tcsetattr(tty_fd,TCSANOW,&tio);
    return RTN_OK;
}

int scan_YPR(char *pst, int len)
{
    char st[256];
    int tail=-1;
    float ypr[3];
    int i, n;
    memcpy(st, pst, len);
    st[len] = 0;
    for (i = len-1; i >= 0 ; i--) {
        if (st[i] == '\n') tail = len;
        if (st[i] == '#' && tail >= 0) {
            n = sscanf(&st[i], "#YPR=%f,%f,%f", &ypr[0], &ypr[1], &ypr[2]);
            memcpy(YPR, ypr, sizeof(float)*3);
            memcpy(pst, &st[tail], len - tail);
            pst[len-tail] = 0;
            return len - tail;
        }
    }
    return len;
}

int monitor_serialport()
{
    static char str[1024]={0};
    static int len=0;

    char st[256];
    int n;
    if(tty_fd < 0)  return RTN_OK;
    n = read(tty_fd, st, 255);
    if (n<0) return RTN_OK;

    st[n] = 0;
    memcpy(&str[len], st, n);
    len += n;
    len = scan_YPR(str, len);
    //printf("YPR: %.02f %.02f %.02f     \t\t\r", YPR[0], YPR[1], YPR[2]);
    return RTN_OK;
}


int init()
{
    // initial
    init_serialport("/dev/ttyUSB0");

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




int clock_sub(float sub[3], const float val[3], const float ref_val[3])
{
    int i;
    for (i = 0; i < 3; i++)
    {
        sub[i] = val[i] - ref_val[i];
        if (sub[i] >= 180) sub[i] -= 360;
        if (sub[i] <=-180) sub[i] += 360;
    }
    return RTN_OK;
}


#define  MAX_TRIG_NUM  4

int IMU_Gesture()
{
    int i, k = 0;
    unsigned int mark=0x01;
    static int current_joint = 0;
    float delta[3], dir=0;
    float Threshold[3] ={20,15,15};
    static int trig[MAX_TRIG_NUM][3];
    static int ptr=-1, cnt=0;
    clock_sub(delta, YPR, ref_YPR);
    printf("delta: %.02f %.02f %.02f     \t\t\r", delta[0], delta[1], delta[2]);
    
    if (ptr<0) {memset(trig, 0, sizeof(trig)); ptr=0;}
    cnt = (cnt + 1) % 1;
    if (cnt != 0) return RTN_ERROR;

    k = ptr;
    ptr = (ptr+1) % MAX_TRIG_NUM;
    for (i=0; i<3; i++) {
        if(delta[i] < -Threshold[i]){
            trig[ptr][i] = -1;
        }else if (delta[i] > Threshold[i]){
            trig[ptr][i] = 1;
        }else{
            trig[ptr][i] = 0;
        }
    }

    if (trig[ptr][2] != 0) {
        if (trig[k][2] == 0){
            //    memcpy(ref_YPR, YPR, sizeof(YPR));
            ref_YPR[0] = YPR[0];
            printf("\n\nref_YPR: %.2f, %.2f, %.2f\n", ref_YPR[0], ref_YPR[1], ref_YPR[2]);

            axis_jog.serial_number = ++counter;
            axis_jog.mark = 0x0f;
            axis_jog.vel = 0;
            pCoreCommandBuffer->write(axis_jog);
            return RTN_OK;
        }

        if (trig[ptr][0] > 0){
            dir = 1;
        }else if (trig[ptr][0] < 0){
            dir = -1;
        }else{
            dir = 0;
        }

        axis_jog.serial_number = ++counter;
        axis_jog.mark = 1 << current_joint;
        axis_jog.vel = dir*ctrl_var[current_joint] + ctrl_var_offset[current_joint];
        pCoreCommandBuffer->write(axis_jog);
    }

    if (trig[ptr][2] == 0 && trig[k][2] != 0) {
        axis_jog.serial_number = ++counter;
        axis_jog.mark = 0x0f;
        axis_jog.vel = 0;
        pCoreCommandBuffer->write(axis_jog);
    }

    if (trig[ptr][1] != 0 && trig[k][1] == 0){
        if(trig[ptr][1] > 0){
            current_joint = (current_joint + 1) %4;
        }else if (trig[ptr][1] < 0){
            current_joint = (current_joint + 3) %4;
        }
        printf("\njoint %d\t\t\t\t\t\n", current_joint);
        axis_jog.serial_number = ++counter;
        axis_jog.mark = 0x0f;
        axis_jog.vel = 0;
        pCoreCommandBuffer->write(axis_jog);
    }

    return RTN_OK;
}

int nasal_imu_key(int key)
{
    unsigned int mark = 0, idx = 0;
    int i, bstop = 1;
    float delta[3];
    float Ref_Delta[3] = {20,15,15};
    double dir = 0;

    clock_sub(delta, YPR, ref_YPR);

    int chkid = 0;
    if(delta[chkid] > Ref_Delta[chkid]) dir = 1;
    else if (delta[chkid] < -Ref_Delta[chkid]) dir = -1;

    switch(key){
        case '\\':
            memcpy(ref_YPR, YPR, sizeof(YPR));
            printf("\n\n\nref_YPR: %.2f, %.2f, %.2f\n\n", ref_YPR[0], ref_YPR[1], ref_YPR[2]);
            return RTN_OK;

        case 'o':
            mark = 0x01; idx = 0;
            bstop = 0;
            break;

        case 'p':
            mark = 0x02; idx = 1;
            bstop = 0;
            break;

        case '[':
            mark = 0x04; idx = 2;
            bstop = 0;
            break;break;

        case ']':
            mark = 0x08; idx = 3;
            bstop = 0;
            break;

        default: break;
    }

    axis_jog.serial_number = ++counter;
    if (bstop) {
        mark = 0x0f;
        axis_jog.vel = 0;
    }
    else
        axis_jog.vel = dir * ctrl_var[idx] + ctrl_var_offset[idx];

    axis_jog.mark = mark;
    pCoreCommandBuffer->write(axis_jog);

}

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
        monitor_serialport();
        IMU_Gesture();
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
                    nasal_imu_key(pressed_key);
                    break;
            }
        }
        usleep(1);
    }
    if (tty_fd >= 0) close(tty_fd);
    return RTN_OK;
}

int main(int argc, char* argv[]) {
    if (init() != RTN_OK) return 0;
    init_keyboard(); // start keyboard reading
    user_loop();
    close_keyboard();
    return 1;
}

