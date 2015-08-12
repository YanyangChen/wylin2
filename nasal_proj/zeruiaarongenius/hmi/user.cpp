#include <stdio.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
#include <errno.h>
//#include <string.h>
#include <pthread.h>
#include "sleep_time.h"





#include "../msg/return_code.h"
#include "../msg/rbt_msg.h"
#include "../config.h"
#include "keyboard_hit.h"
#include "clientuser.h"
using namespace std;
//#define CORE_HMI_STATUS_SHARE_MEM_ID  1000
//#define CORE_HMI_COMMAND_SHARE_MEM_ID 1010


static int counter = 0;
static RBT_TRAJ_LINEAR_MOVE   traj_linear_move;
static RBT_AXIS_JOG           axis_jog;
static RBT_MULTI_AXIS_JOG     multi_axis_jog;
static RBT_TASK_ENABLE_TASK   task_enable_task;
static RBT_TASK_DISABLE_TASK  task_disable_task;
static RBT_IO_GET_AI          io_get_ai;
static RBT_TASK_START_FORCE_THREAD  task_start_force_thread;
static RBT_TASK_TERMINATE_FORCE_THREAD   task_terminate_force_thread;


RBT_CMD_CHANNEL*	pCoreCommandBuffer;
RBT_CMD_MSG*        pCoreCommand;
RBT_STAT_CHANNEL*	pCoreStatusBuffer;
RBT_STAT*           pCoreStatus;

#define  MAX_TRIG_NUM  4

long gettime();

int monitor_serialport();
int check_event();


class CTrigTimeQueue{
private:
    int trig_size;
    int head, tail;
    long* time_q;

public:
    CTrigTimeQueue() {head=0, tail=0, trig_size = 0, time_q = NULL;};
    CTrigTimeQueue(int size) {head=0, tail=0, trig_size = size; if (trig_size > 0) time_q = new long [trig_size];};
    ~CTrigTimeQueue(){if(time_q) delete [] time_q;};

    int record_trig(){
        int k = tail;
        if (time_q == NULL) return RTN_ERROR;
        tail = (tail+1) % trig_size;
        if (tail == head) head = (head + 1) % trig_size;
        time_q[tail] = gettime();
        printf("trig time : %ld\t head = %d, tail = %d\n", time_q[tail] - time_q[k], head, tail);
        return RTN_OK;
    };

    int set_time_q(){head=0, tail=0; return RTN_OK;};
    int check_trig_down(long interval_ms_min, long interval_ms_max, int trig_num) {
        int j, kp, k;
        long delta;
        if (time_q == NULL) return 0;
        k = tail;
        if(tail < head) k = tail + trig_size;
        if ((k - head) < trig_num) return 0;
        k = tail;
        for (j=0; j<trig_num-1; j++) {
            kp = (k + trig_size -1) % trig_size;
            delta = (time_q[k] - time_q[kp]);
            if (delta > interval_ms_max || delta < interval_ms_min) return 0;
            k = kp;
        }
        return 1;
    };
};

CTrigTimeQueue trig_down(10);
CTrigTimeQueue large_trig_up(10);

static char username[256]="default";

// control value
static double ctrl_var[4]={2, -6500, 500000, -2000};
static double ctrl_var_offset[4] = {0, 0, 0, 0};
// Threshold:
// [0]:control direction
// [1]:switch control axis
// [2]:control mode - down:switch axis; up-control direction
struct FloatPair{
  float left, right;
};
static FloatPair Threshold[3];// ={15, 15, 10};
static FloatPair large_angle_threshold;// = 65;

// speaker
static int speak_flag = 1;

// show delta angle
static int delta_angle_show = 0;

// motion control mode
static int motion_state = 0; // 0: galil motion disable   1: galil motion enable
// IMU
static float YPR[3];
static float ref_YPR[3];
// serial port
int tty_fd;

int load_control_value(const char* user)
{
  char filename[256];
  char str[256], *pstr;
  int n;
  FILE* rf = NULL;
  double t_ctrl_var[4];
  FloatPair t_threshold[3], t_large_angle_threshold;
  if (user == NULL) return RTN_ERROR;

  memcpy(t_ctrl_var, ctrl_var, sizeof(ctrl_var));
  memcpy(t_threshold, Threshold, sizeof(Threshold));
  memcpy(&t_large_angle_threshold, &large_angle_threshold, sizeof(large_angle_threshold));

  sprintf(filename, "./user_data/%s.ini", user);
  rf = fopen(filename, "rt");
  cout << "open file sucess" << endl;
  if (rf <= 0) {return RTN_ERROR;}

  pstr = fgets(str, 255, rf);
  if (pstr == NULL) {fclose(rf); return RTN_ERROR;}
  n = sscanf(str, "%lf, %lf, %lf, %lf", &ctrl_var[0], &ctrl_var[1], &ctrl_var[2], &ctrl_var[3]);
  if (n != 4) {fclose(rf); return RTN_ERROR;}

  pstr = fgets(str, 255, rf);
  if (pstr == NULL) {fclose(rf); return RTN_ERROR;}
  sscanf(str, "%f, %f, %f, %f, %f, %f, %f, %f", &t_threshold[0].left, &t_threshold[0].right,
         &t_threshold[1].left, &t_threshold[1].right,
         &t_threshold[2].left, &t_threshold[2].right,
         &t_large_angle_threshold.left, &t_large_angle_threshold.right);
  if (n != 8) {fclose(rf); return RTN_ERROR;}

  memcpy(ctrl_var, t_ctrl_var, sizeof(ctrl_var));
  memcpy(Threshold, t_threshold, sizeof(Threshold));
  memcpy(&large_angle_threshold, &t_large_angle_threshold, sizeof(large_angle_threshold));

  fclose(rf);

  return RTN_OK;
}

int save_control_value(const char* user)
{
  char filename[256];
  FILE* wf = NULL;
  if (user == NULL) return RTN_ERROR;

  sprintf(filename, "./user_data/%s.ini", user);
  wf = fopen(filename, "wt+");
  if (wf <= 0) return RTN_ERROR;

  fprintf(wf, "%.2f, %.2f, %.2f, %.2f\n", ctrl_var[0], ctrl_var[1], ctrl_var[2], ctrl_var[3]);
  fprintf(wf, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
          Threshold[0].left, Threshold[0].right,
          Threshold[1].left, Threshold[1].right,
          Threshold[2].left, Threshold[2].right,
          large_angle_threshold.left, large_angle_threshold.right);
  fclose(wf);
  return RTN_OK;
}

int show_control_value()
{
  printf("%.2f, %.2f, %.2f, %.2f\n", ctrl_var[0], ctrl_var[1], ctrl_var[2], ctrl_var[3]);
  printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
         Threshold[0].left, Threshold[0].right,
         Threshold[1].left, Threshold[1].right,
         Threshold[2].left, Threshold[2].right,
         large_angle_threshold.left, large_angle_threshold.right);
  return RTN_OK;
}

int init_serialport(const char* port)
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
    char st[1024];
    int tail = -1;
    float ypr[3];
    int i;
    if (len >= 1024) len = 1023;

    memcpy(st, pst, len);
    st[len] = 0;
    for (i = len-1; i >= 0 ; i--) {
        if (st[i] == '\n') tail = len;
        if (st[i] == '#' && tail >= 0) {
            sscanf(&st[i], "#YPR=%f,%f,%f", &ypr[0], &ypr[1], &ypr[2]);
            memcpy(YPR, ypr, sizeof(float)*3);
						memcpy(pCoreStatus->imu.current_YPR, ypr, sizeof(float)*3);
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
    if (n < 0) return RTN_OK;

    st[n] = 0;
    memcpy(&str[len], st, n);
    len += n;
    len = scan_YPR(str, len);
    //printf("YPR: %.02f %.02f %.02f     \t\t\r", YPR[0], YPR[1], YPR[2]);
    return RTN_OK;
}

long gettime()
{
    static int flag = 0;
    static timeval start;
    long mtime, seconds, useconds;
    timeval end;
    if (flag == 0){
        flag = 1;
        gettimeofday(&start, NULL);
    }
    gettimeofday(&end, NULL);
    seconds = end.tv_sec - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = long((seconds*1000 + useconds/1000.0) + 0.5);
    return mtime;
}


int init()
{

  // create message channels
  pCoreStatusBuffer = new RBT_STAT_CHANNEL(CORE_HMI_STATUS_SHARE_MEM_ID, sizeof(RBT_STAT));
  pCoreStatus = (RBT_STAT *)pCoreStatusBuffer->get_address();
  pCoreCommandBuffer = new RBT_CMD_CHANNEL(CORE_HMI_COMMAND_SHARE_MEM_ID, 4096);
  pCoreCommand = pCoreCommandBuffer->get_address();
  pCoreCommand->serial_number = pCoreStatus->echo_serial_number = 0;
  printf("create share memory and channel \n");

  // initial parameters
  Threshold[0].left = -12;  Threshold[0].right = 12;
  Threshold[1].left = -15;  Threshold[1].right = 15;
  Threshold[2].left = -10;  Threshold[2].right = 10;
  large_angle_threshold.left = -65; large_angle_threshold.right = 65;

  // initial serial port
  init_serialport("/dev/ttyUSB0");
  sleep(1);
  monitor_serialport(); memcpy(ref_YPR, YPR, sizeof(YPR));
	memcpy(pCoreStatus->imu.reference_YPR, ref_YPR, sizeof(YPR));
  printf("\nref_YPR: %.2f, %.2f, %.2f\n", ref_YPR[0], ref_YPR[1], ref_YPR[2]);
  return RTN_OK;
}

int check_event()
{
    if (pCoreCommand->serial_number != pCoreStatus->echo_serial_number) return RTN_OK;
    return RTN_ERROR;
}

int write_command_buffer(RBT_CMD_MSG& msg)
{
    long begin, cur;
    begin = gettime();
    cur = begin;
    while((cur - begin) < 100){
        if (check_event() == RTN_ERROR) {
            pCoreCommandBuffer->write(msg);
            return RTN_OK;
        }
        usleep(1);
        cur = gettime();
    }
    return RTN_ERROR;
}


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

int speak_words(char *words, int flag)
{
		int rtn;
    char str[256]={0};
    if (words == NULL) return 0;
    if (!flag) return 0;
    sprintf(str, "espeak \'%s\' -v f3 -s 140 -a 50 --stdout | aplay -q &", words);
    rtn = system(str);
    return 1;
}

int MapJoint(int joint){
  switch(joint){
  case 2:
    return 3;
  case 3:
    return 2;
  default:
    break;
  }
  return joint;
}

int IMU_Gesture()
{
  int i=0, k = 0;
  static int current_joint = 0, map_current_joint = 0;
  char str[256];
  float delta[3], dir=0;
  static int trig[MAX_TRIG_NUM][3]={{0}};
  static int large_angle_trig[2]={0,0};
  static int ptr=-1, cnt=0;
  clock_sub(delta, YPR, ref_YPR);
	memcpy(pCoreStatus->imu.delta_YPR, delta, sizeof(delta));
  if (delta_angle_show) printf("delta: %.02f; %.02f; %.02f            \r", delta[0], delta[1], delta[2]);
  
  if (ptr < 0) { memset(trig, 0, sizeof(trig)); ptr=0; }
  
  // check imu
  k = ptr;
  ptr = (ptr+1) % MAX_TRIG_NUM;
  for (i=0; i<3; i++) {
    trig[ptr][i] = (delta[i] > Threshold[i].right) ? trig[ptr][i] = 1 : ((delta[i] < Threshold[i].left)? -1 : 0);
  }
  
  //if(trig[ptr][2]==0 && trig[k][2] != 0) trig_down.record_trig();
  
  large_angle_trig[0] = large_angle_trig[1];
  large_angle_trig[1] = (delta[2] > large_angle_threshold.right || delta[2] < large_angle_threshold.left) ? 1 : 0;
  if (large_angle_trig[0] == 0 && large_angle_trig[1]) large_trig_up.record_trig();
  
  //
  if (trig[ptr][2] != 0) {
    // check the direction
    dir = (trig[ptr][0] >0) ? 1 : ((trig[ptr][0] < 0)? -1: 0);
    
    if(motion_state) {
      multi_axis_jog.serial_number = ++counter;
      multi_axis_jog.mark = 0x0f;
      memcpy(multi_axis_jog.vel, ctrl_var_offset, 4*sizeof(double));
      map_current_joint = MapJoint(current_joint);
      multi_axis_jog.vel[map_current_joint] = dir*ctrl_var[map_current_joint] + ctrl_var_offset[map_current_joint];
      write_command_buffer(multi_axis_jog);
    }
    }
  
  if (trig[ptr][2] == 0)
    {
      ref_YPR[0] = YPR[0];
			pCoreStatus->imu.reference_YPR[0] = ref_YPR[0];
      cnt++;
      if(memcmp(multi_axis_jog.vel, ctrl_var_offset, 4*sizeof(double))!=0 || cnt > 100) {
        cnt = 0 ;
        multi_axis_jog.serial_number = ++counter;
        multi_axis_jog.mark = 0x0f;
        memcpy(multi_axis_jog.vel, ctrl_var_offset, 4*sizeof(double));
        write_command_buffer(multi_axis_jog);
      }
      
      // joint switch function code
      if(trig[ptr][1] != 0 && trig[k][1] == 0 && motion_state == 1){
        if(trig[ptr][1] > 0){
          current_joint = (current_joint + 1) %4;
          printf("\ncurrent joint %d\n", current_joint+1);
          sprintf(str, "joint %d", current_joint+1);
          speak_words(str, speak_flag);
        }else if (trig[ptr][1] < 0){
          current_joint = (current_joint + 3) %4;
          printf("\ncurrent joint %d\n", current_joint+1);
          sprintf(str, "joint %d", current_joint+1);
          speak_words(str, speak_flag);
        }
      }
    }
  
  if (trig_down.check_trig_down(200, 800, 3) || large_trig_up.check_trig_down(200, 2000, 2)) {
    trig_down.set_time_q();
    large_trig_up.set_time_q();
    if (motion_state == 1){
      usleep(50);
      //task_disable_task.serial_number = ++counter;
      //write_command_buffer(task_disable_task);
      sprintf(str, "stop");
    }else{
      usleep(50);
      task_enable_task.serial_number = ++counter;
      write_command_buffer(task_enable_task);
      sprintf(str, "start");
    }
    motion_state = !motion_state;
    
    if(motion_state == 0){
      multi_axis_jog.serial_number = ++counter;
      multi_axis_jog.mark = 0x0f;
      memcpy(multi_axis_jog.vel, ctrl_var_offset, 4*sizeof(double));
      write_command_buffer(multi_axis_jog);
    }
    speak_words(str, 1);
  }
  
pCoreStatus->imu.state = motion_state;
pCoreStatus->imu.act_jnt = current_joint;
  return RTN_OK;
}

int nasal_imu_key(int key)
{
    char str[256];
    switch(key){
    case '\\':
      memcpy(ref_YPR, YPR, sizeof(YPR));
			memcpy(pCoreStatus->imu.reference_YPR, ref_YPR, sizeof(YPR));
      printf("\nref_YPR: %.2f, %.2f, %.2f", ref_YPR[0], ref_YPR[1], ref_YPR[2]);
      return RTN_OK;

    case 'p':
      speak_flag = !speak_flag;
      if (speak_flag) {
        sprintf(str, "voice opened");
      }else{
        sprintf(str, "voice closed");
      }
      speak_words(str, 1);
      return RTN_OK;

    default:
      break;
    }
    return RTN_OK;
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
    write_command_buffer(axis_jog);
    //pCoreCommandBuffer->write(axis_jog);
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
  char tmp_username[256];
    int pressed_key;
    int STOP = 0;
//void clientdef (void);
#define MY_PORT	8765//1234

//void *fast_send(void *ptr); // fast loop... intended to send the force data
//void *slow_receive(void *ptr); // slow loop... intended to receive position measurements

// needed for socket creation and usage
int connectionFd, _true = 1;
struct sockaddr_in servaddr;
char in_buffer[1024], out_buffer[1024];
char zsignal;
double var[3] = {2.2};

//int clientfunc (void);



int rc1; // For the returned values
	pthread_t thread1; // Threads pointer's
	
	// create socket
	if ( (connectionFd = socket( AF_INET, SOCK_STREAM, 0 )) == -1 ) {
		printf("Error creating socket\n");
		exit(1);	
	}
	
		// set socket port options
	if ( setsockopt(connectionFd, SOL_SOCKET, SO_REUSEADDR, &_true, sizeof(int)) == -1 ) {
		printf("Error setting socket port options\n");		
		exit(1);
	}
	
	// initialise address
	memset( &servaddr, 0, sizeof(servaddr) ); // fill with 0's
	servaddr.sin_family = PF_INET; // IPv4
	servaddr.sin_port = htons( MY_PORT ); // port
	servaddr.sin_addr.s_addr = inet_addr( "192.168.1.106" ); // host to network long------------------------------

	// new connected socket... WAITS HERE UNTIL CONNECTED
	if ( connect( connectionFd, (struct sockaddr *)&servaddr, sizeof(servaddr) ) == -1) {
		printf("Error while connecting\n");
		exit(1);
	}
	else
		printf("Connection established\n");
	
	// create fast thread 
	//if( (rc1=pthread_create( &thread1, NULL, &slow_receive, NULL )) )
	//{
	//	printf( "Thread creation failed: %d\n", rc1 );
		//exit(1);
	//}

	/* Wait till threads are complete before main continues. Unless we  */
	/* wait we run the risk of executing an exit which will terminate   */
	/* the process and all threads before the threads have completed.   */
	pthread_join( thread1, NULL ); 












    while(!STOP) {
      monitor_serialport();
      IMU_Gesture();
      
      //void slow_receive(void *ptr);   //in_buffer is the variable to transfer signal from client to user
      
		memset( &in_buffer, 0, sizeof(in_buffer) ); // reset memory to 0's
		recv( connectionFd, in_buffer, sizeof(in_buffer), 0 ); // receive message
		printf("Received: %s\n", in_buffer);
		sleep_time(0.1);
      
      
		zsignal = *in_buffer;
      
      
        if (zsignal == 'i' && motion_state)
      {
		  axis_jog.serial_number = ++counter;
          axis_jog.mark = 0x04;
          axis_jog.vel = 500000;
          write_command_buffer(axis_jog);
		  }
      
      if (zsignal == 'n')
      {
		  axis_jog.serial_number = ++counter; //what's axis_jog.serial_number?
          axis_jog.mark = 0x0f;
          axis_jog.vel = 0;
          write_command_buffer(axis_jog); 
		  }
		  
	  if (zsignal == 'o' && motion_state)
      {
		  axis_jog.serial_number = ++counter;
          axis_jog.mark = 0x04;
          axis_jog.vel = -500000;
          write_command_buffer(axis_jog);
		  }
      
      
      
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
               << "r: start record thread" << endl
               << "t: stop record thread" << endl
               << "s: save user configure data" << endl
               << "n: change user and load the configure data" << endl
               << "\\: set reference YPR" << endl
               << "p: open or close speaker" << endl
               << "u: show delta angle" << endl;

          // nasal robot key help
          nasal_motion_key_help();
          cout << "Esc: exit" <<endl;
          break;

        case 27:
          task_disable_task.serial_number = ++counter;
          write_command_buffer(task_disable_task);
          //pCoreCommandBuffer->write(task_disable_task);
          STOP = 1;
          printf("exit the user program\n");
          break;

        case 'a':
          io_get_ai.serial_number = ++counter;
          write_command_buffer(io_get_ai);
          //pCoreCommandBuffer->write(io_get_ai);
          break;

        case 'l':
          traj_linear_move.serial_number = ++counter;
          traj_linear_move.end.s[0] += 0.1;
          traj_linear_move.end.s[1] += 0.1;
          traj_linear_move.end.s[2] += 0.1;
          write_command_buffer(traj_linear_move);
          //pCoreCommandBuffer->write(traj_linear_move);
          break;

        case 'b':
          motion_state = 1;
          task_enable_task.serial_number = ++counter;
          write_command_buffer(task_enable_task);
          //pCoreCommandBuffer->write(task_enable_task);
          break;

        case 'g':
          motion_state = 0;
          task_disable_task.serial_number = ++counter;
          write_command_buffer(task_disable_task);
          //pCoreCommandBuffer->write(task_disable_task);
          break;

        case 'i': //offset??
          axis_jog.serial_number = ++counter;
          axis_jog.mark = 0xf;
          axis_jog.vel = 0;
          write_command_buffer(axis_jog);
          //pCoreCommandBuffer->write(axis_jog);
          break;

        case 'n':
          cout << "Enter the user name: ";
          tmp_username[0] = 0;
          cin >> tmp_username;
          if (strlen(tmp_username) == 0 ) {cout << "username error" <<endl; break;}
          sprintf(username, "%s", tmp_username);
          //cin >> username;
          cout << username << endl;
          if (load_control_value(username) == RTN_ERROR) {
            save_control_value(username);
            cout << "Create user : " << username << endl;
          }else{
            cout << username << " : load user configure data" << endl;
          }
          break;

        case 'd':
          delta_angle_show = !delta_angle_show ;
          if (delta_angle_show) cout << endl << "Show delta angle:" << endl;
          break;

        case 's':
          if (save_control_value(username) == RTN_OK) cout << username << " : save user configure data" << endl;
          break;

        case 'r':
          task_start_force_thread.serial_number = ++counter;
          write_command_buffer(task_start_force_thread);
          //pCoreCommandBuffer->write(task_start_force_thread);
          cout << endl << "start record thread" << endl;
          break;

        case 't':
          task_terminate_force_thread.serial_number = ++counter;
          write_command_buffer(task_terminate_force_thread);
          //pCoreCommandBuffer->write(task_terminate_force_thread);
          cout << "terminate record thread" << endl;
          break;

        case 'u':
          show_control_value();
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

