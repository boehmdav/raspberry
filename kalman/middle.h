#ifndef DA_MIDDLE_H
#define DA_MIDDLE_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <vector>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fstream>
#include "../../mavlink_huch/huch/mavlink.h"

#include "defines.h"
#include "config.h"
#include "slam.h"
#include "pid.h"
#include "srf.h" 

int s = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
struct sockaddr_in gcAddr;

void measure(short address);
unsigned long get_current_millis();
void schedule_add(unsigned long plus, enum sched_tasks current_task, short param);
void setup();
void loop();

short max_pitch;
short max_roll;
short max_thrust;
unsigned char breakpoint;
unsigned char desktop_build;
float var_s;
float var_i;
float var_b;

short rotation_angle;
short rotation_angle_sign;
short yaw_sign;
unsigned short desired_heading;
int tv_old_heading;
unsigned char hs_state;
unsigned char align_se;


/*Variablen fuer die serielle Kommunikation*/
int tty_fd;
struct termios attr;
fd_set tty_fdset;
unsigned char first_heartbeat;
short current_heading, old_heading;

float current_roll_rad;
float current_pitch_rad;

/*Variablen fuer die Nutzung der Ultraschallsensoren*/
int srf_fd;
unsigned char srf_speed[SE_COUNT];
std::vector<SRF> srf;

unsigned char nvalue[SE_COUNT];
unsigned char nvalue2[SE_COUNT];

unsigned char still;
int16_t xacc, yacc, xacc_m, yacc_m, zacc;

/*Variablen fuer den Scheduler*/
struct sched_task scheduler[MAX_TASKS];
struct timeval tv_start;

/*Variablen fuer die externe Kontrolle*/
enum states state;
short roll, pitch, yaw, thrust;
short first_heading;
short env[360];
short rotation;

#if LOG > 0
/*Variablen fuer das Logging*/
FILE *fd_112, *fd_113, *fd_114, *fd_115, *fd_116, *fd_data, *fd_max, *fd_acc;
char log_dir[32];
#endif

PID pid_thrust(THRUST_KP,THRUST_TN,THRUST_TV,0);
PID pid_roll(HOLD_STILL_ROLL_KP,HOLD_STILL_ROLL_TN,HOLD_STILL_ROLL_TV,0);
PID pid_pitch(HOLD_STILL_PITCH_KP,HOLD_STILL_PITCH_TN,HOLD_STILL_PITCH_TV,0);
PID pid_yaw(HTM_YAW_KP,HTM_YAW_TN,HTM_YAW_TV,0);
PID pid_roll_anc(ANC_ROLL_KP,ANC_ROLL_TN,ANC_ROLL_TV,ANCHOR_DISTANCE);
PID pid_pitch_anc(ANC_PITCH_KP,ANC_PITCH_TN,ANC_PITCH_TV,ANCHOR_DISTANCE);
unsigned char init_state;

template <typename T>
T max(T a, T b) {
	if (a > b) return a;
	return b;
}

template <typename T>
T min(T a, T b) {
	if (a < b) return a;
	return b;
}

template <typename T>
T sign(T value) {
	if (value < 0)       {return -1;}
	else if (value > 0)  {return  1;}
	else                 {return  0;}
}

template <typename T>
T between(T value, T min, T max) {
	if (min > max) {
		T tmp = min;
		min = max;
		max = tmp;
	}
	if (value < min) 	{return min;}
	else if (value > max)	{return max;}
	else 			{return value;}
}

#endif