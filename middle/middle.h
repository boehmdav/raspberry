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
#include "../../mavlink_huch/huch/mavlink.h"

#include "defines.h"
#include "config.h"
#include "slam.h"
#include "pid.h"
#include "srf.h" 

void measure(short address);
unsigned int get_current_millis();
void schedule_add(unsigned int plus, enum sched_tasks current_task, short param);
void setup();
void loop();

/*Variablen fuer die serielle Kommunikation*/
int tty_fd;
struct termios attr;
fd_set tty_fdset;
unsigned char first_heartbeat;
short current_heading, old_heading;

/*Variablen fuer die Nutzung der Ultraschallsensoren*/
int srf_fd;
unsigned char srf_speed[SE_COUNT];
std::vector<SRF> srf;

unsigned char nvalue[SE_COUNT];
unsigned char nvalue2[SE_COUNT];

unsigned char still;
int16_t xacc, yacc;

/*Variablen fuer den Scheduler*/
struct sched_task scheduler[MAX_TASKS];
struct timeval tv_start;

/*Variablen fuer die externe Kontrolle*/
enum states state;
short roll, pitch, yaw;
short first_heading;
short env[360];
short rotation;

#if LOG > 0
/*Variablen fuer das Logging*/
FILE *fd_112, *fd_113, *fd_114, *fd_115, *fd_116, *fd_data;
char log_dir[32];
#endif

PID pid_roll(HOLD_STILL_ROLL_KP,HOLD_STILL_ROLL_TN,HOLD_STILL_ROLL_TV,0);
PID pid_pitch(HOLD_STILL_PITCH_KP,HOLD_STILL_PITCH_TN,HOLD_STILL_PITCH_TV,0);

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