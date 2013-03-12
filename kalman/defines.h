#ifndef DA_DEFINES_H
#define DA_DEFINES_H

#define COPTER_RADIUS		50	//Radius des Copters in cm

#define ENABLED			1
#define DISABLED		0
#define ENABLE			ENABLED
#define DISABLE			DISABLED

struct position {
	int x;
	int y;
};

#define ENV_ITEM_UNDEF		0
#define ENV_ITEM_UNSOLID	1
#define ENV_ITEM_SOLID		2
#define ENV_ITEM_APPROX		3
#define ENV_ITEM_PATH		4
#define ENV_ITEM_ERROR		7

#define COLOR_ITEM_UNDEF	"125 125 125"
#define COLOR_ITEM_SOLID	"0   0   0  "
#define COLOR_ITEM_UNSOLID	"255 255 255"
#define COLOR_ITEM_APPROX	"0   0   255"
#define COLOR_ITEM_PATH		"0   255 0  "
#define COLOR_ITEM_ERROR	"255 0   0  "

#define MEAN			0
#define MEDIAN			1
#define KALMAN			2

#define REAL_VAL		2 //REAL führt zu Komplikationen mit opencv
#define FAKE			1

#define CONST			0
#define LIN			1
#define RAND			2
#define SIN			3
#define DATA			4

#define PI			3.14159265358979323846f
#define RAD(X)			X*0.017453293f 		//X*PI/180
#define DEG(X)			X*57.2957795131f	//X*180/PI

#define ISSQUARE(X)		X && !(X & (X-1))

#define HTMR(X)			((X > 150) ? (MAX_ROLL) :(-0.0002*pow(X,3) + 0.0357*pow(X,2) + 2.6607*X))
#define HTMP(X)			((X > 150) ? (MAX_PITCH):(-0.0002*pow(X,3) + 0.0357*pow(X,2) + 2.6607*X))

/*Definitionen fuer den Scheduler*/
#define MAX_TASKS		16		// Maximale Zahl an Aufgaben, die der Scheduler speichern kann	
enum sched_tasks {NOTHING, MEASURE, READ_MEASURE, SEND_EXT_CTRL, SAVE_LOG, SAVE_SLAM, CHECK_STILL, SHOW_ME, CHANGE_STATE};
struct sched_task {
	unsigned long 		tv_msec;
	enum sched_tasks 	task;
	short 			param;
};

enum states {HOLD_STILL, MEASURE_ENVIRONMENT, GET_ANCHOR, DELAY, IDLE, INIT, HTM_DELAY, GET_ALIGNMENT};

enum srf_error {SRF_OK = 0,SRF_MEASURE_IOCTL,SRF_MEASURE_WRITE,SRF_READ_IOCTL,SRF_READ_WRITE,SRF_READ_READ,SRF_INIT_OPEN};

#endif