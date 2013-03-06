#ifndef DA_SRF_H
#define DA_SRF_H

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <fstream>
#include <math.h>

#include "defines.h"
#include "config.h"

#define BS 	SE_DATA_BUFFER_SIZE

class SRF {
public:
	SRF(unsigned short addr, int srf_fd, unsigned char alignment);	// Konstruktor
	void measure();							// Fordert den Sensor auf, eine Messung zu starten 
	short read_it(unsigned long msec);				// Fordert den Sensor auf, die Daten zu lesen

	void reset();							// Löscht die gespeicherten Daten
									// !!! Filter wird für die nächsten SE_DATA_BUFFER_SIZE-Messungen falsche Werte liefern!
	
	unsigned long 	get_msec() 	{return tv_msec;}
	unsigned long 	get_old_msec()	{return old_tv_msec;}
	unsigned short 	get_mean()	{return mean;}
	void		set_mean(unsigned short m) {mean = m;}
	unsigned short 	get_old_mean()	{return old_mean;}
	unsigned short 	get_data()	{return data[(data_pos-1+SE_DATA_BUFFER_SIZE)%SE_DATA_BUFFER_SIZE];}
	short 		get_mean_diff()	{return (short)(mean - old_mean);}
	short 		get_msec_diff()	{return (short)(tv_msec - old_tv_msec);}
	float		get_cm_per_s()	{if (tv_msec - old_tv_msec == 0) {return 0;} else {return (float)(mean - old_mean)/((float)(tv_msec - old_tv_msec)/1000);}}
	unsigned short 	get_delay()	{return delay;}
	unsigned char	get_alignment()	{return alignment;}
	enum srf_error 	get_error()	{srf_error tmp = error; error = SRF_OK; return tmp;}

private:
	unsigned short _addr;
	unsigned char  alignment;
	
	unsigned short mean;
	unsigned long   tv_msec;
	
	unsigned short old_mean;
	unsigned long  old_tv_msec;
	
	int _srf_fd;
	enum srf_error error;
	short data[SE_DATA_BUFFER_SIZE];
	short data_pos;
	
	unsigned short delay;
	unsigned char  state;
	
	short read_measure();
	short validate();
};

#endif
