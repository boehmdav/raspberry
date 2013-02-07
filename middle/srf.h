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

class SRF {
public:
	SRF(unsigned short addr);					// Konstruktor
	void measure();							// Fordert den Sensor auf, eine Messung zu starten 
	short read_it(unsigned int msec);				// Fordert den Sensor auf, die Daten zu lesen

	void reset();							// Löscht die gespeicherten Daten
									// !!! Filter wird für die nächsten SE_DATA_BUFFER_SIZE-Messungen falsche Werte liefern!
	void debug_read(unsigned int msec, unsigned short val);
	
	unsigned short 	get_msec() 	{return tv_msec;}
	unsigned short 	get_old_msec()	{return old_tv_msec;}
	unsigned short 	get_mean()	{return mean;}
	unsigned short 	get_old_mean()	{return old_mean;}
	unsigned short 	get_data()	{return data[(data_pos-1+SE_DATA_BUFFER_SIZE)%SE_DATA_BUFFER_SIZE];}
	short 		get_mean_diff()	{return (short)(mean - old_mean);}
	short 		get_msec_diff()	{return (short)(tv_msec - old_tv_msec);}
	enum srf_error 	get_error()	{srf_error tmp = error; error = SRF_OK; return tmp;}

private:
	unsigned short _addr;
	
	unsigned short mean;
	unsigned int   tv_msec;
	
	unsigned short old_mean;
	unsigned int   old_tv_msec;
	
	int srf_fd;
	enum srf_error error;
	short data[SE_DATA_BUFFER_SIZE];
	short data_pos;
	
	short read_measure();
	short validate();
};

#endif
