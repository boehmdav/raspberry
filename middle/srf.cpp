#include "srf.h"

#include <iostream>

SRF::SRF(unsigned short addr, int srf_fd, unsigned char align) :
_addr(addr),alignment(align),mean(0),tv_msec(0),old_mean(0),old_tv_msec(0),_srf_fd(srf_fd),error(SRF_OK),data_pos(0),delay(0),state(0)
{
	for(int i = 0; i < BS; i++) data[i] = MAX_DISTANCE;
}

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

/*Veranlasst den Sensor mit der Adresse address eine Messung in cm zu starten.*/
void SRF::measure () {
	if (ioctl(_srf_fd, I2C_SLAVE, _addr) < 0) {perror("MEASURE: Es konnte nicht auf den I2C-Bus zugegriffen werden."); error = SRF_MEASURE_IOCTL; return;}
	unsigned char buf[] = {0,0x51}; 
	if (write(_srf_fd, buf, 2) != 2) {if(WARNINGS){perror("MEASURE: Es konnte nicht auf den I2C-Bus geschrieben werden.");} error = SRF_MEASURE_WRITE;}
}

/*Fragt die Messdaten des Sensors mit der Adresse address ab und uebergibt sie als short*/
short SRF::read_measure() {
	if (ioctl(_srf_fd, I2C_SLAVE, _addr) < 0) {perror("READ_MEASURE: Es konnte nicht auf den I2C-Bus zugegriffen werden."); error = SRF_READ_IOCTL;}
	unsigned char buf[2]; buf[0] = 0x02;
	if (write(_srf_fd, buf, 1) != 1) {if(WARNINGS){perror("READ_MEASURE: Es konnte nicht auf den I2C-Bus geschrieben werden.");} error = SRF_READ_WRITE; delay = 5; return mean;}
	if (read(_srf_fd, buf, 2) < 1)  {if(WARNINGS){perror("READ_MEASURE: Es konnte nicht vom I2C-Bus gelesen werden.");} error = SRF_READ_READ; delay = 5; return mean;}
	delay = 0;
	short val = (buf[0]<<8) + buf[1];
	/*if (val < SE_MIN_DISTANCE) {state = 1;}
	else if (state == 1 && val > 150 && val < 400) {val = 15;}
	else {state = 0;}*/
	return val;
}

/*Erweiterter Mittelwertsfilter zur Glaettung der Messwerte*/
short SRF::validate() {
#if SE_DATA_BUFFER_SIZE == 1
	return data[0];
#elif SE_DATA_BUFFER_SIZE == 2
	return (data[0]+data[1])/2;
#elif FILTER_MODE == MEAN
	/*Mittelwert und Standardabweichung berechnen*/
	short _mean = 0; short std = 0;
	for(int i = 0; i < BS; i++) _mean += data[i];
	_mean = _mean/BS;

	for(int i = 0; i < BS; i++) std += pow((data[i] - _mean), 2);
	std = pow(std/BS, 0.5);

	/*Glaetten der Kurve an Hand der Standardabweichung*/
	_mean = 0;
	for(int i = data_pos+1; i < data_pos-1+SE_DATA_BUFFER_SIZE; i++) {
		if (abs((short)(data[i%SE_DATA_BUFFER_SIZE] - data[(i+1)%SE_DATA_BUFFER_SIZE])) < STD_FACTOR*max<short>(std,MIN_STD)) {
			_mean += data[(i+1)%SE_DATA_BUFFER_SIZE];
		} else {
			_mean += data[i%SE_DATA_BUFFER_SIZE] + sign<short>((short)(data[(i+1)%SE_DATA_BUFFER_SIZE] - data[i%SE_DATA_BUFFER_SIZE]))*(max<short>(std,MIN_STD));
		}
	}
	return _mean/(SE_DATA_BUFFER_SIZE-2);
#elif FILTER_MODE == MEDIAN
	/*Median berechnen*/
	short tmp[BS];
	for (int i = 0; i < BS; i++) {
		tmp[i] = data[i];
	}
	for (int i = 0; i < BS; i++) {
		for (int j = 0; j < BS-i-1; j++) {
			if (tmp[j] < tmp[j+1]) {
				unsigned short v = tmp[j];
				tmp[j] = tmp[j+1];
				tmp[j+1] = v;
			}
		}
	}
 #if BS%2 == 0
	return (tmp[BS/2]+tmp[BS/2-1])/2;
 #else
	return tmp[(BS)/2];
 #endif
#else
	return data[0];
#endif
	
}

short SRF::read_it(unsigned long msec) {
	data[data_pos] = min<short>(read_measure(),MAX_DISTANCE);
	data_pos = (data_pos+1)%BS;
	old_mean = mean; old_tv_msec = tv_msec;
	mean = validate();
	//if (old_mean < SE_MIN_DISTANCE && mean > 200) mean = 0;
	tv_msec = msec;
	return mean;
}

void SRF::reset() {
	mean = 0;
	old_mean = 0;
	error = SRF_OK;
	data_pos = 0;
	for(int i = 0; i < BS; i++) data[i] = MAX_DISTANCE;
}
