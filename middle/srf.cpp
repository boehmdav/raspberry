#include "srf.h"

#include <iostream>

SRF::SRF(unsigned short addr) :
_addr(addr),mean(0),tv_msec(0),old_mean(0),old_tv_msec(0),error(SRF_OK),data_pos(0)
{
	for(int i = 0; i < SE_DATA_BUFFER_SIZE; i++) data[i] = MAX_DISTANCE;
	srf_fd = open(SRF_DEVICE, O_RDWR);
	if (srf_fd == -1) {perror("SETUP: " SRF_DEVICE "kann nicht geoeffnet werden."); error = SRF_INIT_OPEN;}
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
	if (ioctl(srf_fd, I2C_SLAVE, _addr) < 0) {perror("MEASURE: Es konnte nicht auf den I2C-Bus zugegriffen werden."); error = SRF_MEASURE_IOCTL; return;}
	unsigned char buf[] = {0,0x51}; 
	if (write(srf_fd, buf, 2) != 2) {if(WARNINGS){perror("MEASURE: Es konnte nicht auf den I2C-Bus geschrieben werden.");} error = SRF_MEASURE_WRITE;}
}

/*Fragt die Messdaten des Sensors mit der Adresse address ab und uebergibt sie als short*/
short SRF::read_measure() {
	if (ioctl(srf_fd, I2C_SLAVE, _addr) < 0) {perror("READ_MEASURE: Es konnte nicht auf den I2C-Bus zugegriffen werden."); error = SRF_READ_IOCTL;}
	unsigned char buf[2]; buf[0] = 0x02;
	if (write(srf_fd, buf, 1) != 1) {if(WARNINGS){perror("READ_MEASURE: Es konnte nicht auf den I2C-Bus geschrieben werden.");} error = SRF_READ_WRITE; return MAX_DISTANCE;}
	if (read(srf_fd, buf, 2) < 1)  {if(WARNINGS){perror("READ_MEASURE: Es konnte nicht vom I2C-Bus gelesen werden.");} error = SRF_READ_READ; return MAX_DISTANCE;}
	return ((buf[0]<<8) + buf[1]);
}

/*Erweiterter Mittelwertsfilter zur Glaettung der Messwerte*/
short SRF::validate() {
  	/*Mittelwert und Standardabweichung berechnen*/
	short _mean = 0; short std = 0;
	for(int i = 0; i < SE_DATA_BUFFER_SIZE; i++) _mean += data[i];
	_mean = _mean/SE_DATA_BUFFER_SIZE;
	for(int i = 0; i < SE_DATA_BUFFER_SIZE; i++) std += pow((data[i] - _mean), 2);
	std = pow(std/SE_DATA_BUFFER_SIZE, 0.5);

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
}

short SRF::read_it(unsigned int msec) {
	data[data_pos] = min<short>(read_measure(),MAX_DISTANCE);
	data_pos = (data_pos+1)%SE_DATA_BUFFER_SIZE;
	old_mean = mean; old_tv_msec = tv_msec;
	mean = validate();
	tv_msec = msec;
	return mean;
}

void SRF::debug_read(unsigned int msec, unsigned short val) {
	data[data_pos] = min<short>(val,MAX_DISTANCE);
	data_pos = (data_pos+1)%SE_DATA_BUFFER_SIZE;
	old_mean = mean; old_tv_msec = tv_msec;
	mean = validate();
	tv_msec = msec;
	std::cout << val << " " << mean << " " << _addr << "\n";
}

void SRF::reset() {
	mean = 0;
	old_mean = 0;
	error = SRF_OK;
	data_pos = 0;
	for(int i = 0; i < SE_DATA_BUFFER_SIZE; i++) data[i] = MAX_DISTANCE;
}