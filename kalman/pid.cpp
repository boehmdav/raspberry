#include "pid.h"
#include <iostream>

PID::PID(float KP, float KI, float KD, short tval, short imax, short imin) :
P(KP), I(KI), D(KD), target_value(tval), old_deviation(0), deviation_sum(0), _imax(imax), _imin(imin)
{
}

short PID::get(short current_value /*Istgroesze*/, float sampling_time /*Abtastzeit*/) {
	if (sampling_time == 0) return 0;
	sampling_time = sampling_time/1000;
	/*Regelabweichung*/ short deviation = current_value - target_value;
	deviation_sum += (float)sampling_time*(float)deviation;
	float ival = I*(float)deviation_sum;
	if (ival > _imax) {
		ival = _imax;
		if (I != 0) deviation_sum = (float)_imax/I;
	}
	else if (ival < _imin) {
		ival = _imin;
		if (I != 0) deviation_sum = (float)_imin/I;	
	}
	dval = D*((float)old_deviation - (float)deviation)/(float)sampling_time;
	short return_value = P*(float)deviation + ival - dval;
	old_deviation = deviation;
	return return_value;
}

void PID::reset() {
	deviation_sum = 0;
	old_deviation = 0;
}

void PID::set(float vp, float vi, float vd) {
	P = (float) vp;
	I = (float) vi;
	D = (float) vd;
}

void PID::set_target(short tval) {
	target_value = tval;
}

void PID::set_ival(float ival) {
	if (I != 0) deviation_sum = ival/I;
}

float PID::get_pval(short current_value) {
	return P*(float)(current_value - target_value);
}

float PID::get_ival() {
	if (I != 0) return (float)I*deviation_sum;
	return 0;
}

float PID::get_dval() {
	return dval;
}