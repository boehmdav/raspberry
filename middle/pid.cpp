#include "pid.h"

PID::PID(float KP, float KI, float KD, short tval, short imax) :
P(KP), I(KI), D(KD), target_value(tval), deviation_sum(0), old_deviation(0), _imax(imax)
{
	deviation_sum = 0;
	old_deviation = 0;
}

short PID::get(short current_value /*Istgroesze*/, float sampling_time /*Abtastzeit*/) {
	if (sampling_time == 0) return 0;
	sampling_time = sampling_time/1000;
	/*Regelabweichung*/ short deviation = current_value - target_value;
	deviation_sum += deviation;
	float ival = I*(float)sampling_time*(float)deviation_sum;
	if (ival > _imax) ival = _imax;
	else if (ival < -_imax) ival = -_imax;
	short return_value = P*(float)deviation + ival - D*((float)deviation - (float)old_deviation)/(float)sampling_time;
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