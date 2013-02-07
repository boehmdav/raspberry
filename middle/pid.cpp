#include "pid.h"

PID::PID(float KP, float KI, float KD, short tval) :
P(KP), I(KI), D(KD), target_value(tval)
{
	deviation_sum = 0;
	old_deviation = 0;
}

short PID::get(short current_value /*Istgroesze*/, float sampling_time /*Abtastzeit*/) {
	if (sampling_time == 0) return 0;
	sampling_time = sampling_time/1000;
	/*Regelabweichung*/ short deviation = current_value - target_value;
	deviation_sum += deviation;
	short return_value = P*(float)deviation + I*(float)sampling_time*(float)deviation_sum + D*((float)deviation - (float)old_deviation)/(float)sampling_time;
	//std::cout << "P:" << P << " I:" << I << " D:" << D << " deviation:" << deviation << " P*d:" <<  P*(float)deviation << "\n";
	old_deviation = deviation;
	return return_value;
}

void PID::reset() {
	deviation_sum = 0;
	old_deviation = 0;
}

void PID::set_target(short tval) {
	target_value = tval;
}