#ifndef DA_PID_H
#define DA_PID_H

class PID {
public:
	PID(float KP, float KI, float KD, short tval);
	short get(short current_value, float sampling_time);
	void reset();
	void set_target(short tval);

private:
	float P, I, D;
	short target_value, old_deviation;
	int deviation_sum;
};

#endif
