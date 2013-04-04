#ifndef DA_PID_H
#define DA_PID_H
#include "config.h"

class PID {
public:
	PID(float KP, float KI, float KD, short tval, short imax, short imin);		// Konstruktor
	short get(short current_value, float sampling_time);	// Berechnet den Regelwert an Hand des Istwertes und der Abtastzeit
	void reset();						// Setzt I- und D-Anteil zurück
	void set_target(short tval);				// Setzt eine neue Sollgröße für den Regler
	void set(float vp, float vi, float vd);
	void set_imax(short imax) 	{_imax = imax;}
	
	void set_ival(float ival);
	float get_pval(short int current_value);
	float get_ival();
	float get_dval();
	
private:
	float P, I, D;
	short target_value, old_deviation;
	float deviation_sum;
	short _imax, _imin;
	float dval;
};

#endif
