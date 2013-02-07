#ifndef DA_PID_H
#define DA_PID_H

class PID {
public:
	PID(float KP, float KI, float KD, short tval);		// Konstruktor
	short get(short current_value, float sampling_time);	// Berechnet den Regelwert an Hand des Istwertes und der Abtastzeit
	void reset();						// Setzt I- und D-Anteil zurück
	void set_target(short tval);				// Setzt eine neue Sollgröße für den Regler

private:
	float P, I, D;
	short target_value, old_deviation;
	int deviation_sum;
};

#endif
