#ifndef DA_CONFIG_H
#define DA_CONFIG_H

#define TTY_DEVICE		"/dev/ttyAMA0"	// Serielle Schnittstelle
#define TTY_DEVICE_SPEED	B57600		// Uebertragungsrate in Baud
#define DATA_STREAM_SPEED	5		// Geschwindigkeit des Datenstreams in Hz

#define TARGET_SYSTEM		1		// Zielsystem fuer Mavlink-Pakete
#define TARGET_COMPONENT	1		// Zielkomponente fuer Mavlink-Pakete

#define SRF_DEVICE		"/dev/i2c-1"	// I2C-BUS
#define SE_COUNT		4		// Anzahl der SRF-Sensoren (muss ein Teiler von 360 sein) ohne Sensor fuer Hoehenkontrolle 

#define SRF_SPEED		60		// Anfaengliche Verzoegerung zwischen dem Start zweier Messungen in ms
#define SE_DATA_BUFFER_SIZE	10		// Groesze des Messwertebuffers (Legt einen Freiheitsgrad des erweiterten Mittelwertfilters fest)

#define SE0_ADDRESS		112		// I2C-Adressen der Ultraschallsensoren
#define SE1_ADDRESS		113		// SE0_ADDRESS muss die kleinste Adresse haben
#define SE2_ADDRESS		114		// Adressen muessen aufeinanderfolgen
#define SE3_ADDRESS		115

#define EXT_CTRL		1		// Schaltet die externe Kontrolle an/aus
#define STD_FACTOR		2		// Faktor fuer die maximal erlaubte Abweichung von der Standardabweichung
#define MIN_STD			10		// Kleinste erlaubte Abweichung von der Standardabweichung

#define MAX_DISTANCE		600		// Groeszter moeglicher Messwert
#define MAX_ROLL		600		// Groeszter moeglicher Wert fuer roll		
#define MAX_PITCH		MAX_ROLL	// Groeszter moeglicher Wert fuer pitch

#define CONST_YAW		20		// Konstante Drehgeschwindigkeit 
#define HTM_DELAY_TIME		500		// Zeit zwischen Statuswechsel von HEAD_TO_MIDDLE zu HOLD_STILL in ms

#define HOLD_STILL_ROLL_KP	20			// Verstaerkung
#define HOLD_STILL_ROLL_TN	0			// Nachtsellzeit
#define HOLD_STILL_ROLL_TV	3			// Vorhaltzeit
#define HOLD_STILL_PITCH_KP	HOLD_STILL_ROLL_KP	// Verstaerkung
#define HOLD_STILL_PITCH_TN	HOLD_STILL_ROLL_TN	// Nachstellzeit
#define HOLD_STILL_PITCH_TV	HOLD_STILL_ROLL_TV	// Vorhaltzeit

#define HOLD_STILL_MAX_XACC	100		// Maximal erlaubte Beschleunigung auf der x-Achse in mg
#define HOLD_STILL_MAX_YACC	100		// Maximal erlaubte Beschleunigung auf der y-Achse in mg

#define STILL_FAK_ROLL		0.05		// Faktor zur Bestimmung eines Stillstandes auf der Roll-Achse
#define STILL_FAK_PITCH		0.05		// Faktor zur Bestimmung eines Stillstandes auf der Pitch-Achse
#define CHECK_STILL_SPEED	100		// Verzoegerung zwischen zwei Pruefungen auf Stillstand in ms
#define CHECK_STILL_COUNT	5		// Zahl der aufeinanderfolgenden Zahl von Pruefungen bis ein Stillstand angenommen wird

/*Definitionen fuer Debugging und Logging*/
#define DEBUG_LEVEL		0		// Level fuer die Menge der Debug-Informationen
#define LOG			1		// Schaltet das Loggin an/aus
#define LOG_SPEED		1000		// Zeitspanne zwischen zwei Sicherungen der Log-Daten in ms
#define WARNINGS		1		// Schaltet Warnungen an/aus

#define SLAM_SPEED		5000

#endif