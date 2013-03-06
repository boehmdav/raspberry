#ifndef DA_CONFIG_H
#define DA_CONFIG_H

/********************************
 * Mavlink-Einstellungen
 * ******************************/
#define TTY_DEVICE		"/dev/ttyAMA0"	// Serielle Schnittstelle
#define TTY_DEVICE_SPEED	B57600		// Uebertragungsrate in Baud
#define DATA_STREAM_SPEED	50		// Geschwindigkeit des Datenstreams in Hz (Theoretisches Maximum: 50Hz, Praktisches Maximum ~15Hz)

#define TARGET_SYSTEM		1		// Zielsystem fuer Mavlink-Pakete
#define TARGET_COMPONENT	1		// Zielkomponente fuer Mavlink-Pakete

/********************************
 * Sensor-Einstellungen
 * ******************************/
#define SRF_DEVICE		"/dev/i2c-1"	// I2C-BUS
#define SE_COUNT		4		// Anzahl der SRF-Sensoren (muss ein Teiler von 360 sein) ohne Sensor fuer Hoehenkontrolle 
#define SRF_SPEED		60		// Anfaengliche Verzoegerung zwischen dem Start zweier Messungen in ms

#define ENABLED_SENSORS		0b1111		// Bitmaske zum Aktivieren der Sensoren

#define SE0_ADDRESS		112		// I2C-Adressen der Ultraschallsensoren
#define SE1_ADDRESS		SE0_ADDRESS+1	// SE0_ADDRESS muss die kleinste Adresse haben
#define SE2_ADDRESS		SE0_ADDRESS+2	// Adressen muessen aufeinanderfolgen
#define SE3_ADDRESS		SE0_ADDRESS+3

#define MAX_DISTANCE		600		// Größter möglicher Messwert
#define SE_MIN_DISTANCE		40		// Kleinster glaubhafter Sensormesswert
#define SE_MIN 			20		// Kleinster Messwert
#define SE_MIN_DIFF		30		// Minimaländerung der Messwerte, bis gegenüberliegender wieder glaubhaft ist

#define MAX_ROLL_ANGLE		0.25f		// Größter glaubhafter Winkel zur Sensorkorrektur im Bogenmaß
#define MAX_PITCH_ANGLE		MAX_ROLL_ANGLE

/********************************
 * Filter-Einstellungen
 * ******************************/
#define SE_DATA_BUFFER_SIZE	3		// Groesze des Messwertebuffers
						// 1 schaltet den Filter aus, 2 ermittelt Mittelwert zwischen den letzten beiden Messungen, >2 normales Filterverhalten
#define FILTER_MODE		MEDIAN		// Filtermodus (MEDIAN oder MEAN)

#define STD_FACTOR		2		// Faktor fuer die maximal erlaubte Abweichung von der Standardabweichung (MEAN-Filter)
#define MIN_STD			10		// Kleinste erlaubte Abweichung von der Standardabweichung (MEAN-Filter)

/********************************
 * EXT_CTRL-Einstellungen
 * ******************************/
#define EXT_CTRL		1		// Schaltet die externe Kontrolle an/aus

#define MAX_ROLL		1000		// Groeszter moeglicher Wert fuer roll		
#define MAX_PITCH		MAX_ROLL	// Groeszter moeglicher Wert fuer pitch

#define CONST_YAW		20		// Konstante Drehgeschwindigkeit 
#define HTM_DELAY_TIME		500		// Zeit zwischen Statuswechsel von HEAD_TO_MIDDLE zu HOLD_STILL in ms

/********************************
 * Regler-Einstellungen
 * ******************************/
#define IMAX			500

#define HOLD_STILL_ROLL_KP	4			// Verstaerkung
#define HOLD_STILL_ROLL_TN	0			// Nachtsellzeit
#define HOLD_STILL_ROLL_TV	2.5f			// Vorhaltzeit
#define HOLD_STILL_PITCH_KP	HOLD_STILL_ROLL_KP	// Verstaerkung
#define HOLD_STILL_PITCH_TN	HOLD_STILL_ROLL_TN	// Nachstellzeit
#define HOLD_STILL_PITCH_TV	HOLD_STILL_ROLL_TV	// Vorhaltzeit

#define HTM_YAW_KP		10
#define HTM_YAW_TN		0
#define HTM_YAW_TV		0
#define ANC_ROLL_KP		10
#define ANC_ROLL_TN		0
#define ANC_ROLL_TV		0.1f
#define ANC_PITCH_KP		ANC_ROLL_KP
#define ANC_PITCH_TN		ANC_ROLL_TN
#define ANC_PITCH_TV		ANC_ROLL_TV

/********************************
 * Status-Einstellungen
 * ******************************/
#define STILL_FAK_ROLL		5		// Faktor zur Bestimmung eines Stillstandes auf der Roll-Achse
#define STILL_FAK_PITCH		5		// Faktor zur Bestimmung eines Stillstandes auf der Pitch-Achse
#define CHECK_STILL_SPEED	100		// Verzoegerung zwischen zwei Pruefungen auf Stillstand in ms
#define CHECK_STILL_COUNT	5		// Zahl der aufeinanderfolgenden Zahl von Pruefungen bis ein Stillstand angenommen wird

#define HOLD_STILL_MAX_XACC	1000		// Maximal erlaubte Beschleunigung auf der x-Achse
#define HOLD_STILL_MAX_YACC	1000		// Maximal erlaubte Beschleunigung auf der y-Achse

#define ANCHOR_DISTANCE		30

/********************************
 * Log/Debug-Einstellungen
 * ******************************/
#define DEBUG_LEVEL		0		// Level fuer die Menge der Debug-Informationen
#define LOG			1		// Schaltet das Loggin an/aus
#define LOG_SPEED		1000		// Zeitspanne zwischen zwei Sicherungen der Log-Daten in ms
#define WARNINGS		1		// Schaltet Warnungen an/aus

/********************************
 * SLAM-Einstellungen
 * ******************************/
#define SLAM_CORR		1		// Schaltet die Positionskorrektur mit Hilfe einer Kreuzkorrelation ein/aus
#define SLAM_RESOLUTION		3		// Gibt die Auflösung des SLAM-Algorithmus in CM/Zelle an

/********************************
 * Config-Check
 * ******************************/
#include "config_check.h"

#endif