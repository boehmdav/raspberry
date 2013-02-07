#ifndef DA_SLAM_H
#define DA_SLAM_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "defines.h"

void slam_draw_image(const char* file);			// Speichert das derzeitige Grid in der Datei file (Format: *.ppm)

void slam_init(unsigned short heading, unsigned int tv_msec);	// Initialisierung
void slam_insert_measurement(unsigned short measurement, 	// Fügt einen Messwert in ein temporäres Grid ein
			     unsigned short heading);		// Die angenommene Position ist dabei die, die beim lezten Aufruf von slam_refresh_pos
								// erzeugt wurde

void slam_insert_acc(int xa, int ya, unsigned short heading, 	// Speichert Beschleunigungswerte bis zum Aufruf von slam_refresh_pos
		     unsigned int tv_msec);
void slam_refresh_pos(unsigned short tv_msec);			// Berechnet an Hand der Beschleunigungswerte eine neue Position und fügt das temporäre Grid an der neuen Position ein

#endif