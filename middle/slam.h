#ifndef DA_SLAM_H
#define DA_SLAM_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "defines.h"

//void slam_insert(struct position pos, unsigned char val);
//unsigned char get(struct position pos);
//unsigned char get(int x, int y);

//struct position line_value_x(float m, float n, int x);
//struct position line_value_y(float m, float n, int y);

//void draw_line(struct position start_pos, struct position target_pos, unsigned char start_val, unsigned char path_val, unsigned char target_val);
//void draw_line_tmp(struct position start_pos, struct position target_pos, unsigned char start_val, unsigned char path_val, unsigned char target_val);

//unsigned int max_y_o();
//unsigned int max_y_w();


void slam_draw_image(const char* file);

void slam_init(unsigned short heading, unsigned int tv_msec);
void slam_insert_measurement(unsigned short measurement, unsigned short heading);

void slam_insert_acc(int xa, int ya, unsigned short heading, unsigned int tv_msec);
void slam_refresh_pos(unsigned short tv_msec);

#endif