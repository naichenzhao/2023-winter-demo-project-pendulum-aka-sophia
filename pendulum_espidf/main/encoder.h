// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

void encoder_init();
void reset_encoder1();
void reset_encoder2();

int get_count1();
int get_count2();

float get_dist();
float get_angle();

#endif