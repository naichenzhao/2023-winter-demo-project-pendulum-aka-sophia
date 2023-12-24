// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

void encoder_init();
void reset_encoder();

int get_count();
float get_dist();
float get_angle();

#endif