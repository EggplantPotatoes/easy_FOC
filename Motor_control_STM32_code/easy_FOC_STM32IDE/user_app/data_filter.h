/*
 * data_filter.h
 *
 *  Created on: Sep 16, 2023
 *      Author: smile
 */

#ifndef DATA_FILTER_H_
#define DATA_FILTER_H_

#include "main.h"

#define WIN_SIZE 50
#define Pi 3.1415926f
#define HP_CUT_FRQ 5
#define SAMPLE_RATE 2000.0f


extern float window_speed[WIN_SIZE];



float window_filter(float data, float *buf, uint8_t len);
void high_pass_filter(float in, float *out);
void LowPassFilter_RC(float Vi, float *Vo);

#endif /* DATA_FILTER_H_ */
