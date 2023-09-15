/*
 * ADC_encoder.h
 *
 *  Created on: Sep 15, 2023
 *      Author: smile
 */

#ifndef ADC_ENCODER_H_
#define ADC_ENCODER_H_

#include "main.h"

typedef struct
{
	uint16_t adc_value;
	float angle;
	float angle_rad; // ������
	float electronic_angle;
	float angle_rad_offset;
} ADC_ENCODER_Def;

extern ADC_ENCODER_Def adc_encoder;

void get_ADC_encoder_data(void);
void test_adc_encoder(void);
#endif /* ADC_ENCODER_H_ */
