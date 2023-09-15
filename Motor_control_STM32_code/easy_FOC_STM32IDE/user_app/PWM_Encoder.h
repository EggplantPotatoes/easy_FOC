#ifndef __PWM_ENCODER_H
#define __PWM_ENCODER_H

#include "stm32g4xx_hal.h"



typedef struct
{
	uint32_t period;
	uint16_t frq;
	uint32_t high_time;
	float duty;
	float angle;	 // 0~360
	float angle_rad; // ������
	float electronic_angle;
	float angle_rad_offset;
} PWM_ENCODER_Def;

extern PWM_ENCODER_Def pwm_encoder;

void PWM_encoder_init(void);
void Get_PWM_Encoder_Angles(void);

void test_PWM_encoder(void);
#endif
