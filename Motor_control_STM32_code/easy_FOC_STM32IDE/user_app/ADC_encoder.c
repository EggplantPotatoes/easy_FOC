/*
 * ADC_encoder.c
 *
 *  Created on: Sep 15, 2023
 *      Author: smile
 */
#include "ADC_encoder.h"
#include "user_config.h"
#include "foc.h"
#include <stdio.h>
#include <math.h>
#include "debug.h"

ADC_ENCODER_Def adc_encoder;

void get_ADC_encoder_data(void)
{
	float angle_temp;

	adc_encoder.angle = adc_encoder.adc_value/4096.0f * 360.0f;
	adc_encoder.angle_rad = adc_encoder.adc_value/4096.0f * 2 * PI;
//	adc_encoder.angle_rad_offset = 6.055f;					   // 手动找到的零点，后续通过函数调用获得
	if (adc_encoder.angle_rad >= adc_encoder.angle_rad_offset) // 减去零点偏置
	{
		adc_encoder.angle_rad = adc_encoder.angle_rad - adc_encoder.angle_rad_offset;
	}
	else
	{
		adc_encoder.angle_rad = 2 * PI - adc_encoder.angle_rad_offset + adc_encoder.angle_rad;
	}
	angle_temp =  adc_encoder.angle_rad * MOTOR_POLE;
	adc_encoder.electronic_angle = fmod(angle_temp,2 * PI);

}

void test_adc_encoder(void)
{
	float theta = 0;
	DQ_Def test_dq;
	AlphaBeta_Def test_ab;
	SVPWM_Def svpwm_out;

	test_dq.d = 0.0f;
	test_dq.q = 0.2f;

	for (theta = 0; theta < 6.2831853f; theta += 0.375f)
	{
//		pwm_encoder.angle = pwm_encoder.duty * 360.0f;	   // 计算角度0~360
//		pwm_encoder.angle_rad = pwm_encoder.duty * 2 * PI; // 计算角度，弧度制
		get_ADC_encoder_data(); // 获取电角度
		vofa_JustFloat_output(adc_encoder.angle, adc_encoder.angle_rad, adc_encoder.electronic_angle, adc_encoder.adc_value);
		inverseParkTransform(&test_dq, &test_ab, theta);
		SVPWM(&test_ab, &svpwm_out); // 电机会转动
	}
}

