#ifndef __MOTOR_BSP_H
#define __MOTOR_BSP_H

#include "stm32g4xx_hal.h"



void PWM_ADC_init(void);
void set_PWM_value(uint16_t pwm_u,uint16_t pwm_v,uint16_t pwm_w);

#endif



