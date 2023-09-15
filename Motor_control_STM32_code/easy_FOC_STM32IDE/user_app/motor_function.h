#ifndef __MOTOR_FUNCTION_H
#define __MOTOR_FUNCTION_H

#include "stm32g4xx_hal.h"


extern float Iq_set ;

void motor_function_init(void);
void Motor_Align(float ud);
void motor_open_loop_control(float ud,float uq);

#endif



