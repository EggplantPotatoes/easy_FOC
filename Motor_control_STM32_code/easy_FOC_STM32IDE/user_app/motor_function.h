#ifndef __MOTOR_FUNCTION_H
#define __MOTOR_FUNCTION_H

#include "stm32g4xx_hal.h"
#define OPEN_LOOP_MODE  1
#define CUREENT_LOOP_MODE  2
#define SPEED_LOOP_MODE 3
#define POSITION_LOOP_MODE  4

typedef struct
{
 uint8_t mode;
 float ramp_target_value;
 float ramp_acc_value;
 float ramp_acc_time;

 float open_value;
 float current_value;

 float current_angle;
 float angle_err;
 float angle_old;
 float speed;
 float speed_filter_win;
 float speed_set;
// float speed_filter_lp;

 float position_set_value;
 float position_old_value;
 float position_finish;
}MOTOR_CONTROL;

extern MOTOR_CONTROL m_c ;
void motor_function_init(void);
void Motor_Align(float ud);
void motor_open_loop_control(float ud,float uq);

#endif



