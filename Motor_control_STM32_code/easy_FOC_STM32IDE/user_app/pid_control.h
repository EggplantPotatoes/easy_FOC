/*
 * pid_control.h
 *
 *  Created on: Sep 12, 2023
 *      Author: smile
 */

#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#include "main.h"
#include "stdlib.h"
#include <math.h>


#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

#define INTEGRAL_MAX   6.930f    //sqrt(3)/3  * Udc mv
#define POSITION_ACCURACY   2.0f   //sqrt(3)/3  * Udc mv
typedef struct
{
    float SetVal;            //定义设定值
    float ActualVal;        //定义实际值
    float out;                //定义输出值
    float err;                //定义偏差值
    float err_next;            //定义上一个偏差值
    float err_last;            //定义最上前的偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数

    float Integral;
    float Integral_max;
    uint8_t dirc;				//定义方向
    uint8_t samp_time;		    //取样时间（每秒多少次）
}PID_Def;


extern PID_Def curret_Id_pid;
extern PID_Def curret_Iq_pid;
extern PID_Def speed_loop_pid;
extern PID_Def position_loop_pid;

extern float pid_out_ud;
extern float pid_out_uq;
extern float pid_ud_set;
extern float pid_uq_set;

void PID_init(void);
float PID_control(PID_Def *PID,float set_Val,float Actual_Val);
float position_PID_control(PID_Def *PID,float set_Val,float Actual_Val);

#endif /* PID_CONTROL_H_ */
