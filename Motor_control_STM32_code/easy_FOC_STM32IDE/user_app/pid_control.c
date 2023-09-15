/*
 * pid_control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: smile
 */
#include "pid_control.h"

PID_Def curret_Id_pid;
PID_Def curret_Iq_pid;


void PID_init(void)
{
	curret_Id_pid.Kp = 0.5f;
	curret_Id_pid.Ki = 0.001f;
	curret_Id_pid.Kd = 0;
	curret_Id_pid.Integral_max = 6.93f;


	curret_Iq_pid.Kp = curret_Id_pid.Kp;
	curret_Iq_pid.Ki = curret_Id_pid.Ki;
	curret_Iq_pid.Kd = curret_Id_pid.Kd;
	curret_Iq_pid.Integral_max = curret_Id_pid.Integral_max;

//	curret_Iq_pid.Kp = 0.0f;
//	curret_Iq_pid.Ki = 0.0f;
//	curret_Iq_pid.Kd = 0.0f;

}


float PID_control(PID_Def *PID,float set_Val,float Actual_Val)
{

	PID->SetVal=set_Val;
	PID->ActualVal=Actual_Val;
	PID->err=set_Val-Actual_Val;
	PID->Integral += PID->err;

	PID->Integral=LIMIT((PID->Integral),-PID->Integral_max,PID->Integral_max);

	PID->out=PID->Kp*PID->err + PID->Ki*(PID->Integral) + PID->Kd*(PID->err-PID->err_next);
	PID->err_next=PID->err;
    return PID->out;
}

