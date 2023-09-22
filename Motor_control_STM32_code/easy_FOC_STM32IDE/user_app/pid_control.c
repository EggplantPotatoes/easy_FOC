/*
 * pid_control.c
 *
 *  Created on: Sep 12, 2023
 *      Author: smile
 */
#include "pid_control.h"

PID_Def curret_Id_pid;
PID_Def curret_Iq_pid;
PID_Def speed_loop_pid;
PID_Def position_loop_pid;




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

	speed_loop_pid.Kp = 0.00255f;
	speed_loop_pid.Ki = 0.00015f;
	speed_loop_pid.Kd = 0;
	speed_loop_pid.Integral_max = 100.0f;

	position_loop_pid.Kp = 0.001f;
	position_loop_pid.Ki = 0.0003f;
	position_loop_pid.Kd = 0.0001;
	position_loop_pid.Integral_max = 100.0f;

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


//位置式PID
float position_PID_control(PID_Def *PID,float set_Val,float Actual_Val)
{

	float inter;
	PID->SetVal= set_Val;
	PID->ActualVal=Actual_Val;

	PID->err=set_Val-Actual_Val;
	if(fabs(PID->err)>=POSITION_ACCURACY) //位置精度
	{
        inter = PID->err;
        PID->Integral += inter;
        PID->Integral = LIMIT((PID->Integral),-100.0f,100.0f);
        PID->out = PID->Kp*PID->err + PID->Ki*(PID->Integral) + PID->Kd*(PID->err - PID->err_next);
        PID->err_next = PID->err;

	}
	else
	{
		PID->out = 0.0f;
	}
    return PID->out;
}

