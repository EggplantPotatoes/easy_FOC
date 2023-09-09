#include "motor_function.h"
#include "PWM_Encoder.h"
#include "foc.h"

void Motor_Align(float ud) // 电机预定位，找电角度零点
{
	float theta = 0;
	DQ_Def align_dq;
	AlphaBeta_Def align_ab;
	SVPWM_Def svpwm_out;

	align_dq.d = ud;
	align_dq.q = 0.0f;

	theta = 0;
	inverseParkTransform(&align_dq, &align_ab, theta);
	SVPWM(&align_ab, &svpwm_out); // 电机停到电角度零点

	pwm_encoder.angle_rad_offset = pwm_encoder.duty * 2 * PI; // 获取当前机械角度
}

void motor_open_loop_control(float uq) // 开环控制电机
{

	float theta = 0.0f;
	DQ_Def open_loop_dq;
	AlphaBeta_Def open_loop_ab;
	SVPWM_Def svpwm_out;

	open_loop_dq.d = 0.0f;
	open_loop_dq.q = uq;

	Get_PWM_Encoder_Angles(); // 获取电角度
	theta = pwm_encoder.electronic_angle;
	inverseParkTransform(&open_loop_dq, &open_loop_ab, theta); // 电角度传入FOC SVPWM中
	SVPWM(&open_loop_ab, &svpwm_out);
}
