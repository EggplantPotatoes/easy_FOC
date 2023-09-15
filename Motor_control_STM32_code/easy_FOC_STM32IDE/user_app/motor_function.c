
#include "main.h"
#include "motor_function.h"
#include "PWM_Encoder.h"
#include "foc.h"
#include "user_config.h"
#include "current_sample.h"
#include "pid_control.h"
#include "debug.h"
#include "ADC_encoder.h"

extern TIM_HandleTypeDef htim2;

void motor_function_init(void)
{
	HAL_TIM_Base_Start_IT(&htim2); //打开定时器，1ms

}




void Motor_Align(float ud) // 电机预定位，找电角度零点
{
	float theta = 0;
	DQ_Def align_dq;
	AlphaBeta_Def align_ab;
	SVPWM_Def svpwm_out;
	float offset_1,offset_2,offset_3;
    float offset_sum;


	align_dq.d = ud;
	align_dq.q = 0.0f;

    /*找三次零点，取平均值，这样做的目的是，保证正转跟反转一致，否则正转和反转会有差别*/
	//第一次定位到0，
	theta = 0;
	inverseParkTransform(&align_dq, &align_ab, theta);
	SVPWM(&align_ab, &svpwm_out); // 电机停到电角度零点
	HAL_Delay(500);
	offset_1 = adc_encoder.adc_value/4096.0f * 2 * PI;

	//定位到+90°
	theta = 0.5*PI;
	inverseParkTransform(&align_dq, &align_ab, theta);
	SVPWM(&align_ab, &svpwm_out); // 电机停到电角度零点
	HAL_Delay(500);

	//第二次定位到0
	theta = 0;
	inverseParkTransform(&align_dq, &align_ab, theta);
	SVPWM(&align_ab, &svpwm_out); // 电机停到电角度零点
	HAL_Delay(500);
	offset_2 = adc_encoder.adc_value/4096.0f * 2 * PI;

	//定位到-90°
	theta = -0.5*PI;
	inverseParkTransform(&align_dq, &align_ab, theta);
	SVPWM(&align_ab, &svpwm_out); // 电机停到电角度零点
	HAL_Delay(500);

	//第三次定位到0
	theta = 0;
	inverseParkTransform(&align_dq, &align_ab, theta);
	SVPWM(&align_ab, &svpwm_out); // 电机停到电角度零点
	HAL_Delay(500);
	offset_3 = adc_encoder.adc_value/4096.0f * 2 * PI;
    //求平均
	offset_sum = offset_1 + offset_2 + offset_3;

	adc_encoder.angle_rad_offset = offset_sum/3.0f; // 获取当前机械角度

	align_dq.d = 0.0f;    //电机停止
	align_dq.q = 0.0f;
	theta = 0;
	inverseParkTransform(&align_dq, &align_ab, theta);
	SVPWM(&align_ab, &svpwm_out);

}

void motor_open_loop_control(float ud,float uq) // 开环控制电机
{

	float theta = 0.0f;
	DQ_Def open_loop_dq;
	AlphaBeta_Def open_loop_ab;
	SVPWM_Def svpwm_out;

	open_loop_dq.d = ud;
	open_loop_dq.q = uq;

//	Get_PWM_Encoder_Angles(); // 获取电角度
	get_ADC_encoder_data();
	theta = adc_encoder.electronic_angle;
	inverseParkTransform(&open_loop_dq, &open_loop_ab, theta); // 电角度传入FOC SVPWM中
	SVPWM(&open_loop_ab, &svpwm_out);
}


float pid_ud_set = 0;
float pid_uq_set = 0;
float pid_out_ud,pid_out_uq;
float Iq_set = 0.2f;

void motor_current_loop_control(float I_set) // 电流环
{
//	float pid_out_ud,pid_out_uq;

	FOC.Phase_Curr.Ua = current.Ia ;
	FOC.Phase_Curr.Ub = current.Ib ;
	FOC.Phase_Curr.Uc = current.Ic ;

	//转换
	clarkeTransform(&FOC.Phase_Curr,&FOC.AlphaBeta);
	FOC.theta = adc_encoder.electronic_angle;//FOC电角度赋值
	parkTransform(&FOC.AlphaBeta,FOC.theta,&FOC.DQ);

	pid_out_ud = PID_control(&curret_Id_pid,0.0,FOC.DQ.d);
	pid_out_uq = PID_control(&curret_Iq_pid,I_set,FOC.DQ.q);

	motor_open_loop_control(pid_out_ud,pid_out_uq);


}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t TimerCount = 0;
	TimerCount++;

    if (htim == (&htim2))
    {
//    	motor_open_loop_control(0.0f,0.1f);   //0.5ms控制周期  2KHz
    	motor_current_loop_control(Iq_set);
		if(TimerCount == 1000)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			TimerCount = 0;
		}
    }

}

