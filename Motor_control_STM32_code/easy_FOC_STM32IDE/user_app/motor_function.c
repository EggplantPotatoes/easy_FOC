
#include "main.h"
#include "motor_function.h"
#include "PWM_Encoder.h"
#include "foc.h"
#include "user_config.h"
#include "current_sample.h"
#include "pid_control.h"
#include "debug.h"
#include "ADC_encoder.h"
#include "motor_bsp.h"
#include "data_filter.h"

extern TIM_HandleTypeDef htim2;

MOTOR_CONTROL m_c;

void motor_function_init(void)
{
	HAL_TIM_Base_Start_IT(&htim2); //打开定时器，1ms
//	m_c.mode = SPEED_LOOP_MODE;
	m_c.mode = POSITION_LOOP_MODE;
}


void motor_stop(void)
{
	set_PWM_value(0,0,0);
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

//	align_dq.d = 0.0f;    //电机停止
//	align_dq.q = 0.0f;
//	theta = 0;
//	inverseParkTransform(&align_dq, &align_ab, theta);
//	svpwm_out.ta = 0.0f;
//	svpwm_out.tb = 0.0f;
//	svpwm_out.tc = 0.0f;
//	SVPWM(&align_ab, &svpwm_out);
	motor_stop();

}

void motor_open_loop_control(float ud,float uq) // 开环控制电机
{

	float theta = 0.0f;
	DQ_Def open_loop_dq;
	AlphaBeta_Def open_loop_ab;
	SVPWM_Def svpwm_out;

	open_loop_dq.d = ud;
	open_loop_dq.q = uq;

	if((ud==0.0f)&&(uq==0.0f))
	{
		motor_stop();
		return;
	}
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
float motor_ramp(float t_value,float acc_time) //电机爬坡  爬坡时间s
{
	static float actual_value = 0.0f;
	float ram_acc_value ;
	float delta;
	delta = acc_time/CONTROL_PERIOD;
	ram_acc_value = fabs(t_value-actual_value)*delta;

	if((t_value>0.0f))
	{
		actual_value += ram_acc_value;
		if(actual_value>t_value)
		{
		  actual_value = t_value;
		}
	}
	else if(t_value<0.0f)
	{
		actual_value -= ram_acc_value;
		if(actual_value<t_value)
		{
		  actual_value = t_value;

		}
	}
	else
	{
		actual_value = 0;
	}
   return actual_value;
}

void motor_speed_loop_control(float speed_set) // 速度环控制
{
	float delta;
	float speed_pid_out,speed_temp;

	if(speed_set==0.0f)
	{
		motor_stop();
		return;
	}
   //计算速度
	get_ADC_encoder_data();
	m_c.current_angle = adc_encoder.angle;
	delta = m_c.current_angle - m_c.angle_old;  //两次采集的角度差值
	m_c.angle_err = delta;
	if(delta<-180.0f)  //处理0，360°角度突变点  正转
	{
		m_c.angle_err = delta + 360.0f;
	}
	if(delta>180.0f)  //处理0，360°角度突变点  反转
	{
		m_c.angle_err = 360.0f - delta;

	}
	m_c.speed = m_c.angle_err*ANGLE_SPEED_RATIO;  //转换系数
	m_c.angle_old = m_c.current_angle;
	//滤波
	m_c.speed_filter_win = window_filter(m_c.speed,window_speed,WIN_SIZE);

	speed_pid_out = PID_control(&speed_loop_pid,speed_set,m_c.speed_filter_win);
//	speed_temp = motor_ramp(speed_pid_out,0.2f);
	motor_open_loop_control(0.0f,speed_pid_out);

}


void motor_position_loop_control(float position_set) // 位置环控制
{
	float position_pid_out;
	float delta;
   //计算角度增量
	get_ADC_encoder_data();
	m_c.current_angle = adc_encoder.angle;
	delta = m_c.current_angle - m_c.angle_old;  //两次采集的角度差值
	m_c.angle_err = delta;
	if(delta<-180.0f)  //处理0，360°角度突变点  正转
	{
		m_c.angle_err = delta + 360.0f;
	}
	if(delta>180.0f)  //处理0，360°角度突变点  反转
	{
		m_c.angle_err = 360.0f - delta;
	}
	m_c.position_set_value = m_c.position_set_value - m_c.angle_err; //计算剩余角度
	m_c.angle_old = m_c.current_angle;
	position_pid_out = position_PID_control(&position_loop_pid,position_set,m_c.angle_err);
	position_pid_out = LIMIT(position_pid_out,-0.5f,0.5f);  //限制转速
	motor_open_loop_control(0.0f,position_pid_out);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t TimerCount = 0;
	TimerCount++;

    if (htim == (&htim2))
    {
    	if(m_c.mode==OPEN_LOOP_MODE)
    	{
    		m_c.ramp_acc_value = motor_ramp(m_c.open_value,OPEN_ACC_TIME);  //爬坡，防止电机急停急起
    		motor_open_loop_control(0.0f,m_c.ramp_acc_value);
    		motor_speed_loop_control(0.0f);
    	}
    	else if(m_c.mode==CUREENT_LOOP_MODE)
    	{
    		m_c.ramp_acc_value = motor_ramp(m_c.current_value,OPEN_ACC_TIME);  //爬坡，防止电机急停急起
    		motor_current_loop_control(m_c.ramp_acc_value);
    	}
    	else if(m_c.mode==SPEED_LOOP_MODE)
    	{
    		motor_speed_loop_control(m_c.speed_set);
    	}
    	else if(m_c.mode==POSITION_LOOP_MODE)
    	{
    		motor_position_loop_control(m_c.position_set_value);
    	}
//    	motor_open_loop_control(0.0f,0.0f);   //0.5ms控制周期  2KHz
//    	motor_current_loop_control(Iq_set);

		if(TimerCount == 1000)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			TimerCount = 0;
		}
    }

}

