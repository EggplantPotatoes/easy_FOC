#include "PWM_Encoder.h"
#include "user_config.h"
#include "foc.h"
#include "debug.h"



extern TIM_HandleTypeDef htim3;

PWM_ENCODER_Def pwm_encoder;


void PWM_encoder_init(void)
{
  
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
		if(htim->Instance==TIM3)
		{
				if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)                 
				{
					pwm_encoder.period=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1); 
					pwm_encoder.frq=pwm_encoder.period*0.1f;              
					pwm_encoder.duty=((float)pwm_encoder.high_time/(float)pwm_encoder.period); 
				}
				if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)                 
				{
					pwm_encoder.high_time=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);  
				}				
	  }

}

void test_PWM_encoder(void)
{
    float theta = 0;
    DQ_Def test_dq;
    AlphaBeta_Def test_ab;
    SVPWM_Def svpwm_out;
	
    test_dq.d = 0.0f;
    test_dq.q = 0.2f;
    
    for (theta = 0; theta < 6.2831853f; theta += 0.275f)
     {
			  pwm_encoder.angle = pwm_encoder.duty * 360.0f; //����Ƕ�0~360
        pwm_encoder.angle_rad = pwm_encoder.duty*2*PI; //����Ƕȣ�������
			 
			  vofa_JustFloat_output(pwm_encoder.angle,pwm_encoder.angle_rad,0.0f,0.0f);
        inverseParkTransform(&test_dq,&test_ab,theta);
        SVPWM(&test_ab,&svpwm_out);		  //�����ת�� 	
    }

}


//��ȡ�������Ļ�е�Ƕȣ���ʵ������Ƕ�
//��ȡ��ǶȽǶȣ�FOC��Ƕ�
void Get_PWM_Encoder_Angles(void)
{
  pwm_encoder.angle = pwm_encoder.duty * 360.0f;
  pwm_encoder.angle_rad = pwm_encoder.duty*2*PI;
	pwm_encoder.angle_rad_offset = 6.045f; //�ֶ��ҵ�����㣬����ͨ���������û��
	if(pwm_encoder.angle_rad>=pwm_encoder.angle_rad_offset) //��ȥ���ƫ��
	{
		 pwm_encoder.angle_rad = pwm_encoder.angle_rad - pwm_encoder.angle_rad_offset;
	}
	else
	{
		 pwm_encoder.angle_rad = 2*PI - pwm_encoder.angle_rad_offset + pwm_encoder.angle_rad;
	}	
	pwm_encoder.electronic_angle = pwm_encoder.angle_rad*MOTOR_POLE;
}

