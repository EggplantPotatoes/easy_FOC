#include "BEMF.h"
#include "user_config.h"

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


void U_UP_PWM_V_DOWN_ON(void)  //UV  U_UP ON  V_DOWN ON
{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);	

}

void U_UP_PWM_W_DOWN_ON(void)  //UW U_UP ON  W_DOWN ON
{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);	
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

}

void V_UP_PWM_W_DOWN_ON(void)  //VW  V_UP ON  W_DOWN ON
{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

}

void V_UP_PWM_U_DOWN_ON(void)  //VU V_UP ON  U_DOWN ON
{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

}

void W_UP_PWM_U_DOWN_ON(void)  //WU  W_UP ON  U_DOWN ON
{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

}

void W_UP_PWM_V_DOWN_ON(void)  //WV W_UP ON  V_DOWN ON
{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

}

