#include "test.h"
#include "main.h"

#define PWM_PERIOD TIM_CLK_MHz*1000000/PWM_FREQUENCY/2

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void test_PWM_ADC_init(void)  
{
 
 HAL_TIM_Base_Start(&htim1);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
 HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
 HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
 HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0.2*PWM_PERIOD); 
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0.2*PWM_PERIOD); 
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0); 
	
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
	
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,PWM_PERIOD-1);  //TIM1通道4的PWM配置  关联ADC采样时间 // 在PWM波的正中间采样
}


void motor_six_steps(uint8_t num) //六步换相测试程序
{ 
	if (num >= 6)
	{
		num = 0;
	}
	
	switch (num)
	{
		case 0:	 //UV  U_UP ON  V_DOWN ON
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0.5*PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);	
		break;
		
		case 1: //UW U_UP ON  W_DOWN ON
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0.5*PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);	
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		
		break;
		
		case 2:	 //VW  V_UP ON  W_DOWN ON
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0.5*PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		break;
		
		case 3:	//VU V_UP ON  U_DOWN ON
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0.5*PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		break;
	
		case 4:	 //WU  W_UP ON  U_DOWN ON
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0.5*PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		break;
		
		case 5://WV W_UP ON  V_DOWN ON
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0.5*PWM_PERIOD);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

		break;
	}

}

void test_pwm_control_motor(void)
{
  uint8_t i;
	for(i=0;i<6;i++)
	{
		motor_six_steps(i);
		HAL_Delay(5);
	}
}

uint16_t PA1_adc_value,PB11_adc_value,PA0_adc_value,PA7_adc_value;
void test_get_ADC_value(void)
{
	PA1_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
	PB11_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
	PA0_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
	PA7_adc_value = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);

}




