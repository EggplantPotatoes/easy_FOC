#include "motor_bsp.h"
#include "main.h"
#include "user_config.h"

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void PWM_ADC_init(void)
{
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);

	//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, PWM_PERIOD - 1); // TIM1通道4的PWM配置  关联ADC采样时间 // 在PWM波的正中间采样
}

void set_PWM_value(uint16_t pwm_u, uint16_t pwm_v, uint16_t pwm_w)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_u);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_v);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_w);
}

// uint16_t PA1_adc_value,PB11_adc_value,PA0_adc_value,PA7_adc_value;
// void adc_sample_task(void)
//{
//	PA1_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
//	PB11_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
//	PA0_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
//	PA7_adc_value = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);

//}
