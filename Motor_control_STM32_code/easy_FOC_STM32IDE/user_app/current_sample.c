/*
 * current_sample.c
 *
 *  Created on: Sep 8, 2023
 *      Author: smile
 */


#include "current_sample.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


uint16_t PA1_adc_value, PB11_adc_value, PA0_adc_value, PA7_adc_value;
/* USER CODE BEGIN 4 */

/*重构回调函数*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		PA1_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		PB11_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
		PA0_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

	}
	if(hadc->Instance == ADC2)
	{
		PA7_adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	}
}

/* USER CODE END 4 */
