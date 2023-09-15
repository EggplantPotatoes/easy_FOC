/*
 * current_sample.c
 *
 *  Created on: Sep 8, 2023
 *      Author: smile
 */


#include "current_sample.h"
#include "user_config.h"
#include "ADC_encoder.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

CURRENT_Def current;

/* USER CODE BEGIN 4 */

/*重构回调函数*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		current.ADC_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		current.ADC_b = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
		current.ADC_bus = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

		current.Ia =  ADC_V_K*(current.ADC_a-current.ADC_Ia_offset)/AM_GAIN/R_SENSE ;
		current.Ib = ADC_V_K*(current.ADC_b-current.ADC_Ib_offset)/AM_GAIN/R_SENSE ;
		current.Vbus = ADC_V_K*current.ADC_bus/VBUS_k;

	}
	if(hadc->Instance == ADC2)
	{
		current.ADC_c = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		current.Ic = ADC_V_K*(current.ADC_c-current.ADC_Ic_offset)/AM_GAIN/R_SENSE;
		adc_encoder.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	}

}

void GetCurrentOffset(void)
{
 uint16_t i;
 uint16_t a_offset,b_offset,c_offset;
 uint32_t a_offset_sum,b_offset_sum,c_offset_sum;
 a_offset_sum = 0;
 b_offset_sum = 0;
 c_offset_sum = 0;
 for(i=0;i<1000;i++)
 {
	 a_offset = current.ADC_a;
	 b_offset = current.ADC_b;
	 c_offset = current.ADC_c;
	 a_offset_sum = a_offset_sum + a_offset;
	 b_offset_sum = b_offset_sum + b_offset;
	 c_offset_sum = c_offset_sum + c_offset;
	 HAL_Delay(1);
 }
 current.ADC_Ia_offset = a_offset_sum/1000;
 current.ADC_Ib_offset = b_offset_sum/1000;
 current.ADC_Ic_offset = c_offset_sum/1000;
}
/* USER CODE END 4 */
