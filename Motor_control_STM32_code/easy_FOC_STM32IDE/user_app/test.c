#include "test.h"
#include "main.h"
#include "user_config.h"

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart2;

void test_PWM_ADC_init(void)
{

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0.2*PWM_PERIOD);
	//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0.2*PWM_PERIOD);
	//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);

	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);

	//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, PWM_PERIOD - 1); // TIM1通道4的PWM配置  关联ADC采样时间 // 在PWM波的正中间采样
}

void motor_six_steps(uint8_t num) // 六步换相测试程序
{
	//	TIM_OC_InitTypeDef sConfigOC;
	if (num >= 6)
	{
		num = 0;
	}

	switch (num)
	{
	case 0: // UV  U_UP ON  V_DOWN ON
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		break;

		//		case 1: //UW U_UP ON  W_DOWN ON
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0.5*PWM_PERIOD);
		//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		//
		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
		//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		//
		//		break;
		//
		//		case 2:	 //VW  V_UP ON  W_DOWN ON
		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		//
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0.5*PWM_PERIOD);
		//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		//
		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
		//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		//		break;
		//
		//		case 3:	//VU V_UP ON  U_DOWN ON
		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
		//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		//
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0.5*PWM_PERIOD);
		//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		//
		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		//		break;
		//
		//		case 4:	 //WU  W_UP ON  U_DOWN ON
		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
		//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		//
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0.5*PWM_PERIOD);
		//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		//		break;
		//
		//		case 5://WV W_UP ON  V_DOWN ON
		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		//
		//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD);
		//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		//
		//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD);
		//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

		//		break;
	}
}

void test_pwm_control_motor(void)
{
	uint8_t i = 0;
	//	for(i=0;i<6;i++)
	//	{
	motor_six_steps(i);
	HAL_Delay(5);
	//	}
}

//uint16_t PA1_adc_value, PB11_adc_value, PA0_adc_value, PA7_adc_value;
//void test_get_ADC_value(void)
//{
//	PA1_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//	PB11_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
//	PA0_adc_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
//	PA7_adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//}

// uint8_t StaMessages[4] = {0x11,0x22,0x33,0x44};
// uint8_t RxBuffer[20];

// void test_uart_init(void)
//{

//    HAL_UART_Transmit_IT(&huart2 ,(uint8_t*)StaMessages,sizeof(StaMessages));
//	  HAL_UART_Receive_IT(&huart2,(uint8_t*)RxBuffer,1);//调用中断接收函数，接收长度设为1，接收一个字节进一次中断
//}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//     UNUSED(huart);
//     HAL_UART_Transmit(&huart2,(uint8_t*)RxBuffer,1,0xFFFF);  //发送接收到的数据
//     HAL_UART_Receive_IT(&huart2,(uint8_t*)RxBuffer,1);  //再开启接收中断（因为里面中断只会触发一次，因此需要再次开启）

//}
