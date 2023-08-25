#include "test.h"
#include "main.h"

#define PWM_PERIOD TIM_CLK_MHz*1000000/PWM_FREQUENCY/2

extern TIM_HandleTypeDef htim1;

void test_PWM(void)  
{

 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
 HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
 HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
 HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0.2*PWM_PERIOD); 
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0.2*PWM_PERIOD); 
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0); 
}


void test_pwm_control_motor(void) //六步换相测试程序
{ 
	  //UV  U_H_UP ON  V_H_DOWN ON
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,PWM_PERIOD);
	  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
    HAL_Delay(5);
	  //UW
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,PWM_PERIOD);
	  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
//	  //VW
//    VH_WL_HPwmLOn();
//    Delay_ms(3);
//	  //VU
//    VH_UL_HPwmLOn();
//    Delay_ms(3);
//	  //WU
//    WH_UL_HPwmLOn();
//    Delay_ms(3);
//	  //WV
//    WH_VL_HPwmLOn();
//    Delay_ms(5);


}
