#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32g4xx_hal.h"
#include "main.h"

//BSP ���

#define POWER_BUS    12.0  //���ߵ�ѹ  


//�������
#define PWM_FREQUENCY 30000
#define PWM_PERIOD TIM_CLK_MHz*1000000/PWM_FREQUENCY/2    //PWM Ƶ��



#endif


