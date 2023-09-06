#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32g4xx_hal.h"
#include "main.h"

//BSP 相关

#define POWER_BUS    12.0  //总线电压  


//控制相关
#define PWM_FREQUENCY 30000
#define PWM_PERIOD TIM_CLK_MHz*1000000/PWM_FREQUENCY/2    //PWM 频率



#endif


