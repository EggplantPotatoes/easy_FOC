#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32g4xx_hal.h"
#include "main.h"

#define PI 3.14159265f

// 电机相关
#define MOTOR_POLE 7

// BSP 相关



//母线电压采集电阻分压电路
#define POWER_BUS 12.0 // 总线电压
#define ADC_V_K   0.0008  //3.3/4096.0  // ADC 与电压转换系数
#define VBUS_R1   180  //KΩ
#define VBUS_R2   12  //KΩ
#define VBUS_k    0.0625  //VBUS_R2/(VBUS_R1 + VBUS_R2)  //分压系数

//电流采样相关

#define AM_GAIN   1.53 //放大倍数
#define R_SENSE   0.33  //Ω 采样电阻


// 控制相关
#define PWM_FREQUENCY 30000
#define PWM_PERIOD TIM_CLK_MHz * 1000000 / PWM_FREQUENCY / 2 // PWM 频率
// 控制相关
#define PWM_FREQUENCY 30000
#define PWM_PERIOD TIM_CLK_MHz * 1000000 / PWM_FREQUENCY / 2 // PWM 频率
#define CONTROL_FRE   2000.0f //FOC 控制频率 Hz
#define CONTROL_PERIOD   1/CONTROL_FRE //FOC 控制频率 Hz
#define OPEN_ACC_TIME    1.0f//FOC 控制频率 Hz
#define SPEED_ACC_TIME    1.0f//FOC 控制频率 Hz
#define ANGLE_SPEED_RATIO    CONTROL_FRE*60/360  // 角度转换成速度系数

#endif

