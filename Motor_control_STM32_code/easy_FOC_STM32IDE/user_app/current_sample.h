/*
 * current_sample.h
 *
 *  Created on: Sep 8, 2023
 *      Author: smile
 */

#ifndef CURRENT_SAMPLE_H_
#define CURRENT_SAMPLE_H_

#include "stm32g4xx_hal.h"

typedef struct
{
    float Ia; // Phase A current
    float Ib; // Phase B current
    float Ic; // Phase C current
    float Vbus;

    uint16_t ADC_a; // Phase A Voltage
    uint16_t ADC_b; // Phase B Voltage
    uint16_t ADC_c; // Phase C Voltage
    uint16_t ADC_bus; // Phase bus Voltage
    uint16_t ADC_Ia_offset; // 电流零点偏置
    uint16_t ADC_Ib_offset; // 电流零点偏置
    uint16_t ADC_Ic_offset; // 电流零点偏置
} CURRENT_Def;

extern CURRENT_Def current;

void GetCurrentOffset(void);

#endif /* CURRENT_SAMPLE_H_ */


