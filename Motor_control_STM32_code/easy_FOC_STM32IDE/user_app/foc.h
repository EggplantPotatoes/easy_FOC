#ifndef __FOC_H
#define __FOC_H

#include "stm32g4xx_hal.h"

typedef struct
{
    float Ia; // Phase A current
    float Ib; // Phase B current
    float Ic; // Phase C current

    float Ua; // Phase A Voltage
    float Ub; // Phase B Voltage
    float Uc; // Phase C Voltage

} ABC_Def;

typedef struct
{
    float alpha; // alpha-axis current
    float beta;  // beta-axis current
} AlphaBeta_Def;

typedef struct
{
    float d; // d-axis current
    float q; // q-axis current
} DQ_Def;

typedef struct
{
    float U_alpha; // alpha-axis current
    float U_beta;  // beta-axis current
    int sector;

    float u1;
    float u2;
    float u3;

    float ta;
    float tb;
    float tc;

    float Ts; // 总控制时间
    float t0;
    float t1;
    float t2;
    float t3;
    float t4;
    float t5;
    float t6;
    float t7;

} SVPWM_Def;

typedef struct
{
//    float U_d;
//    float U_q;
    float theta;
//    float U_alpha;
//    float U_bate;
    ABC_Def Phase_Curr;
    AlphaBeta_Def AlphaBeta;
    DQ_Def DQ;
} FOC_Def;

extern FOC_Def FOC;

void FOC_hardware_init(void);
void FOC_software_init(void);

void clarkeTransform(ABC_Def *abc, AlphaBeta_Def *alphaBeta);
void parkTransform(const AlphaBeta_Def *alphaBeta, float angle, DQ_Def *dq);
void inverseParkTransform(DQ_Def *dq, AlphaBeta_Def *alphaBeta, float angle);
void SVPWM(AlphaBeta_Def *U_alphaBeta, SVPWM_Def *svpwm);

void svpwm_test(void);

#endif
