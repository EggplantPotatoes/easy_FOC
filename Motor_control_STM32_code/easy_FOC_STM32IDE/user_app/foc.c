#include "foc.h"
#include "motor_bsp.h"
#include "user_config.h"
#include <stdio.h>
#include <math.h>
#include "debug.h"
#include "PWM_Encoder.h"
FOC_Def FOC;

void FOC_hardware_init(void)
{
    PWM_ADC_init();
}

void FOC_software_init(void)
{
}

/**********************************************************************************************************
Clarke变换 输入三相电流，输出alpha，bate电流
Iα = Ia
Iβ = (Ia + 2Ib) / sqrt(3)
**********************************************************************************************************/
void clarkeTransform(ABC_Def *abc, AlphaBeta_Def *alphaBeta)
{
    alphaBeta->alpha = abc->Ua;
    alphaBeta->beta = (abc->Ua + 2 * abc->Ub) * 0.57735027f;
}

/**********************************************************************************************************
Park变换，输入电角度、Ialpha和Ibeta，经过Park变换得到Iq、Id
Id = Iα · cosθ + Iβ · sinθ
Iq = Iα · sinθ + Iβ · cosθ
**********************************************************************************************************/
void parkTransform(const AlphaBeta_Def *alphaBeta, float angle, DQ_Def *dq)
{
    float sinAngle = sin(angle);
    float cosAngle = cos(angle);

    dq->d = cosAngle * alphaBeta->alpha + sinAngle * alphaBeta->beta;
    dq->q = -sinAngle * alphaBeta->alpha + cosAngle * alphaBeta->beta;
}

/**********************************************************************************************************
park逆变换，输入Uq、Ud得到Ualpha、Ubeta
Uα = Ud · cosθ - Uq · sinθ
Uβ = Ud · sinθ + Uq · cosθ
**********************************************************************************************************/
void inverseParkTransform(DQ_Def *dq, AlphaBeta_Def *alphaBeta, float angle)
{
    float cosAngle = cos(angle);
    float sinAngle = sin(angle);

    alphaBeta->alpha = dq->d * cosAngle - dq->q * sinAngle;
    alphaBeta->beta = dq->d * sinAngle + dq->q * cosAngle;
}

/**********************************************************************************************************
反Clarke变换，输入Ualpha、Ubeta，得到Ua,Ub,Uc
Ua = Uα
Ub = -1/2 * Uα + sqrt(3)/2  * Uβ
Ub = -1/2 * Uα - sqrt(3)/2  * Uβ
**********************************************************************************************************/
// void inverseClarkeTransform(AlphaBeta_Def *abVoltage, ABC_Def *abc)
// {
//     abc->U_u = abVoltage->alpha;
//     abc->U_v = -0.5 * abVoltage->alpha + 0.8660254 * abVoltage->beta;
//     abc->U_w = -0.5 * abVoltage->alpha - 0.8660254 * abVoltage->beta;
// }

/**********************************************************************************************************
将坐标变换中的反Park变换得到的 Valpha 、Vbeta 转换六路PWM输出。

**********************************************************************************************************/
void SVPWM(AlphaBeta_Def *U_alphaBeta, SVPWM_Def *svpwm)
{
    float sum;
    float k_svpwm;

    svpwm->Ts = 1.0f; // SVPWM的采样周期

    svpwm->U_alpha = U_alphaBeta->alpha;
    svpwm->U_beta = U_alphaBeta->beta;

	  // step1 计算u1、u2和u3
    // 计算SVPWM算法中的三个控制电压u1、u2和u3
    svpwm->u1 = U_alphaBeta->beta;
    svpwm->u2 = -0.8660254f * U_alphaBeta->alpha - 0.5f * U_alphaBeta->beta; // sqrt(3)/2 ≈ 0.86603
    svpwm->u3 = 0.8660254f * U_alphaBeta->alpha - 0.5f * U_alphaBeta->beta;
	 // step2：扇区判断
    // 根据u1、u2和u3的正负情况确定所处的扇区
    svpwm->sector = (svpwm->u1 > 0.0f) + ((svpwm->u2 > 0.0f) << 1) + ((svpwm->u3 > 0.0f) << 2); // N=4*C+2*B+A

    // step3:计算基本矢量电压作用时间（占空比）
    // 根据扇区的不同，计算对应的t_a、t_b和t_c的值，表示生成的三相电压的时间
    switch (svpwm->sector)
    {
    case 5:
        // 扇区5
        svpwm->t4 = svpwm->u3;
        svpwm->t6 = svpwm->u1;
        sum = svpwm->t4 + svpwm->t6;
        if (sum > svpwm->Ts)
        {
            k_svpwm = svpwm->Ts / sum; //
            svpwm->t4 = k_svpwm * svpwm->t4;
            svpwm->t6 = k_svpwm * svpwm->t6;
        }
        svpwm->t7 = (svpwm->Ts - svpwm->t4 - svpwm->t6) / 2;
        svpwm->ta = svpwm->t4 + svpwm->t6 + svpwm->t7;
        svpwm->tb = svpwm->t6 + svpwm->t7;
        svpwm->tc = svpwm->t7;
        break;
      case 1:
        // 扇区1
        svpwm->t2 = -svpwm->u3;
        svpwm->t6 = -svpwm->u2;
        sum = svpwm->t2 + svpwm->t6;
        if (sum > svpwm->Ts)
        {
            k_svpwm = svpwm->Ts / sum; // 计算缩放系数
            svpwm->t2 = k_svpwm * svpwm->t2;
            svpwm->t6 = k_svpwm * svpwm->t6;
        }
        svpwm->t7 = (svpwm->Ts - svpwm->t2 - svpwm->t6) / 2;
        svpwm->ta = svpwm->t6 + svpwm->t7;
        svpwm->tb = svpwm->t2 + svpwm->t6 + svpwm->t7;
        svpwm->tc = svpwm->t7;
        break;
    case 3:
        // 扇区3
        svpwm->t2 = svpwm->u1;
        svpwm->t3 = svpwm->u2;
        sum = svpwm->t2 + svpwm->t3;
        if (sum > svpwm->Ts)
        {
            k_svpwm = svpwm->Ts / sum; //
            svpwm->t2 = k_svpwm * svpwm->t2;
            svpwm->t3 = k_svpwm * svpwm->t3;
        }
        svpwm->t7 = (svpwm->Ts - svpwm->t2 - svpwm->t3) / 2;
        svpwm->ta = svpwm->t7;
        svpwm->tb = svpwm->t2 + svpwm->t3 + svpwm->t7;
        svpwm->tc = svpwm->t3 + svpwm->t7;
        break;

    case 2:
        // 扇区2
        svpwm->t1 = -svpwm->u1;
        svpwm->t3 = -svpwm->u3;
        sum = svpwm->t1 + svpwm->t3;
        if (sum > svpwm->Ts)
        {
            k_svpwm = svpwm->Ts / sum; //
            svpwm->t1 = k_svpwm * svpwm->t1;
            svpwm->t3 = k_svpwm * svpwm->t3;
        }
        svpwm->t7 = (svpwm->Ts - svpwm->t1 - svpwm->t3) / 2;
        svpwm->ta = svpwm->t7;
        svpwm->tb = svpwm->t3 + svpwm->t7;
        svpwm->tc = svpwm->t1 + svpwm->t3 + svpwm->t7;
        break;

    case 6:
        // 扇区6
        svpwm->t1 = svpwm->u2;
        svpwm->t5 = svpwm->u3;
        sum = svpwm->t1 + svpwm->t5;
        if (sum > svpwm->Ts)
        {
            k_svpwm = svpwm->Ts / sum; //
            svpwm->t1 = k_svpwm * svpwm->t1;
            svpwm->t5 = k_svpwm * svpwm->t5;
        }
        svpwm->t7 = (svpwm->Ts - svpwm->t1 - svpwm->t5) / 2;
        svpwm->ta = svpwm->t5 + svpwm->t7;
        svpwm->tb = svpwm->t7;
        svpwm->tc = svpwm->t1 + svpwm->t5 + svpwm->t7;
        break;

    case 4:
        // 扇区4
        svpwm->t4 = -svpwm->u2;
        svpwm->t5 = -svpwm->u1;
        sum = svpwm->t4 + svpwm->t5;
        if (sum > svpwm->Ts)
        {
            k_svpwm = svpwm->Ts / sum; //
            svpwm->t4 = k_svpwm * svpwm->t4;
            svpwm->t5 = k_svpwm * svpwm->t5;
        }
        svpwm->t7 = (svpwm->Ts - svpwm->t4 - svpwm->t5) / 2;
        svpwm->ta = svpwm->t4 + svpwm->t5 + svpwm->t7;
        svpwm->tb = svpwm->t7;
        svpwm->tc = svpwm->t5 + svpwm->t7;
        break;


    default:
        break;
    }

		// step4：6路PWM输出
    set_PWM_value(PWM_PERIOD*svpwm->ta,PWM_PERIOD*svpwm->tb,PWM_PERIOD*svpwm->tc);
//    vofa_JustFloat_output(PWM_PERIOD * svpwm->ta,PWM_PERIOD * svpwm->tb,PWM_PERIOD * svpwm->tc,0.0f);
}

void svpwm_test(void)
{
    float theta = 0;
    DQ_Def test_dq;
    AlphaBeta_Def test_ab;
    SVPWM_Def svpwm_out;

    test_dq.d = 0.0f;
    test_dq.q = 0.2f;

    for(theta=0.0f;theta<2*PI;theta+=0.3f)
    {
        inverseParkTransform(&test_dq, &test_ab, theta);
        SVPWM(&test_ab, &svpwm_out);
//        vofa_JustFloat_output(100*svpwm_out.ta,100*svpwm_out.tb,100*svpwm_out.tc,0.0f);
    }

}
