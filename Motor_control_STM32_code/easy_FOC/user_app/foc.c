#include "foc.h"
#include "motor_bsp.h"
#include "user_config.h"
#include <stdio.h>
#include <math.h>
#include "debug.h"

FOC_Def FOC;


void FOC_hardware_init(void)
{
  PWM_ADC_init();
  
}

void FOC_software_init(void)
{
 

}


/**********************************************************************************************************
Clarke�任 ����������������alpha��bate����
I�� = Ia
I�� = (Ia + 2Ib) / sqrt(3)
**********************************************************************************************************/
void clarkeTransform(ABC_Def *abc, AlphaBeta_Def *alphaBeta)
{
    alphaBeta->alpha = abc->Ua;
    alphaBeta->beta = (abc->Ua + 2 * abc->Ub) * 0.57735027f;
}

/**********************************************************************************************************
Park�任�������Ƕȡ�Ialpha��Ibeta������Park�任�õ�Iq��Id
Id = I�� �� cos�� + I�� �� sin��
Iq = I�� �� sin�� + I�� �� cos��
**********************************************************************************************************/
void parkTransform(const AlphaBeta_Def *alphaBeta, float angle, DQ_Def *dq)
{
    float sinAngle = sin(angle);
    float cosAngle = cos(angle);

    dq->d = cosAngle * alphaBeta->alpha + sinAngle * alphaBeta->beta;
    dq->q = -sinAngle * alphaBeta->alpha + cosAngle * alphaBeta->beta;
}

/**********************************************************************************************************
park��任������Uq��Ud�õ�Ualpha��Ubeta
U�� = Ud �� cos�� - Uq �� sin��
U�� = Ud �� sin�� + Uq �� cos��
**********************************************************************************************************/
void inverseParkTransform(DQ_Def *dq, AlphaBeta_Def *alphaBeta, float angle)
{
    float cosAngle = cos(angle);
    float sinAngle = sin(angle);

    alphaBeta->alpha = dq->d * cosAngle - dq->q * sinAngle;
    alphaBeta->beta = dq->d * sinAngle + dq->q * cosAngle;
}

/**********************************************************************************************************
��Clarke�任������Ualpha��Ubeta���õ�Ua,Ub,Uc
Ua = U��
Ub = -1/2 * U�� + sqrt(3)/2  * U��
Ub = -1/2 * U�� - sqrt(3)/2  * U��
**********************************************************************************************************/
// void inverseClarkeTransform(AlphaBeta_Def *abVoltage, ABC_Def *abc)
// {
//     abc->U_u = abVoltage->alpha;
//     abc->U_v = -0.5 * abVoltage->alpha + 0.8660254 * abVoltage->beta;
//     abc->U_w = -0.5 * abVoltage->alpha - 0.8660254 * abVoltage->beta;
// }

/**********************************************************************************************************
������任�еķ�Park�任�õ��� Valpha ��Vbeta ת����·PWM�����

**********************************************************************************************************/
void SVPWM(AlphaBeta_Def *U_alphaBeta, SVPWM_Def *svpwm)
{
    float sum;
    float k_svpwm;
    
    svpwm->Ts = 1.0f; // SVPWM�Ĳ�������

    svpwm->U_alpha = U_alphaBeta->alpha;
    svpwm->U_beta = U_alphaBeta->beta;

	  // step1 ����u1��u2��u3
    // ����SVPWM�㷨�е��������Ƶ�ѹu1��u2��u3
    svpwm->u1 = U_alphaBeta->beta;
    svpwm->u2 = -0.8660254f * U_alphaBeta->alpha - 0.5f * U_alphaBeta->beta; // sqrt(3)/2 �� 0.86603
    svpwm->u3 = 0.8660254f * U_alphaBeta->alpha - 0.5f * U_alphaBeta->beta;
	 // step2�������ж�
    // ����u1��u2��u3���������ȷ������������
    svpwm->sector = (svpwm->u1 > 0.0f) + ((svpwm->u2 > 0.0f) << 1) + ((svpwm->u3 > 0.0f) << 2); // N=4*C+2*B+A
	
    // step3:�������ʸ����ѹ����ʱ�䣨ռ�ձȣ�
    // ���������Ĳ�ͬ�������Ӧ��t_a��t_b��t_c��ֵ����ʾ���ɵ������ѹ��ʱ��
    switch (svpwm->sector)
    {
    case 5:
        // ����5
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
        // ����1
        svpwm->t2 = -svpwm->u3;
        svpwm->t6 = -svpwm->u2;
        sum = svpwm->t2 + svpwm->t6;
        if (sum > svpwm->Ts)
        {
            k_svpwm = svpwm->Ts / sum; // ��������ϵ��
            svpwm->t2 = k_svpwm * svpwm->t2;
            svpwm->t6 = k_svpwm * svpwm->t6;
        }
        svpwm->t7 = (svpwm->Ts - svpwm->t2 - svpwm->t6) / 2;
        svpwm->ta = svpwm->t6 + svpwm->t7;
        svpwm->tb = svpwm->t2 + svpwm->t6 + svpwm->t7;
        svpwm->tc = svpwm->t7;
        break; 
    case 3:
        // ����3
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
        // ����2
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
        // ����6
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
        // ����4
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
		
		// step4��6·PWM���
    set_PWM_value(PWM_PERIOD*svpwm->ta,PWM_PERIOD*svpwm->tb,PWM_PERIOD*svpwm->tc);
}

void svpwm_test(void)
{
    float theta = 0;
    DQ_Def test_dq;
    AlphaBeta_Def test_ab;
    SVPWM_Def svpwm_out;

    test_dq.d = 0.2f;
    test_dq.q = 0.0f;

    for (theta = 0; theta < 6.2831853f; theta += 0.275f)
     {
        inverseParkTransform(&test_dq,&test_ab,theta);
        SVPWM(&test_ab,&svpwm_out);
			 	vofa_JustFloat_output(100.0f*svpwm_out.ta,100.0f*svpwm_out.tb,100.0f*svpwm_out.tc,0.0f);
//			 HAL_Delay(1);
    }
}


