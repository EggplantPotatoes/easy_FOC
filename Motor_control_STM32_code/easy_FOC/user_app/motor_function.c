#include "motor_function.h"
#include "PWM_Encoder.h"
#include "foc.h"

void Motor_Align(float ud) //���Ԥ��λ���ҵ�Ƕ����
{
	float theta = 0;
	DQ_Def align_dq;
	AlphaBeta_Def align_ab;
	SVPWM_Def svpwm_out;
	
	align_dq.d = ud;
	align_dq.q = 0.0f;
	
	theta = 0;
	inverseParkTransform(&align_dq,&align_ab,theta);
	SVPWM(&align_ab,&svpwm_out);  //���ͣ����Ƕ����
	
	pwm_encoder.angle_rad_offset = pwm_encoder.duty*2*PI;//��ȡ��ǰ��е�Ƕ�


}

void  motor_open_loop_control(float uq)  //�������Ƶ��
{
 
    float theta = 0.0f;
    DQ_Def open_loop_dq;
    AlphaBeta_Def open_loop_ab;
    SVPWM_Def svpwm_out;

	
    open_loop_dq.d = 0.0f;
    open_loop_dq.q = uq;

		Get_PWM_Encoder_Angles();   //��ȡ��Ƕ�
		theta = pwm_encoder.electronic_angle;   
		inverseParkTransform(&open_loop_dq,&open_loop_ab,theta);  //��Ƕȴ���FOC SVPWM��
		SVPWM(&open_loop_ab,&svpwm_out);

}






