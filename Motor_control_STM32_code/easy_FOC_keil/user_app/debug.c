#include "stdio.h"
#include "debug.h"
 #include "math.h"

extern UART_HandleTypeDef huart2;

uint8_t StaMessages[4] = {0x11,0x22,0x33,0x44};
uint8_t RxBuffer[20];

/**
  * ��������: �ض���c�⺯��printf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart2, &ch, 1, 0xffff);
  return ch;
}

void uart_debug_init(void)
{

    HAL_UART_Transmit_IT(&huart2 ,(uint8_t*)StaMessages,sizeof(StaMessages));
	  HAL_UART_Receive_IT(&huart2,(uint8_t*)RxBuffer,1);//�����жϽ��պ��������ճ�����Ϊ1������һ���ֽڽ�һ���ж�
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    HAL_UART_Transmit(&huart2,(uint8_t*)RxBuffer,1,0xFFFF);  //���ͽ��յ�������
    HAL_UART_Receive_IT(&huart2,(uint8_t*)RxBuffer,1);  //�ٿ��������жϣ���Ϊ�����ж�ֻ�ᴥ��һ�Σ������Ҫ�ٴο�����

}


//��Э����CSV�����ַ�������ֱ�ۼ�࣬�����printf��
void vofa_FireWater_output(float s1,float s2,float s3)
{
	printf("simples:%f,%f,%f\n", s1, s2,s3); 	

}

//��Э���Ǵ�ʮ�����Ƹ��㴫��
void vofa_JustFloat_output(float s1,float s2,float s3,float s4)
{
	float data[4];  
	uint8_t tail[4]  = {0x00, 0x00, 0x80, 0x7f}; 
    // ��������
    data[0] = s1;
    data[1] = s2;
    data[2] = s3;
    data[3] = s4;
	  HAL_UART_Transmit(&huart2,(uint8_t*)data,sizeof(float) * 4,100);  //��������
    // ����֡β 
		HAL_UART_Transmit(&huart2,tail,4,100);

}



