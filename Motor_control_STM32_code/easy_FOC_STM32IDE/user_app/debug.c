#include "stdio.h"
#include "debug.h"
#include "math.h"

extern UART_HandleTypeDef huart2;

uint8_t StaMessages[4] = {0x11, 0x22, 0x33, 0x44};
uint8_t RxBuffer[20];

/**
 * 函数功能: 重定向c库函数printf到DEBUG_USARTx
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：无
 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
 * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：无
 */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart2, &ch, 1, 0xffff);
  return ch;
}

void uart_debug_init(void)
{

  HAL_UART_Transmit_IT(&huart2, (uint8_t *)StaMessages, sizeof(StaMessages));
  HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1); // 调用中断接收函数，接收长度设为1，接收一个字节进一次中断
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  HAL_UART_Transmit(&huart2, (uint8_t *)RxBuffer, 1, 0xFFFF); // 发送接收到的数据
  HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);       // 再开启接收中断（因为里面中断只会触发一次，因此需要再次开启）
}

// 本协议是CSV风格的字符串流，直观简洁，编程像printf简单
void vofa_FireWater_output(float s1, float s2, float s3)
{
  	printf("simples:%f,%f,%f\n", s1, s2,s3);
}

// 本协议是纯十六进制浮点传输
void vofa_JustFloat_output(float s1, float s2, float s3, float s4)
{
  float data[4];
  uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
  // 发送数据
  data[0] = s1;
  data[1] = s2;
  data[2] = s3;
  data[3] = s4;
  HAL_UART_Transmit(&huart2, (uint8_t *)data, sizeof(float) * 4, 100); // 发送数据
  // 发送帧尾
  HAL_UART_Transmit(&huart2, tail, 4, 100);
}
