/*
 * IIC_encoder.c
 *
 *  Created on: Sep 14, 2023
 *      Author: smile
 */
#include "IIC_encoder.h"


extern I2C_HandleTypeDef hi2c3;

uint8_t encoder_addr_H = 0x03;
uint8_t encoder_addr_L = 0x04;

uint8_t encoder_read_buf_h;
uint8_t encoder_read_buf_l;
uint16_t encoder_read_buf;
unsigned char mt6701_write_reg(unsigned char reg, unsigned char value)
{
	return HAL_I2C_Mem_Write(&hi2c3, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, MT6701_Timeout);
}

unsigned char mt6701_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	return HAL_I2C_Mem_Write(&hi2c3, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, len, MT6701_Timeout);
}

unsigned char mt6701_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	return HAL_I2C_Mem_Read(&hi2c3, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_Timeout);
}

void mt6701_delay(unsigned int ms)
{
	HAL_Delay(ms);
}

// 14Bit角度信息，存储在0x03[13:6]、0x04[5:0]两个寄存器中，高位在前，原始读数0~16383，对应0-360°
void encoder_read_data(void)
{
    uint8_t temp[2];
    mt6701_read_reg(MT6701_REG_ANGLE_14b, temp, 2);

    encoder_read_buf = ((int16_t)temp[0] << 6) | (temp[1] >> 2);
//    *angle_f = (float)*angle * 360 / 16384;
}

//void encoder_read_data(void)
//{
//	HAL_I2C_Master_Transmit(&hi2c3,0x06,&encoder_addr_H,1,1000);
//	HAL_I2C_Master_Receive(&hi2c3,0x06,&encoder_read_buf_h,1,1000);
//
//	HAL_I2C_Master_Transmit(&hi2c3,0x06,&encoder_addr_L,1,1000);
//	HAL_I2C_Master_Receive(&hi2c3,0x06,&encoder_read_buf_l,1,1000);
//}


