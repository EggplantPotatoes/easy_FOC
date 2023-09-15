/*
 * IIC_encoder.h
 *
 *  Created on: Sep 14, 2023
 *      Author: smile
 */

#ifndef IIC_ENCODER_H_
#define IIC_ENCODER_H_

#include "main.h"


#define MT6701_SLAVE_ADDR         0x06<<1
#define MT6701_Timeout            50

#define MT6701_REG_ANGLE_14b      0x03    // 14Bit角度信息，存储在0x03[13:6]、0x04[5:0]两个寄存器中，高位在前，原始读数0~16383

#define mt6701_log		printf



void i2c_mt6701_get_angle(int16_t *angle, float *angle_f);

void encoder_read_data(void);


#endif /* IIC_ENCODER_H_ */
