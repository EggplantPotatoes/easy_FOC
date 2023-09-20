/*
 * data_filter.c
 *
 *  Created on: Sep 16, 2023
 *      Author: smile
 */
#include "data_filter.h"


float window_speed[WIN_SIZE];


/**
 * @brief
 *
 * @param in
 * @param out
 * @param window_Array
 * @param i
 * @param sliding_Window_Length
 */
float window_filter(float data, float *buf, uint8_t len)
{
    uint8_t i;
    float sum = 0;

    for (i = 1; i < len; i++)
    {
        buf[i - 1] = buf[i];
    }
    buf[len - 1] = data;

    for (i = 0; i < len; i++)
    {
        sum += buf[i];
    }

    sum /= len;

    return sum;
}


/**
 * @brief  高通滤波器
 * @author
 * @param  in-输入数据 out-滤波输出数据
 * @return void
 */
void high_pass_filter(float in, float *out)
{
    float rc, coff;
    static float in_p, out_p;

    rc = 1.0 / 2.0 / Pi / HP_CUT_FRQ;
    coff = rc / (rc + 1 / SAMPLE_RATE);
    *out = (in - in_p + out_p) * coff;

    out_p = *out;
    in_p = in;
}

/**
 * @brief  implement 1 order RC low pass filter
 *         raw data filtered by a simple RC low pass filter@cufoff=5Hz
 * @param  Vi 		: 	Vi(k)
 * @param  Vi_p 	: 	Vi(k-1)
 * @param  Vo 		: 	Vo(k)
 * @param  Vo_p 	: 	Vo(k-1)
 * @note   This example shows a simple way to report end of conversion
 *         and get conversion result. You can add your own implementation.
 * @retval None
 */
void LowPassFilter_RC(float Vi, float *Vo)
{
    float CutFrq, RC, Cof1, Cof2;
    static float *Vo_p;
    // low pass filter @cutoff frequency = 5 Hz
    CutFrq = 2.0f;
    RC = (float)1.0f / 2.0f / Pi / CutFrq;
    Cof1 = 1.0f / (1.0f + RC * SAMPLE_RATE);
    Cof2 = RC * SAMPLE_RATE / (1.0f + RC * SAMPLE_RATE);
    *Vo = Cof1 * (Vi) + Cof2 * (*Vo_p);

    // update
    *Vo_p = *Vo;
}
