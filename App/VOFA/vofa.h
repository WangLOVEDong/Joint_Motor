#ifndef VOFA_H
#define VOFA_H

#include "stm32g4xx_hal.h"

/* 一帧最多发送8路浮点数据 */
#define VOFA_MAX_FLOAT_COUNT  8U

/*
 * 使用USART1 DMA发送JustFloat数据。
 * data：浮点数组。
 * count：通道数量，范围1～8。
 *
 * HAL_OK：已启动发送。
 * HAL_BUSY：串口或上一帧仍忙，本次未发送。
 * HAL_ERROR：参数错误或发送失败。
 *
 * 当前约定：仅从主循环调用。
 */
HAL_StatusTypeDef VOFA_SendFloatArray_DMA(const float *data, uint8_t count);

#endif