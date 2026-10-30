#include "vofa.h"
#include "usart.h"
#include <string.h>

#define VOFA_FRAME_TAIL_SIZE  4U

#define VOFA_FRAME_MAX_SIZE \
    (VOFA_MAX_FLOAT_COUNT * sizeof(float) + VOFA_FRAME_TAIL_SIZE)

/* DMA异步读取，缓冲区必须在函数返回后仍然有效 */
static uint8_t vofa_tx_buffer[VOFA_FRAME_MAX_SIZE];

/* 主循环设置，中断回调清除 */
static volatile uint8_t vofa_tx_busy = 0U;

/* JustFloat固定帧尾 */
static const uint8_t vofa_frame_tail[VOFA_FRAME_TAIL_SIZE] =
{
    0x00, 0x00, 0x80, 0x7F
};

HAL_StatusTypeDef VOFA_SendFloatArray_DMA(const float *data, uint8_t count)
{
    uint16_t data_size;
    HAL_StatusTypeDef status;

    /* 检查数组指针和通道数量 */
    if ((data == NULL) ||
        (count == 0U) ||
        (count > VOFA_MAX_FLOAT_COUNT))
    {
        return HAL_ERROR;
    }

    /* 上一帧未结束时，禁止改写发送缓冲区 */
    if (vofa_tx_busy != 0U)
    {
        return HAL_BUSY;
    }

    /* 保留旧BLDC中对串口状态的检查 */
    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    data_size = (uint16_t)(count * sizeof(float));

    /* 复制浮点数据，不进行字符串格式化 */
    memcpy(vofa_tx_buffer, data, data_size);

    /* 在浮点数据后添加帧尾 */
    memcpy(&vofa_tx_buffer[data_size],
           vofa_frame_tail,
           VOFA_FRAME_TAIL_SIZE);

    /* 启动DMA前先占用缓冲区 */
    vofa_tx_busy = 1U;

    status = HAL_UART_Transmit_DMA(&huart1, vofa_tx_buffer, (uint16_t)(data_size + VOFA_FRAME_TAIL_SIZE));

    /* 启动失败，释放缓冲区供后续重试 */
    if (status != HAL_OK)
    {
        vofa_tx_busy = 0U;
    }

    return status;
}

/* HAL在串口发送完成后调用 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        vofa_tx_busy = 0U;
    }
}