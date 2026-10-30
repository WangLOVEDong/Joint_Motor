#ifndef ADC_SAMPLING_H
#define ADC_SAMPLING_H

#include "stm32g4xx_hal.h"

/* 一次三相采样结果 */
typedef struct
{
    uint16_t current_u_raw;  /* A/U相原始ADC码 */
    uint16_t current_v_raw;  /* B/V相原始ADC码 */
    uint16_t current_w_raw;  /* C/W相原始ADC码 */
    uint32_t sample_count;   /* 采样回调次数 */
} ADC_RawData_t;

/* 数组下标：0=U相，1=V相，2=W相 */
typedef struct
{
    float offset[3];       /* 三相平均零点，单位：ADC计数 */
    uint16_t min_raw[3];    /* 校准期间最小值 */
    uint16_t max_raw[3];    /* 校准期间最大值 */
    uint32_t sample_count; /* 已累计的校准样本数 */
    uint8_t ready;         /* 1表示平均计算完成 */
} ADC_ZeroData_t;

/* 获取校准结果快照，仅从主循环调用 */
void ADC_Sampling_GetZeroData(ADC_ZeroData_t *data);

/* 上电初始化完成后调用一次 */
HAL_StatusTypeDef ADC_Sampling_Start(void);

/* 获取同一批次的三相数据快照，仅从主循环调用 */
void ADC_Sampling_GetRawData(ADC_RawData_t *data);

/* 减零点后的三相数据，单位仍是ADC计数 */
typedef struct
{
    float u_counts;
    float v_counts;
    float w_counts;
    uint8_t ready;   /* 零偏校准是否完成 */
} ADC_CenteredData_t;

/* 仅从主循环调用 */
void ADC_Sampling_GetCenteredData(ADC_CenteredData_t *data);

/* 三相电流，单位：安培 */
typedef struct
{
    float u_amp;
    float v_amp;
    float w_amp;
    uint8_t ready;  /* 零偏校准是否完成 */
} ADC_CurrentData_t;

/* 仅从主循环调用 */
void ADC_Sampling_GetCurrent(ADC_CurrentData_t *data);

#endif