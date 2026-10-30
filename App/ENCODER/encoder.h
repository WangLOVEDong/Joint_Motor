#ifndef ENCODER_H
#define ENCODER_H

#include "stm32g4xx_hal.h"

#define MT6816_REG_ANGLE_HIGH  0x03U
#define MT6816_REG_ANGLE_LOW   0x04U

/* 编码器对外提供的数据 */
typedef struct
{
    uint16_t raw_angle;       /* 原始角度：0～16383 */
    float angle_deg;         /* 单圈机械角度，单位：度 */
    float delta_deg;         /* 跨零点修正后的角度差，单位：度 */
    uint8_t delta_valid;     /* 1：本次角度差可用 */
    HAL_StatusTypeDef status; /* 最近一次读取结果 */
} Encoder_Data_t;

/* 变量在encoder.c中定义，这里声明供其他文件访问 */
extern Encoder_Data_t encoder_data;

/* 读取、校验并更新角度及角度差，仅从主循环调用 */
void Encoder_Update(void);

/*
 * 读取MT6816的一个寄存器。
 * address：寄存器地址。
 * value：保存读取结果的变量地址。
 *
 * 仅在返回HAL_OK时，value中的数据才被更新。
 */
HAL_StatusTypeDef Encoder_ReadRegister(uint8_t address, uint8_t *value);

/*
 * 读取14位原始角度，范围0～16383。
 * 成功时更新输出，失败时保留原值。
 */
HAL_StatusTypeDef Encoder_ReadRawAngle(uint16_t *raw_angle);

/* 计算跨零点修正后的角度差，输入、输出单位均为度 */
float Encoder_CalculateAngleDelta(float current_angle, float last_angle);

#endif