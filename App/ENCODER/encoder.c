#include "encoder.h"
#include "main.h"
#include "spi.h"

#define MT6816_READ_COMMAND_BIT  0x80U
#define MT6816_SPI_TIMEOUT_MS    10U

/* 对外提供的数据，整个程序只定义这一份 */
Encoder_Data_t encoder_data =
{
    .raw_angle = 0U,
    .angle_deg = 0.0f,
    .delta_deg = 0.0f,
    .delta_valid = 0U,
    .status = HAL_ERROR
};

/* 仅供本文件内部使用的历史数据 */
static float encoder_last_angle_deg = 0.0f;
static uint8_t encoder_history_ready = 0U;

HAL_StatusTypeDef Encoder_ReadRegister(uint8_t address,
                                       uint8_t *value)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2] = {0U, 0U};
    HAL_StatusTypeDef status;

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    /* 第一个字节：读取命令 + 寄存器地址 */
    tx_data[0] = (uint8_t)(
        MT6816_READ_COMMAND_BIT | (address & 0x7FU));

    /* 第二个字节：用于继续产生SPI时钟 */
    tx_data[1] = 0x00U;

    /* 选择编码器SPI功能   保持高电平是SPI模式 低电平就是ABZ模式或者UVW模式，内部配置决定*/
    HAL_GPIO_WritePin(MT6816_MODE_GPIO_Port,
                    MT6816_MODE_Pin,
                    GPIO_PIN_SET);

    /* 拉低片选，开始本次事务 */
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port,
                      MT6816_CS_Pin,
                      GPIO_PIN_RESET);

    /* 同时发送2字节并接收2字节 */
    status = HAL_SPI_TransmitReceive(
        &hspi1,
        tx_data,
        rx_data,
        2U,
        MT6816_SPI_TIMEOUT_MS);

    /* 成功或失败，都释放片选 */
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port,
                      MT6816_CS_Pin,
                      GPIO_PIN_SET);

    if (status == HAL_OK)
    {
        /* 寄存器数据位于第二个接收字节 */
        *value = rx_data[1];
    }

    return status;
}

/*
 * 两个寄存器共16位，包含校验位本身。
 * 按偶校验规则，1的总数应为偶数。
 *
 * 返回1：通过；返回0：失败。
 */
static uint8_t Encoder_CheckEvenParity(uint8_t high,
                                       uint8_t low)
{
    uint16_t bits =
        (uint16_t)(((uint16_t)high << 8) | low);

    uint8_t one_count = 0U;

    for (uint8_t i = 0U; i < 16U; i++)
    {
        one_count += (uint8_t)((bits >> i) & 0x01U);
    }

    return ((one_count % 2U) == 0U) ? 1U : 0U;
}

HAL_StatusTypeDef Encoder_ReadRawAngle(uint16_t *raw_angle)
{
    uint8_t high_before;
    uint8_t angle_low;
    uint8_t high_after;
    HAL_StatusTypeDef status;

    if (raw_angle == NULL)
    {
        return HAL_ERROR;
    }

    /* 每次调用最多尝试3次 */
    for (uint8_t attempt = 0U; attempt < 3U; attempt++)
    {
        /* 第一次读取高字节 */
        status = Encoder_ReadRegister(
            MT6816_REG_ANGLE_HIGH, &high_before);

        if (status != HAL_OK)
        {
            return status;
        }

        /* 读取低字节 */
        status = Encoder_ReadRegister(
            MT6816_REG_ANGLE_LOW, &angle_low);

        if (status != HAL_OK)
        {
            return status;
        }

        /* 再读一次高字节 */
        status = Encoder_ReadRegister(
            MT6816_REG_ANGLE_HIGH, &high_after);

        if (status != HAL_OK)
        {
            return status;
        }

        /* 高字节发生变化，本次不拼接，重新尝试 */
        if (high_before != high_after)
        {
            continue;
        }

        /* 偶校验失败，重新尝试 */
        if (Encoder_CheckEvenParity(
                high_before, angle_low) == 0U)
        {
            continue;
        }

        /* 0x04的bit1为弱磁报警位，1表示报警 */
        if ((angle_low & 0x02U) != 0U)
        {
            return HAL_ERROR;
        }

        /* 检查通过，才更新调用者的角度变量 */
        *raw_angle = (uint16_t)(
            ((uint16_t)high_before << 6) |
            ((uint16_t)angle_low >> 2));

        return HAL_OK;
    }

    /* 尝试3次仍未通过 */
    return HAL_ERROR;
}

float Encoder_CalculateAngleDelta(float current_angle, float last_angle)
{
    float delta = current_angle - last_angle;
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }
    return delta;
}

void Encoder_Update(void)
{
    uint16_t raw_angle;

    HAL_StatusTypeDef status =
        Encoder_ReadRawAngle(&raw_angle);

    encoder_data.status = status;

    /* 默认本次角度差不可用 */
    encoder_data.delta_valid = 0U;

    if (status != HAL_OK)
    {
        /* 保留上次角度，但丢弃历史比较起点 */
        encoder_data.delta_deg = 0.0f;
        encoder_history_ready = 0U;
        return;
    }

    encoder_data.raw_angle = raw_angle;

    encoder_data.angle_deg = (float)raw_angle * (360.0f / 16384.0f);

    if (encoder_history_ready != 0U)
    {
        encoder_data.delta_deg =
            Encoder_CalculateAngleDelta(
                encoder_data.angle_deg,
                encoder_last_angle_deg);

        encoder_data.delta_valid = 1U;
    }
    else
    {   /* 第一次成功读取只建立起点 */
        encoder_data.delta_deg = 0.0f;
        encoder_history_ready = 1U;
    }
    encoder_last_angle_deg = encoder_data.angle_deg;
}