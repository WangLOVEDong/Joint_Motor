#include "adc_sampling.h"
#include "adc.h"
#include "tim.h"
#include "main.h"

/* 中断更新，主循环通过接口获取 */
static volatile ADC_RawData_t adc_raw_data = {0};

#define ADC_ZERO_SKIP_COUNT    1000U
#define ADC_ZERO_SAMPLE_COUNT  1024U

#define ADC_REFERENCE_VOLTAGE  3.3f
#define ADC_COUNTS            4096.0f
#define CURRENT_SHUNT_OHM     0.005f
#define CURRENT_AMP_GAIN      20.0f

#define CURRENT_AMP_PER_COUNT  (ADC_REFERENCE_VOLTAGE / (ADC_COUNTS * CURRENT_SHUNT_OHM * CURRENT_AMP_GAIN))


static uint32_t zero_skip_count = 0U;
static uint32_t zero_sum[3] = {0U, 0U, 0U};

static volatile ADC_ZeroData_t adc_zero_data =
{
    .offset = {0.0f, 0.0f, 0.0f},
    .min_raw = {4095U, 4095U, 4095U},
    .max_raw = {0U, 0U, 0U},
    .sample_count = 0U,
    .ready = 0U
};

HAL_StatusTypeDef ADC_Sampling_Start(void)
{
    HAL_StatusTypeDef status;

    /* 本阶段保持功率驱动关闭 */
    HAL_GPIO_WritePin(DRV_NSLEEP_OC_EN_GPIO_Port,
                      DRV_NSLEEP_OC_EN_Pin,
                      GPIO_PIN_RESET);

    /* ADC内部自校准，不是电流零偏校准 */
    status = HAL_ADCEx_Calibration_Start(
        &hadc1, ADC_SINGLE_ENDED);

    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADCEx_Calibration_Start(
        &hadc2, ADC_SINGLE_ENDED);

    if (status != HAL_OK)
    {
        return status;
    }

    /* 先准备从ADC2，不开启它的完成中断 */
    status = HAL_ADCEx_InjectedStart(&hadc2);

    if (status != HAL_OK)
    {
        return status;
    }

    /* 再准备主ADC1，开启注入组完成中断 */
    status = HAL_ADCEx_InjectedStart_IT(&hadc1);

    if (status != HAL_OK)
    {
        return status;
    }

    /* 最后启动内部CH4，建立ADC触发节拍 */
    return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}


static void ADC_Sampling_CollectZero(uint16_t u, uint16_t v, uint16_t w)
{
    /* 已经完成，后续采样不再改变零点 */
    if (adc_zero_data.ready != 0U) return;

    /* 跳过刚启动的一段采样 */
    if (zero_skip_count < ADC_ZERO_SKIP_COUNT)
    {
        zero_skip_count++;
        return;
    }

    uint16_t raw[3] = {u, v, w};

    for (uint8_t i = 0U; i < 3U; i++)
    {
        zero_sum[i] += raw[i];

        if (raw[i] < adc_zero_data.min_raw[i])
        {
            adc_zero_data.min_raw[i] = raw[i];
        }

        if (raw[i] > adc_zero_data.max_raw[i])
        {
            adc_zero_data.max_raw[i] = raw[i];
        }
    }

    adc_zero_data.sample_count++;

    if (adc_zero_data.sample_count >= ADC_ZERO_SAMPLE_COUNT)
    {
        for (uint8_t i = 0U; i < 3U; i++)
        {
            adc_zero_data.offset[i] =
                (float)zero_sum[i]
                / (float)ADC_ZERO_SAMPLE_COUNT;
        }

        /* 三相平均值全部计算完，再发布完成标志 */
        adc_zero_data.ready = 1U;
    }
}

/* ADC注入组完成后，HAL自动调用 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1)
    {
        return;
    }

    uint16_t u = (uint16_t)ADC2->JDR1;
    uint16_t v = (uint16_t)ADC1->JDR1;
    uint16_t w = (uint16_t)ADC1->JDR2;

    adc_raw_data.current_u_raw = u;
    adc_raw_data.current_v_raw = v;
    adc_raw_data.current_w_raw = w;
    adc_raw_data.sample_count++;

    /* 把这一批三相数据交给零偏校准 */
    ADC_Sampling_CollectZero(u, v, w);
}

void ADC_Sampling_GetRawData(ADC_RawData_t *data)
{
    if (data == NULL)
    {
        return;
    }

    /* 保存原来的中断屏蔽状态 */
    uint32_t saved_primask = __get_PRIMASK();

    /* 短暂屏蔽中断，避免复制一半时采样结果更新 */
    __disable_irq();

    data->current_u_raw = adc_raw_data.current_u_raw;
    data->current_v_raw = adc_raw_data.current_v_raw;
    data->current_w_raw = adc_raw_data.current_w_raw;
    data->sample_count = adc_raw_data.sample_count;

    /* 恢复进入函数之前的中断状态 */
    __set_PRIMASK(saved_primask);
}

void ADC_Sampling_GetZeroData(ADC_ZeroData_t *data)
{
    if (data == NULL) return;


    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();

    for (uint8_t i = 0U; i < 3U; i++)
    {
        data->offset[i] = adc_zero_data.offset[i];
        data->min_raw[i] = adc_zero_data.min_raw[i];
        data->max_raw[i] = adc_zero_data.max_raw[i];
    }

    data->sample_count = adc_zero_data.sample_count;
    data->ready = adc_zero_data.ready;

    __set_PRIMASK(saved_primask);
}

void ADC_Sampling_GetCenteredData(ADC_CenteredData_t *data)
{
    if (data == NULL)
    {
        return;
    }

    ADC_RawData_t raw;
    ADC_ZeroData_t zero;

    ADC_Sampling_GetRawData(&raw);
    ADC_Sampling_GetZeroData(&zero);

    data->ready = zero.ready;

    if (zero.ready == 0U)
    {
        /* 校准尚未完成，暂不提供减零点结果 */
        data->u_counts = 0.0f;
        data->v_counts = 0.0f;
        data->w_counts = 0.0f;
        return;
    }

    /* 实时原始值减去对应相的平均零点 */
    data->u_counts =
        (float)raw.current_u_raw - zero.offset[0];

    data->v_counts =
        (float)raw.current_v_raw - zero.offset[1];

    data->w_counts =
        (float)raw.current_w_raw - zero.offset[2];
}

void ADC_Sampling_GetCurrent(ADC_CurrentData_t *data)
{
    if (data == NULL)  return;

    ADC_CenteredData_t centered;
    ADC_Sampling_GetCenteredData(&centered);

    data->ready = centered.ready;

    data->u_amp = centered.u_counts * CURRENT_AMP_PER_COUNT;
    data->v_amp = centered.v_counts * CURRENT_AMP_PER_COUNT;
    data->w_amp = centered.w_counts * CURRENT_AMP_PER_COUNT;
}
