#include <drv_dac.h>

#include "stm32f1xx_hal.h"
#ifdef RT_USING_DAC1
static DAC_HandleTypeDef hdac;

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

    /* USER CODE BEGIN DAC_Init 0 */

    /* USER CODE END DAC_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC_Init 1 */

    /* USER CODE END DAC_Init 1 */
    /** DAC Initialization 
  */
    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
        rt_kprintf("drv_dac.c --DACinit failed.\n");
    }
    /** DAC channel OUT1 config 
  */
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        rt_kprintf("drv_dac.c --DAC-channel1-init failed.\n");
    }
    /** DAC channel OUT2 config 
  */
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
        rt_kprintf("drv_dac.c --DAC-channel2-init failed.\n");
    }
    /* USER CODE BEGIN DAC_Init 2 */

    /* USER CODE END DAC_Init 2 */
}

//根据单片机供电电压计算出DAC转换时使用的系数
static float DAC_DATA_SET_K = (4095.0f / HW_VOLTAGE_SUPPLY);

rt_err_t User_dac_write(rt_int8_t channel,float voltage_set)
{
    float SPLY_Voltage_REC;
    char VoltageErr = 0;
    rt_uint32_t DAC_DATA_SET;//计算电压设定值对应的DAC数值

    SPLY_Voltage_REC = Sply_Voltage;
    DAC_DATA_SET_K = (4095.0f / SPLY_Voltage_REC);
    if (voltage_set<0)
    { //检查是否为负数电压
        voltage_set = 0;
        VoltageErr = 1;
    }
    else if (voltage_set > SPLY_Voltage_REC)
    {
        voltage_set = SPLY_Voltage_REC;
        VoltageErr = 1;
    }

    DAC_DATA_SET = (rt_uint32_t)(voltage_set * DAC_DATA_SET_K);

    if (DAC_DATA_SET>4095)
    { //由于DAC_DATA_SET_K的float可能存在误差，则还需要检查计算结果是否超过DAC转换上限
        DAC_DATA_SET = 4095;
    }
    switch (channel)
    {
        case 1:
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_DATA_SET);
            break;
        case 2:
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_DATA_SET);
            break;

        default:
            break;
    }
    if (VoltageErr)
    {
        return RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}



//使用HAL库中的初始化函数对dac外设进行初始化
int drv_dac_init(void)
{
	MX_DAC_Init();
    User_dac_write(1, drv_dac_ch1vol_default);
    User_dac_write(2, drv_dac_ch2vol_default);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    return RT_EOK;
}
INIT_DEVICE_EXPORT(drv_dac_init);

#endif
