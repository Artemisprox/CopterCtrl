#include "drv_iic.h"

#if (USE_CUBEMX_IIC)

static I2C_HandleTypeDef hi2c2;

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static int MX_I2C2_Init(void)
{

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */
    return RT_EOK;
}
INIT_DEVICE_EXPORT(MX_I2C2_Init);

/* 写iic */
rt_err_t iic_write_reg(rt_uint16_t SLAVE_ADDR, rt_uint8_t cmd, rt_uint8_t dat)
{
    rt_uint8_t buf[2];

    buf[0] = cmd; //cmd
    buf[1] = dat;

    /* 调用I2C设备接口传输数据 */
    if (HAL_OK==HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDR, buf, 2, 5))
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}

#else
#define RTT_IIC_DEV_NAME "i2c2"

static struct rt_i2c_bus_device *i2c_bus = RT_NULL; /* I2C总线设备句柄 */

int rtt_iic_init(void)
{
    /* 查找I2C总线设备，获取I2C总线设备句柄 */
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(RTT_IIC_DEV_NAME);

    if (i2c_bus == RT_NULL)
    {
		return RT_ERROR;
    }
	return RT_EOK;
}
INIT_DEVICE_EXPORT(MX_I2C2_Init);

/* 写iic */
rt_err_t iic_write_reg(rt_uint16_t SLAVE_ADDR, rt_uint8_t cmd, rt_uint8_t dat)
{
    rt_uint8_t buf[2];

    buf[0] = cmd; //cmd
    buf[1] = dat;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_master_send(i2c_bus, SLAVE_ADDR >> 1, RT_I2C_WR, buf, 2))
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
    
}
#endif

