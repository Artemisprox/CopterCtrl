#include "IMU_TEST.h"

#include <rtdevice.h>
#include "drv_gpio.h"

#include "stm32f4xx_hal.h"

#define IMU1_IS_USING 0
#define IMU2_IS_USING 1

#define ICM20_PWR_MGMT_1 0x6B
#define ICM20_PWR_MGMT_2 0x6C

#define CS1_PIN GET_PIN(C, 11)
#define CS2_PIN GET_PIN(A, 4)

#define RGB_DEV_NAME "pwm3" /* PWM设备名称 */

#define RGB_R 2 /* PWM通道 */
#define RGB_B 3 /* PWM通道 */

#define TVCC_DEV_NAME "pwm2" /* PWM设备名称 */

#define T_VCC_1 1 /* PWM通道 */
#define T_VCC_2 2 /* PWM通道 */

static struct rt_device_pwm *RGB_dev; /* PWM设备句柄 */
static struct rt_device_pwm *TVCC_dev; /* PWM设备句柄 */

static rt_uint32_t period_R, pulse_R, period_B, pulse_B;

SPI_HandleTypeDef hspi1;

static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

static uint8_t tx, rx;

static uint8_t icm20602_write_reg(uint8_t reg, uint8_t val, rt_base_t CS_Pin_Num)
{

    rt_pin_write(CS_Pin_Num, PIN_LOW);
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);
    tx = val;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);
    rt_pin_write(CS_Pin_Num, PIN_HIGH);

    return 0;
}

static uint8_t icm20602_read_reg(uint8_t reg, rt_base_t CS_Pin_Num)
{
    rt_pin_write(CS_Pin_Num, PIN_LOW);
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);
    rt_pin_write(CS_Pin_Num, PIN_HIGH);

    return rx;
}

static uint8_t ID[2]={0};
rt_err_t IMU_Init(void)
{
	
	period_R = 200000;
    period_B = 200000;

    RGB_dev = (struct rt_device_pwm *)rt_device_find(RGB_DEV_NAME);

    rt_pwm_set(RGB_dev, RGB_R, period_R, pulse_R);
    rt_pwm_set(RGB_dev, RGB_B, period_B, pulse_B);
    /* 使能设备 */
    rt_pwm_enable(RGB_dev, RGB_R);
    rt_pwm_enable(RGB_dev, RGB_B);
	
	TVCC_dev = (struct rt_device_pwm *)rt_device_find(TVCC_DEV_NAME);

    rt_pwm_set(TVCC_dev, T_VCC_1, 200000, 0);
    rt_pwm_set(TVCC_dev, T_VCC_2, 200000, 0);
    /* 使能设备 */
    rt_pwm_enable(TVCC_dev, T_VCC_1);
    rt_pwm_enable(TVCC_dev, T_VCC_2);
	// 片选引脚初始化
	rt_pin_mode(CS1_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(CS1_PIN, PIN_HIGH);
	
	
    rt_pin_mode(CS2_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(CS2_PIN, PIN_HIGH);
	// SPI1初始化
    MX_SPI1_Init();

#if (IMU1_IS_USING)
    
    // 读取陀螺仪1的ID
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x80, CS1_PIN);
    rt_thread_mdelay(10);

    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x01, CS1_PIN);
    rt_thread_mdelay(10);

    ID[0] = icm20602_read_reg(0x75, CS1_PIN);

    RT_ASSERT(ID[0] == 0x12);
	rt_pwm_set(RGB_dev, RGB_R, period_R, 1000);
#endif
#if (IMU2_IS_USING)

    // 读取陀螺仪2的ID
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x80, CS2_PIN);
    rt_thread_mdelay(10);

    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x01, CS2_PIN);
    rt_thread_mdelay(10);

    ID[1] = icm20602_read_reg(0x75, CS2_PIN);

    RT_ASSERT(ID[1] == 0x12);
	rt_pwm_set(RGB_dev, RGB_B, period_B, 1000);
#endif
    return 0;
}
