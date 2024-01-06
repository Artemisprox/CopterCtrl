#include "drv_spithread.h"
#include "rtthread.h"
#include "rtdevice.h"

#include "board.h"
#include "drv_spi.h"

#define IMU1_SPI_DEVICE_NAME     "spi10"
#define IMU2_SPI_DEVICE_NAME     "spi11"
struct rt_spi_device *spi_dev_IMU1;
struct rt_spi_device *spi_dev_IMU2;

struct rt_spi_message msgACCLRead1, msgACCLRead2, msgACCLRead3;
struct rt_spi_message msgACCLWrite1, msgACCLWrite2;
struct rt_spi_message msgGYRORead1, msgGYRORead2;
struct rt_spi_message msgGYROWrite1, msgGYROWrite2;

void spi_ICM20602_init(void)
{
    char name0[RT_NAME_MAX];
    char name1[RT_NAME_MAX];

	/* 片选引脚初始化 */
	rt_pin_mode(IMU1_CS_Pin, PIN_MODE_OUTPUT);
	rt_pin_mode(IMU2_CS_Pin, PIN_MODE_OUTPUT);
	rt_pin_write(IMU1_CS_Pin, PIN_HIGH);
	rt_pin_write(IMU2_CS_Pin, PIN_HIGH);

    rt_strncpy(name0, IMU1_SPI_DEVICE_NAME, RT_NAME_MAX);
    rt_strncpy(name1, IMU2_SPI_DEVICE_NAME, RT_NAME_MAX);
    
    rt_hw_spi_device_attach("spi1", name0, GPIOC, GPIO_PIN_11);
    rt_hw_spi_device_attach("spi1", name1, GPIOA, GPIO_PIN_4);

    /* spi设备初始化 */
    spi_dev_IMU1 = (struct rt_spi_device *)rt_device_find(name0);
    spi_dev_IMU2 = (struct rt_spi_device *)rt_device_find(name1);
    if ((!spi_dev_IMU1) || (!spi_dev_IMU2))
    {
        rt_kprintf("spi sample run failed! can't find %s device!\n");
    }
	else
	{
		/* config spi */
		struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 10 * 1000 * 1000; /* 10M */
		rt_spi_configure(spi_dev_IMU1, &cfg);
		rt_spi_configure(spi_dev_IMU2, &cfg);

	}
}
