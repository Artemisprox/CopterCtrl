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
static uint8_t read3buffer;

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
    
    rt_hw_spi_device_attach("spi1", name0, GPIOB, GPIO_PIN_1);
    rt_hw_spi_device_attach("spi1", name1, GPIOC, GPIO_PIN_11);

    /* 查找 spi 设备获取设备句柄 */
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
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 10 * 1000 * 1000; /* 10M */
		rt_spi_configure(spi_dev_IMU1, &cfg);
		rt_spi_configure(spi_dev_IMU2, &cfg);

		/* GYRO WRITE */
        msgGYROWrite1.recv_buf   = RT_NULL;
        msgGYROWrite1.length     = 1;
        msgGYROWrite1.cs_take    = 1;
        msgGYROWrite1.cs_release = 0;
        msgGYROWrite1.next       = &msgGYROWrite2;

        msgGYROWrite2.recv_buf   = RT_NULL;
        msgGYROWrite2.length     = 1;
        msgGYROWrite2.cs_take    = 0;
        msgGYROWrite2.cs_release = 1;
        msgGYROWrite2.next       = RT_NULL;
		
		/* ACCL WRITE */
        msgACCLWrite1.recv_buf   = RT_NULL;
        msgACCLWrite1.length     = 1;
        msgACCLWrite1.cs_take    = 1;
        msgACCLWrite1.cs_release = 0;
        msgACCLWrite1.next       = &msgACCLWrite2;

        msgACCLWrite2.recv_buf   = RT_NULL;
        msgACCLWrite2.length     = 1;
        msgACCLWrite2.cs_take    = 0;
        msgACCLWrite2.cs_release = 1;
        msgACCLWrite2.next       = RT_NULL;

		/* GYRO READ */
        msgGYRORead1.recv_buf   = RT_NULL;
        msgGYRORead1.length     = 1;
        msgGYRORead1.cs_take    = 1;
        msgGYRORead1.cs_release = 0;
        msgGYRORead1.next       = &msgGYRORead2;

        msgGYRORead2.send_buf   = RT_NULL;
        msgGYRORead2.length     = 1;
        msgGYRORead2.cs_take    = 0;
        msgGYRORead2.cs_release = 1;
        msgGYRORead2.next       = RT_NULL;

		/* ACCL READ */
        msgACCLRead1.recv_buf   = RT_NULL;
        msgACCLRead1.length     = 1;
        msgACCLRead1.cs_take    = 1;
        msgACCLRead1.cs_release = 0;
        msgACCLRead1.next       = &msgACCLRead2;

    	msgACCLRead2.recv_buf   = &read3buffer;
        msgACCLRead2.send_buf   = RT_NULL;
        msgACCLRead2.length     = 1;
        msgACCLRead2.cs_take    = 0;
        msgACCLRead2.cs_release = 0;
        msgACCLRead2.next       = &msgACCLRead3;
		
        msgACCLRead3.send_buf   = RT_NULL;
        msgACCLRead3.length     = 1;
        msgACCLRead3.cs_take    = 0;
        msgACCLRead3.cs_release = 1;
        msgACCLRead3.next       = RT_NULL;
	}
}
