#ifndef __DRV_SPITHREAD_H__
#define __DRV_SPITHREAD_H__

#include <rtdevice.h>
#include <rtthread.h>

#include <board.h>

#define IMU1_CS_Pin GET_PIN(B,1)
#define IMU2_CS_Pin GET_PIN(C,11)

extern struct rt_spi_device *spi_dev_IMU1;
extern struct rt_spi_device *spi_dev_IMU2;
extern void spi_ICM20602_init(void);

extern struct rt_spi_message msgACCLRead1, msgACCLRead2, msgACCLRead3;
extern struct rt_spi_message msgACCLWrite1, msgACCLWrite2;
extern struct rt_spi_message msgGYRORead1, msgGYRORead2;
extern struct rt_spi_message msgGYROWrite1, msgGYROWrite2;

#endif
