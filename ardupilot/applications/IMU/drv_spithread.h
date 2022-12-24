#ifndef __DRV_SPITHREAD_H__
#define __DRV_SPITHREAD_H__

#include <rtdevice.h>
#include <rtthread.h>

#include <board.h>

#define ACCL_CS_Pin GET_PIN(A,4)
#define GYRO_CS_Pin GET_PIN(B,0)

extern struct rt_spi_device *spi_dev_accl;
extern struct rt_spi_device *spi_dev_gyro;
extern void spi_BMI088_init(void);

extern struct rt_spi_message msgACCLRead1, msgACCLRead2, msgACCLRead3;
extern struct rt_spi_message msgACCLWrite1, msgACCLWrite2;
extern struct rt_spi_message msgGYRORead1, msgGYRORead2;
extern struct rt_spi_message msgGYROWrite1, msgGYROWrite2;

#endif



