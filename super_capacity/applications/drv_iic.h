#ifndef __DRV_IIC_H__
#define __DRV_IIC_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define USE_CUBEMX_IIC (1)

//iic向从机写入，使用Hal库函数实现硬件iic
extern rt_err_t iic_write_reg(rt_uint16_t SLAVE_ADDR, rt_uint8_t cmd, rt_uint8_t dat);

#endif
