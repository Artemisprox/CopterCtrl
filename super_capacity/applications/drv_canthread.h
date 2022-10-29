#ifndef __DRV_CANTHREAD_H__
#define __DRV_CANTHREAD_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "drv_thread.h"

/* CAN1外设含自动初始化 */

extern rt_device_t can1_dev;      //CAN 设备句柄

extern int CAN1_Send(rt_uint8_t *DataSource, char DataLength, rt_uint32_t CAN1_Send_ID); //CAN1发送函数

extern int can1_init(void); //CAN初始化，上电后延时一段时间再运行，否则可能因为CAN芯片上电速度慢而卡死

#endif



