/*
 * @Author: your name
 * @Date: 2020-08-22 20:24:42
 * @LastEditTime: 2020-08-25 16:04:31
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: \tfc_dxy\public\drv_canthread.h
 */
#ifndef __DRV_CANTHREAD_H__
#define __DRV_CANTHREAD_H__

#include <rtdevice.h>

extern rt_device_t can1_dev;      //CAN 设备句柄
extern rt_device_t can2_dev;      //CAN 设备句柄

//can1初始化，can1数据处理线程和中断设定
extern int can1_init(void);
//can2初始化，can2数据处理线程和中断设定
extern int can2_init(void);

#endif



