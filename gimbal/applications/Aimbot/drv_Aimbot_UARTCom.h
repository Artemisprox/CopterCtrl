#ifndef __DRV_AIMBOT_UARTCOM_H__
#define __DRV_AIMBOT_UARTCOM_H__

#include <rtthread.h>

#ifndef AIMBOT_CIMMUNICATION_USING_CAN

/**
 * @brief 用于将待发送的数据以符合通信协议的方式发送出去
 * @author fwlh
 * @param  dev              串口设备
 * @param  ID               信息 ID, 1 代表姿态, 0 代表标志位
 * @param  msg              真实数据(数组名)
 * @param  size             数组中数据的数量
 * @return rt_size_t        返回写入数据的数量
 */
extern rt_size_t Aimbot_Write_UART_Data(rt_device_t *dev, rt_uint8_t ID, rt_uint8_t msg[], rt_size_t size);

/**
 * @brief 初始化与视觉通信的串口
 * @author fwlh
 * @param  aimbot_device    设备指针
 * @param  flag_get         处理标志位报文的函数
 * @param  atti_get         处理云台姿态报文的函数
 * @return rt_err_t         初始化结果
 */
extern rt_err_t Aimbot_UART_Init(rt_device_t *aimbot_device, rt_err_t (*flag_get)(rt_uint8_t rxmsg[]), rt_err_t (*atti_get)(rt_uint8_t rxmsg[]));

#endif /* AIMBOT_CIMMUNICATION_USING_CAN */

#endif /* __DRV_AIMBOT_UARTCOM_H__ */
