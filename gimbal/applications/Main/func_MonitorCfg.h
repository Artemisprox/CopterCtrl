#ifndef __FUNC_MONITORCFG_H__
#define __FUNC_MONITORCFG_H__

#include <rtthread.h>

/**
 * @brief 机器人云台单片机复位
 * @author fwlh
 */
extern void Robot_Reset_Gimbal(void);

/**
 * @brief 线程出现异常时的通用回调函数
 * @author fwlh
 * @param  status           当前状态, 输入 RT_ERROR 代表需要复位
 * @param  param            传入的参数, 该参数为初始化时的自定义参数
 * @return rt_err_t         返回值, 恒定为 RT_EOK
 */
extern rt_err_t Thread_Err_Exception(rt_bool_t status, void *param);

#endif /* __FUNC_MONITORCFG_H__ */
