#ifndef __FUNC_MONHANDLING_H__
#define __FUNC_MONHANDLING_H__

#include <rtthread.h>

/**
 * @brief 机器人云台单片机复位
 * @author fwlh
 */
extern void Robot_Reset_Gimbal(void);

extern rt_err_t Thread_Err_Exception(rt_bool_t status);


#endif

