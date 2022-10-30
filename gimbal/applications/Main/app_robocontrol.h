#ifndef __APP_ROBOCOUNTROL_H_
#define __APP_ROBOCOUNTROL_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "func_Key_Record.h"
#include "func_KeyCallback.h"

#define GIMBAL_SMOOTHADD_TIME_MAX 60 // 用于设定遥控数据梯形平滑过程的最长时间

extern rt_err_t RoboControl_init(void);

#endif

