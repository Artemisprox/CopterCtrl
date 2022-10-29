#ifndef __MOD_CAN_H__
#define __MOD_CAN_H__

#include "func_can.h"
#include "mod_adc.h"

//初始化CAN接收发送数据处理控制线程
extern rt_err_t CAN_Mod_Init(void);

extern char MOD_CAN_Fresh; // 用于记录CAN工作状态的变量，在mod charge中的看门狗程序中清零

#endif
