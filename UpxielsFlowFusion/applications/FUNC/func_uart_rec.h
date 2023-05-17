#ifndef __FUNC_UART_REC_H
#define __FUNC_UART_REC_H
#include <rtthread.h>
#include "func_FlowFusion.h"

// 接收此编码器数据使用的串口设备，磁编码器1,2,3分别对应X，Y，Z
#define Flow_front_DevName "uart3"
#define Flow_behind_DevName "uart4"

// 接收数据长度
#define Data_len 14
#define ENCODER1_FILTERRATE (0.8f)
#define ENCODER2_FILTERRATE (0.8f)
#define ENCODER3_FILTERRATE (0.8f)

extern rt_err_t UART_REC_Init(void);
extern void *FlowData_Get(flow_num flow_num);
#endif //__FUNC_UART_REC_H
