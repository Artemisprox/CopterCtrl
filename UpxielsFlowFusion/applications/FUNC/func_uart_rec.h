#ifndef __FUNC_UART_REC_H
#define __FUNC_UART_REC_H
#include <rtthread.h>
#include "func_FlowFusion.h"

#define Flow_front_DevName "uart3"
#define Flow_behind_DevName "uart1"
#define Data_len 14

extern rt_err_t UART_REC_Init(void);
extern void *FlowData_Get(flow_num flow_num);
#endif //__FUNC_UART_REC_H
