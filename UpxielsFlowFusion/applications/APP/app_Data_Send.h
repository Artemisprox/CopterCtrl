#ifndef _APP_DATA_SEND_
#define _APP_DATA_SEND_

#include <rtthread.h>
#define SENDDATA_LEN (2 + 6 + 2)
#define MOD_TOLERANCE 1
// 发送设备
#define SendDev_name "uart2"

// 串口数接收协议
typedef __packed struct
{
    rt_uint8_t head[2]; // 帧头 0xFD,0xFE
    rt_int16_t move_X;  // X轴
    rt_int16_t move_Y;  // Y轴
    rt_int16_t move_Z;  // Z轴
    rt_int16_t Sum;     // 和校验
} UART_Send_Data;

extern rt_err_t UART_Send_Init(void);

#endif //_APP_DATA_SEND_
