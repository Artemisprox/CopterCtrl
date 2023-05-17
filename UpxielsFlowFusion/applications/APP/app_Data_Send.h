#ifndef _APP_DATA_SEND_
#define _APP_DATA_SEND_

#include <rtthread.h>
#define SENDDATA_LEN 14
#define MOD_TOLERANCE 1
// 发送设备
#define SendDev_name "uart2"

// 串口数接收协议
typedef __packed struct
{
    rt_uint8_t head[2]; // 帧头 0xFE,0x0A
		rt_uint8_t flow_x_integral[2];
    rt_uint8_t flow_y_integral[2];
    rt_uint8_t integration_timespan[2];
		rt_uint8_t gound_distance[2];
		rt_uint8_t vaild;
		rt_uint8_t version;
		rt_uint8_t xor_check;
		rt_uint8_t end;
} UART_Send_Data;

extern rt_err_t UART_Send_Init(void);

#endif //_APP_DATA_SEND_
