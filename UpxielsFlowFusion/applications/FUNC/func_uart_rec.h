#ifndef __FUNC_UART_REC_H
#define __FUNC_UART_REC_H
#include <rtthread.h>

// 接收此编码器数据使用的串口设备，磁编码器1,2,3分别对应X，Y，Z
#define Encoder_1_DevName "uart3"
#define Encoder_2_DevName "uart4"
#define Encoder_3_DevName "uart1"

// 接收数据长度
#define Data_len 5
#define ENCODER1_FILTERRATE (0.8f)
#define ENCODER2_FILTERRATE (0.8f)
#define ENCODER3_FILTERRATE (0.8f)

// 接收此编码器的数据
typedef __packed struct
{
    uint8_t head[2]; // 0xFD,0xFE
    int16_t raw_angle;
    uint8_t end; // 0xFC
} SendData_t;

extern rt_err_t UART_REC_Init(void);

#endif //__FUNC_UART_REC_H
