#ifndef INS_FLOW_H
#define INS_FLOW_H

#include <rtthread.h>
#define NUOFDATA 10

typedef union 
{
    float flo[NUOFDATA];
    uint8_t cha[NUOFDATA*4];
}JustFloat;

extern rt_device_t serial;
extern const uint8_t just_float_tail[4];
extern rt_device_t serial;
extern JustFloat ins_flow_data;
extern void Ins_deal(float *accl, float ave_accl,float datarate);
extern int Test_UART_Init(void);
#endif /* INS_FLOW_H */


