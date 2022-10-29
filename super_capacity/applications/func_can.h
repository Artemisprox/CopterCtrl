#ifndef __FUN_CAN_H__
#define __FUN_CAN_H__

#include "drv_canthread.h"
#include "func_HW_Pin_Set.h"

#define CAN_ID_CHASSIS (0x095)
#define CAN_ID_SEND_CAPDATA (0x096)

// adc实际使用数据结构体
typedef struct
{
    char NewData;                //标志位，用于标记是否接收到新数据
    char LocalPowerCtrl_Flag;    //标志位, 用于标记是否需要进行本地功率控制
    rt_uint8_t ChargeClose_Flag; //当前云台是否允许充电
    rt_uint16_t Energy_Buff;     //底盘功率缓冲能量数值
    rt_uint16_t Power_Set;       //底盘功率上限设定值

} RoboState_Type;

extern RoboState_Type _RoboState_Data;

extern void CAN1_SEND_CAPState(float CAP_Percentage, rt_uint16_t Power_Now);

#endif
