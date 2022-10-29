#include "func_can.h"

RoboState_Type _RoboState_Data;

// CAN1接收到底盘报文后进行数据保存
void CAN1REC_ChassisData(struct rt_can_msg *msg)
{
    _RoboState_Data.ChargeClose_Flag = msg->data[0];
    _RoboState_Data.Energy_Buff = (((rt_uint16_t)(msg->data[4])) << 8) | msg->data[5];
    _RoboState_Data.Power_Set = (((rt_uint16_t)(msg->data[6])) << 8) | msg->data[7];
    if (_RoboState_Data.Energy_Buff != (rt_uint16_t)-1)
        _RoboState_Data.LocalPowerCtrl_Flag = 0;
    else
        _RoboState_Data.LocalPowerCtrl_Flag = 1;
    _RoboState_Data.NewData = 1;
}

//发送电容电量数据、底盘实际功率数据（底盘实际功率暂未编写）
void CAN1_SEND_CAPState(float CAP_Percentage, rt_uint16_t Power_Now)
{
    rt_uint8_t Sendbuf[6];
    *(float *)(&(Sendbuf[0])) = CAP_Percentage;
    *(rt_uint16_t *)(&(Sendbuf[4])) = Power_Now;

    //发送电容电量数据
    CAN1_Send(Sendbuf, 6, CAN_ID_SEND_CAPDATA);
}

void LED1_Toggle()
{
    static char LED_REM = 0;
    LED_REM++;
    if (LED_REM == 8)
        HWFUN_LED1_ON;
    else if (LED_REM == 25)
    {
        LED_REM = 0;
        HWFUN_LED1_OFF;
    }
}

// CAN1接收回调函数
void can1_rec(struct rt_can_msg *msg)
{
    rt_uint16_t receive_can_ID;
    //在此处对CAN1接收到的报文进行处理
    switch (msg->id)
    {
    case CAN_ID_CHASSIS:
        CAN1REC_ChassisData(msg);
        LED1_Toggle();
        break;
    // 用于测试超级电容控制板能否收到总线上的数据
    case 0x100:
        receive_can_ID = msg->id;
        break;
    case 0x205:
        receive_can_ID = msg->id;
        break;
    default:
        receive_can_ID = msg->id;
        break;
    }
    (void)receive_can_ID; // 防止编译器报 warning
}
