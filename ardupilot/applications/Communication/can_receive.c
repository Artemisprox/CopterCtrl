#include "drv_canthread.h"
#include "drv_IMU.h"
#include "drv_battery.h"
#include "mod_recoil_force_compensate.h"
#include "CAN_TEST.h"

volatile int Last_CANID; // 用于检查收到的无效报文的ID

void can1_rec(struct rt_can_msg *msg)
{
    switch (msg->id)
    {
    /*
    //发射机构
    case GIMBAL_ID:
        gun_readmsg(msg->data);
    //电池
    case POWER_ID:
        battery_readmsg(msg->data);
        return;
    
#if defined CORE_USING_HERO
    case LAUNCH_ID:
        motor_readmsg(msg, &Read_Gun_Motor(LaunchMotor)->dji);
        return;
#endif

    default:
        Last_CANID = msg->id;
        return;
    */
    case 0x201:
        motor_readmsg(msg->data, &Read_Gun_Motor()->dji);
        return;
    }
}

void can2_rec(struct rt_can_msg *msg)
{
/*
    switch (msg->id)
    {
    //视觉通信数据接收ID
    case ID_VISUALDATA_AIMFLAGS:
        VisualCom_Receive_Flag(msg->data);
        return;
    case ID_VISUALDATA_GIMBALSET:
        VisualCom_Receive_Atti(msg->data);
        return;
    case ID_RUB_LEFT:
        motor_readmsg(msg->data, &Read_Gun_Motor(RubMotorLeft)->dji);
        return;
    case ID_RUB_RIGHT:
        motor_readmsg(msg->data, &Read_Gun_Motor(RubMotorRight)->dji);
        return;
#if defined CORE_USING_INFANTRY
    case LAUNCH_ID:
        motor_readmsg(msg->data, &Read_Gun_Motor(LaunchMotor)->dji);
        return;
#endif
    default:
        Last_CANID = msg->id;
        return;
    }
*/
}
