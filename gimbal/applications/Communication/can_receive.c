#include "drv_canthread.h"
#include "robodata.h"
#include "drv_GimbalPublic.h"
#include "drv_Aimbot_Public.h"
#include "drv_IMU.h"
#include "func_gun.h"
#include "func_Aimbot_Com.h"

volatile int Last_CANID; // 用于检查收到的无效报文的ID

void can1_rec(struct rt_can_msg *msg)
{
    switch (msg->id)
    {
    //更新云台电机数据
    case YAW_ID:
        motor_readmsg(msg, &Yaw.dji);
        return;
    case PITCH_ID:
        motor_readmsg(msg, &Pitch.dji);
        return;

    //底盘
    case CHASSIS_REC:
        refresh_heat(msg->data);
        return;
    
#if defined CORE_USING_HERO
    case LAUNCH_ID:
        motor_readmsg(msg, &Read_Gun_Motor(LaunchMotor)->dji);
        return;
#endif

    default:
        Last_CANID = msg->id;
        return;
    }
}

void can2_rec(struct rt_can_msg *msg)
{
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
        motor_readmsg(msg, &Read_Gun_Motor(RubMotorLeft)->dji);
        return;
    case ID_RUB_RIGHT:
        motor_readmsg(msg, &Read_Gun_Motor(RubMotorRight)->dji);
        return;
#if defined CORE_USING_INFANTRY
    case LAUNCH_ID:
        motor_readmsg(msg, &Read_Gun_Motor(LaunchMotor)->dji);
        return;
#endif
    default:
        Last_CANID = msg->id;
        return;
    }
}
