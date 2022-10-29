#include "can_receive.h"
#include "HCanID_data.h"
#include "SuperCap_Com.h"
#include "drv_wheel.h"
#include "mod_Monitor.h"
#include "drv_GimMotor.h"
#include "drv_GimbalCom.h"
#include "drv_IMU.h"

/***
 * @brief    根据can1接收到的ID进入不同的处理函数
 * @param    msg can1报文
 * @return   None
 ***/
void can1_rec(struct rt_can_msg *msg)
{
    switch (msg->id)
    {
    case RIGHT_FRONT:
        Refresh_Wheels_Motor(msg, WHEEL_RF);
        return;

    case LEFT_FRONT:
        Refresh_Wheels_Motor(msg, WHEEL_LF);
        return;

    case LEFT_BACK:
        Refresh_Wheels_Motor(msg, WHEEL_LB);
        return;

    case RIGHT_BACK:
        Refresh_Wheels_Motor(msg, WHEEL_RB);
        return;
    case CHASSIS_IMU_DATA1_RX:
        Refresh_Chassis_IMU_Data1(msg);
        return;

    case CHASSIS_IMU_DATA2_RX:
        Refresh_Chassis_IMU_Data2(msg);
        return;

    default:
        return;
    }
}

/***
 * @brief    根据can2接收到的ID进入不同的处理函数
 * @param    msg can2报文
 * @return   None
 ***/
void can2_rec(struct rt_can_msg *msg)
{
    switch (msg->id)
    {
    case YAW_ID:
    case PITCH_ID:
        Refresh_GimbalMotor(msg); //更新云台电机
        return;

    case GIMBAL_CCTRL_RX:
        Refresh_Ctldata(msg); //更新云台端控制数据
        return;

#ifdef USE_CAPACITY
    case SCPR_RX:
        Refresh_Scprdata(msg); //更新超级电容剩余量
        return;
#endif /* USE_CAPACITY */

    case GIMBAL_DATA_RX:
        Refresh_Gimbaldata(msg);
        return;

    case GIMBAL_IMU_DATA_RX:
        Refresh_Gimbal_IMU_Data(msg);
        return;

    default:
        return;
    }
}
