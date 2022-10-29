#include "drv_GimMotor.h"
#include "HCanID_data.h"
#include "HChassis_Data.h"
#include "HLxfunc.h"

static Motordata_t pitch_data; // pitch轴当前数据（以水平为0度
static Motordata_t yaw_data;   // yaw轴数据

/**
 * @brief    赋值电机反馈信息
 * @param    message     接收到的数据帧指针
 * @param    motordata   云台电机结构体
 * @return   None
 */
static void Assign_Motor_Data(Motordata_t *motordata, struct rt_can_msg *message)
{
    motordata->angle = (message->data[0] << 8) + message->data[1];   //转子角度
    motordata->speed = (message->data[2] << 8) + message->data[3];   //转子转速
    motordata->current = (message->data[4] << 8) + message->data[5]; //实际电流
    motordata->temperature = message->data[6];                       //温度
    motordata->FreshTick = rt_tick_get();
}

/**
 * @brief    修正电机的编码器数据；修正后角度和速度方向：云台和底盘正方向重合后，底盘逆时针旋转时返回为正
 *           修正后角度数据范围(-8192/2,8192/2]；修正后速度数据范围不变
 * @param    motordata   云台电机结构体
 * @param    zero_pot    编码器零点
 * @param    if_reversal 数据源是否取反
 * @return   None
 */
static void Fix_GimbalMotor(Motordata_t *motordata, rt_uint16_t zero_pot, rt_bool_t if_reversal)
{
    /*转换零点，数据范围(-8192/2,8192/2]*/
    motordata->fix_angle = CIRCLE_SHORTEST_DIS(zero_pot, motordata->angle, 0, 8192);

    /*对yaw angle的增大方向进行矫正*/
    if (if_reversal == RT_TRUE)
    {
        if (motordata->fix_angle != (rt_int16_t)(8192 / 2))
            motordata->fix_angle = -motordata->fix_angle;

        motordata->speed = -motordata->speed;
    }
}

/***
 * @brief    更新云台电机数据，数据范围(-8192/2,8192/2]
 * @note     yaw轴编码器数值：俯视图下底盘车头在云台枪管的逆时针方位 >0。二者重合时 =0.
 * @note     pitch轴编码器数值：云台枪管往上抬 >0，云台枪管水平时 =0.
 * @param    message 云台yaw or pitch电机报文
 * @return   None
 ***/
void Refresh_GimbalMotor(struct rt_can_msg *message)
{
    switch (message->id)
    {
    case YAW_ID:
        Assign_Motor_Data(&yaw_data, message);
        Fix_GimbalMotor(&yaw_data, YAW_ZERO_ANGLE, YAW_ANGLE_REVERSAL);
        break;

    case PITCH_ID:
        Assign_Motor_Data(&pitch_data, message);
        Fix_GimbalMotor(&pitch_data, PITCH_ZERO_ANGLE, PITCH_ANGLE_REVERSAL);
        break;

    default:
        break;
    }
}

/***
 * @brief    获取云台电机角度数据
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-4096,4096]
 ***/
rt_int16_t Get_GM_8192(GimMotor_e kind)
{
    if (kind == GM_YAW)
        return yaw_data.fix_angle;
    else if (kind == GM_PITCH)
        return pitch_data.fix_angle;

    return 0;
}

/***
 * @brief    获取云台电机角度数据
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-180.0°,180.0°]
 ***/
float Get_GM_360f(GimMotor_e kind)
{
    if (kind == GM_YAW)
        return (float)yaw_data.fix_angle / 8192.0f * 360.0f;
    else if (kind == GM_PITCH)
        return (float)pitch_data.fix_angle / 8192.0f * 360.0f;

    return 0;
}

/***
 * @brief    获取云台电机速度数据
 * @param    kind 云台yaw or pitch电机
 * @return   单位：°/s
 ***/
float Get_GM_Speed(GimMotor_e kind)
{
    if (kind == GM_YAW)
        return (float)yaw_data.speed * 6.0f;
    else if (kind == GM_PITCH)
        return (float)pitch_data.speed * 6.0f;

    return 0;
}

/***
 * @brief    获取云台相对于底盘的自转速度; 云台相对于底盘逆时针旋转时，>0
 * @return   单位：0.1°/s
 ***/
float Get_GMtoCH_Speed(void)
{
    return (-Get_GM_Speed(GM_YAW) * 10.0f);
}

/**
 * @brief 获取底盘 Yaw 轴电机的离线状态
 * @author fwlh
 * @return rt_bool_t        返回真代表 Yaw 轴电机离线(200ms)
 */
rt_bool_t Get_Yaw_Motor_Offline_State(void)
{
    return (((yaw_data.FreshTick && (rt_tick_get() - yaw_data.FreshTick > 200))) ? RT_TRUE : RT_FALSE);
}
