#ifndef __DRV_GIMMOTOR_H__
#define __DRV_GIMMOTOR_H__
#include <rtthread.h>
#include <rtdevice.h>

typedef struct
{
    rt_uint16_t angle;      //转子角度，大小范围[0~8191]
    rt_int16_t speed;       //转子转速，单位rpm
    rt_int16_t current;     //实际转矩电流
    rt_uint8_t temperature; //电机的实际温度

    rt_int16_t fix_angle; //修正过后的角度，大小范围(-4096~4096]
    rt_tick_t FreshTick;  // 数据更新的时刻
} Motordata_t;

typedef enum
{
    GM_YAW,
    GM_PITCH,

} GimMotor_e;

/***
 * @brief    更新云台电机数据，数据范围(-8192/2,8192/2]
 * @note     yaw轴编码器数值：俯视图下底盘车头在云台枪管的逆时针方位 >0。二者重合时 =0.
 * @note     pitch轴编码器数值：云台枪管往上抬 >0，云台枪管水平时 =0.
 * @param    message 云台yaw or pitch电机报文
 * @return   None
 ***/
void Refresh_GimbalMotor(struct rt_can_msg *message);

/***
 * @brief    获取云台电机角度数据
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-4096,4096]
 ***/
rt_int16_t Get_GM_8192(GimMotor_e kind);

/***
 * @brief    获取云台电机角度数据
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-180.0°,180.0°]
 ***/
float Get_GM_360f(GimMotor_e kind);

/***
 * @brief    获取云台电机速度数据
 * @param    kind 云台yaw or pitch电机
 * @return   单位：°/s
 ***/
float Get_GM_Speed(GimMotor_e kind);

/***
 * @brief    获取云台相对于底盘的自转速度; 云台相对于底盘逆时针旋转时，>0
 * @return   单位：0.1°/s
 ***/
float Get_GMtoCH_Speed(void);

/**
 * @brief 获取底盘 Yaw 轴电机的离线状态
 * @author fwlh
 * @return rt_bool_t        返回真代表 Yaw 轴电机离线(200ms)
 */
extern rt_bool_t Get_Yaw_Motor_Offline_State(void);

#endif
