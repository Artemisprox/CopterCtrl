#ifndef __APP_GETREF_H__
#define __APP_GETREF_H__
#include <rtthread.h>
#include "HRef_ID.h"
#include "mod_RefSystem.h"

/* 英雄弹速上限档位 */
#define HERO_BS_1       10
#define HERO_BS_2       16

/* 步兵弹速上限档位 */
#define INFANTRY_BS_1   15
#define INFANTRY_BS_2   18
#define INFANTRY_BS_3   30


typedef enum
{
    LOW_SPEED = 0,
    MID_SPEED,
    HIGH_SPEED,

} Ref_bullet_speed_mode_e;

typedef enum
{
    REF_ERROR,
    RED_TEAM = 1,
    BLUE_TEAM,
    
} Ref_team_color_e;

typedef enum
{
    REF_OUTPOST,
    REF_SENTRY,
    REF_BASEMENT,
} Ref_Build_e;

typedef struct
{
    rt_uint8_t  robot_if_hurt;        //机器人是否受伤
    rt_uint8_t  build_if_attacked;    //建筑物是否受伤
    rt_uint16_t sentry_hp;            //哨兵血量
    rt_uint16_t outpost_hp;           //前哨站血量
    rt_uint16_t base_hp;              //基地血量

} Ref_Hurt_t;


/**
* @brief    枪口热量冷却加速
*		  	发送频率： 1Hz 周期发送， 所有机器人发送
* @param    None
* @return   剩余弹量
*/
rt_uint8_t Ref_Power_Rune_Buff(void);

/**
* @brief    获取剩余弹量
*		  	发送频率： 10Hz 周期发送， 所有机器人发送
* @param    None
* @return   剩余弹量
*/
rt_uint16_t Ref_AmmoRemain(void);

/**
* @brief    获取上一发子弹射速,发送频率：射击后发送
* @param    None
* @return   单位 cm/s
*/
rt_uint16_t Ref_Bullet_Speed(void);

/**
* @brief    获取枪口上限速度
* @param    None
* @return   单位 m/s
*/
rt_uint16_t Ref_Bullet_Speed_Limit(void);

/**
* @brief    获取弹速档位
* @param    None
* @return   单位 cm/s
*/
rt_uint8_t Ref_Bullet_Speed_Mode(void);

/**
* @brief    获取枪口热量
* @param    None
* @return   原始数据，单位不变
*/
rt_uint16_t Ref_Shooter_Cooling_Heat(void);

/**
* @brief    获取枪口每秒冷却值
* @param    None
* @return   原始数据，单位不变
*/
rt_uint16_t Ref_Shooter_Cooling_Rate(void);

/**
* @brief    获取枪口热量上限
* @param    None
* @return   原始数据，单位不变
*/
rt_uint16_t Ref_Shooter_Cooling_Limit(void);

/**
* @brief    获取红蓝方
* @param    None
* @return   Ref_team_color_e
*/
Ref_team_color_e Ref_Team_Color(void);

/**
* @brief    获取当前机器人ID和客户端ID
* @param    None
* @return   None
*/
void Ref_Robot_ID(void);

/**
* @brief    获取当前底盘输出功率
* @param    None
* @return   单位 W 瓦
*/
float Ref_Chassis_Power(void);

/**
* @brief    获取机器人底盘功率限制上限
* @param    None
* @return   单位 W 瓦
*/
rt_uint16_t Ref_Chassis_Power_Limit(void);

/**
* @brief    获取底盘功率缓冲
* @note 	飞坡根据规则增加至 250J
* @param    None
* @return   单位 J 焦耳
*/
rt_uint16_t Ref_Chassis_Power_Buffer(void);

/**
* @brief    获取伤害状态,发送频率：伤害发生后发送
* @param    None
* @return   ext_robot_hurt_t
*/
ext_robot_hurt_t Ref_Robot_Hurt_Data(void);

/**
* @brief    获取当前机器人是否受伤
* @param    None
* @return   1：受伤，0：未受伤
*/
rt_uint8_t Ref_Get_Robot_If_Hurt(void);

/**
* @brief    设置机器人状态为受伤
* @param    None
* @return   None
*/
void Ref_Robot_Set_Hurt(void);

/**
* @brief    设置机器人状态为未受伤
* @param    None
* @return   None
*/
void Ref_Robot_Reset_Hurt(void);

/**
* @brief    判断当前建筑(前哨站，哨兵，基地)是否掉血
* @param    None
* @return   None
*/
void Ref_Bulid_If_Hurt(ext_game_robot_survivors_t* robot_survivors);

/**
* @brief    获取建筑物是否掉血
* @param    None
* @return   低三位代表前哨站，哨兵，基地。1：掉血，0：不掉血
*/
rt_uint8_t Ref_Get_Bulid_If_Hurt(void);

/***
* @brief 清除建筑受攻击标志位
* @param type : 建筑类型
***/
void Ref_Reset_Bulid_If_Hurt(Ref_Build_e type);

#endif
