#ifndef __MOD_MOTION_H__
#define __MOD_MOTION_H__
#include <rtthread.h>
#include "drv_vector.h"
#include "drv_GimMotor.h"
#include "HLxfunc.h"
#include "drv_MecanOmni.h"

/*
 * 注释缩写指代：
 * Mot->Motion;         incre->increment;      fol->follow;             Vel->velocity             pos->position
 * Chass->chassis;      Gim->Gimbal;           Gc,GC->Gimbal Chassis    MotPack->Motion Module Pack运动模块包
 * Eccentric,offset,off_set->偏心,偏离中心;     angvel->角速度;           MotModule->运动模块       av->angle velocity
 * VxyW,xyw->速度矢量的xy分量和角速度w;          Pxy->位置坐标点(x,y)      ExMotMod->External Motion Module
 */

//输入限幅参数
#define M_MAX_YSPEED        4000        //底盘最大移动速度，单位mm/s
#define M_MAX_XSPEED        4000        
#define M_MAX_ROTATE_AC     4500        //底盘最大自旋角速度，单位0.1°/s
#define M_MAX_POSITION_Y    2000        //旋转轴坐标最大值，单位mm
#define M_MAX_POSITION_X    2000        
#define M_MAX_REVOLVE_AC    3600        //底盘最大公转角速度，单位0.1°/s


//模块宏开关
#define SMOOTH_VELOCITY_ENABLE          //使能速度平滑
#define SMOOTH_ANGVEL_ENABLE            //使能角速度平滑
#define LOW_POWER_PROTECT_ENABLE        //低功耗保护
#define GIMBAL_CHASSIS_ENABLE           //云台底盘夹角使能


#if defined(SMOOTH_VELOCITY_ENABLE)
    #define SMOOTH_INPUTXY(Vxy)     MotModule_Smooth_InputXY(Vxy)
#else
    #define SMOOTH_INPUTXY(Vxy)     Vxy
#endif
#if defined(SMOOTH_ANGVEL_ENABLE)
    #define SMOOTH_OUTPUTW(angvel)  MotModule_Smooth_OutputW(angvel)
#else
    #define SMOOTH_OUTPUTW(angvel)  angvel
#endif

/*重命名函数,mod_motion层实现一个兼容
数据范围(-180.0°,180.0°]*/
#if defined(GIMBAL_CHASSIS_ENABLE)
    #define MotModule_Get_YawGc()    Get_GM_360f(GM_YAW)
#else
    #define MotModule_Get_YawGc()    0
#endif
#define MotModule_Resolve(x, y)     MecanOmni_Resolve(*(MecanOmni_t *)&x, y)//MecanOmni_t类型成员与Mot_base_t一致

/*运动模式*/
typedef enum
{
	SMALL_TOP = 0,						//模式：小陀螺 || 当angvel输入为0时,为不跟随模式
	FOLLOW_GIMBAL = 1,					//模式：底盘跟随云台 
	CHASSIS_ONLY = 2,					//模式：直接控制底盘 
	CHASSIS_NOCTRL = 3,					//模式：底盘无闭环控制 
	OFFSET_ONLY = 4,					//模式：偏心 +只有底盘 
	OFFSET_TOP = 5,						//模式：偏心 +小陀螺 
	OFFSET_FOLLOW = 6,					//模式：偏心 +跟随 
	SPIN_XY = 7,						//模式：绕 xy点旋转
	CHASSIS_STOP = 8,					//模式：底盘停止运动

	MOTION_MODE_NUM,
} Motion_mode_e;


typedef struct
{
    Vector2_t   vel;        //平移速度矢量，单位mm/s
    float       angvel;     //角速度，单位0.1°/s

} Mot_base_t;

/*公转运动录制结构体,创建时成员值为0即可*/
typedef struct 
{
    Vector2_t       last_pos;
    float           last_angvel;
    float           incre_theta;
    Tick_probe_t    tick;

} Mot_Record_t;


Mot_base_t MotPack_Only_Chass(Mot_base_t VxyW);
Mot_base_t MotPack_Small_Top(Mot_base_t VxyW);
Mot_base_t MotPack_Follow_Gim(Mot_base_t VxyW,float fol_angle);
Mot_base_t MotPack_Offset_OnlyChass(Mot_base_t VxyW,Vector2_t pos);
Mot_base_t MotPack_Offset_SmallTop(Mot_base_t VxyW,Vector2_t pos);
Mot_base_t MotPack_Offset_FollowGim(Mot_base_t VxyW,Vector2_t pos,float fol_angle);
Mot_base_t MotPack_Spin_Dot(Vector2_t pos,float dot_angvel);
void MotModule_Modify_Apid(float kp,float ki,float kd);
Vector2_t ExMotMod_Get_xyDelta(void);
Mot_base_t ExMotMod_Input(Mot_base_t VxyW,Motion_mode_e mode);
void ExMotMod_Output(Mot_base_t VxyW,Motion_mode_e mode,float fol_angle);


#endif
