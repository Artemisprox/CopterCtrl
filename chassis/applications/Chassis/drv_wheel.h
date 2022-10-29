#ifndef __DRV_WHEEL_H__
#define __DRV_WHEEL_H__
#include <rtthread.h>
#include <rtdevice.h>
#include "pid.h"

#define W_CAL_PERIOD 1 //车轮pid线程轮询周期，单位ms

#define INSTALL_DIR W_OUTSIDE    //所有电机安装方向
#define LOW_BATTERY_TL 10.0f     //判断超级电容低电量的下界限，单位百分比，0-100
#define LOW_BATTERY_TH 20.0f     //判断超级电容低电量的上界限，单位百分比，0-100
#define RESERVE_BATTERY_TL 30.0f //操作手需要预留电量时留下来的电量下界
#define RESERVE_BATTERY_TH 60.0f //操作手需要预留电量时留下来的电量上界

/* pid参数 (偷懒``汗)*/
#define SPE_PID_PARAMETER_1 20, 0.05, 9, 950, 12000, -12000
#define SPE_PID_PARAMETER_2 17, 0.03, 10, 2000, 9000, -9000
#define SPE_PID_PARAMETER_TEST 25, 0.05, 10, 1000, 10000, -10000

#if defined(WHEEL_MOTOR_3508) //驱动电机使用3508

#define REDUCT_RATIO (3591.0f / 187.0f) //电机减速比
#define RATED_SPEED 9000                //电机额定转速
#define CURRENT_LIMIT 4 * 12288.0f      //最大电流限制，对应实际电流15A，单位 /16384*20 A

#define SPE_PID_PARAMETER_RF SPE_PID_PARAMETER_1
#define SPE_PID_PARAMETER_LF SPE_PID_PARAMETER_1
#define SPE_PID_PARAMETER_LB SPE_PID_PARAMETER_1
#define SPE_PID_PARAMETER_RB SPE_PID_PARAMETER_1

#elif defined(WHEEL_MOTOR_2006) //驱动电机使用2006

#define REDUCT_RATIO (36.0f)   //电机减速比
#define RATED_SPEED 14976      //电机额定转速
#define CURRENT_LIMIT 15000.0f //最大电流限制，对应实际电流15A，单位 /10000*10 A

#define SPE_PID_PARAMETER_RF SPE_PID_PARAMETER_2
#define SPE_PID_PARAMETER_LF SPE_PID_PARAMETER_2
#define SPE_PID_PARAMETER_LB SPE_PID_PARAMETER_2
#define SPE_PID_PARAMETER_RB SPE_PID_PARAMETER_2

#endif

/*电机安转方向*/
typedef enum
{
    W_OUTSIDE, //输出轴朝外
    W_INSIDE   //输出轴朝内

} Wheel_install_e;

/*电机闭环方式*/
typedef enum
{
    MECHANICAL_CALI,
    LOOP_CTRL,
    NO_CTRL, //无闭环

} Crtl_e;

/*车轮位置*/
typedef enum
{
    WHEEL_RF,
    WHEEL_LF,
    WHEEL_LB,
    WHEEL_RB,

    WHEELS_NUM, //车轮个数
} Wheel_local_e;

/*车轮管理结构体*/
typedef struct
{
    Crtl_e ctrl_mode;

    float bound_TL_Low; // 允许使用预留电量时电量判断区间下限
    float bound_TH_Low; // 允许使用预留电量时电量判断区间上限

    float bound_TL_High; // 不允许使用预留电量时电量判断区间下限
    float bound_TH_High; // 不允许使用预留电量时电量判断区间上限

    rt_bool_t if_hysteresis;    //是否在低电量保护
    rt_bool_t if_SC_protect;    //是否开启超级电容低电量保护
    rt_bool_t if_ReserveEnergy; // 是否预留超级电容能量

} Wheels_manage_t;

/**
 * @brief   车轮初始化
 * @param   None
 * @return  None
 * @author  lfp
 */
rt_err_t Wheels_Init(void);

// 使能/失能底盘电机
void Wheel_Enable(int EN);

/**
 * @brief 初始化轮子的机械阻力
 * @author fwlh
 * @param  param            机械阻力参数
 */
extern void Write_MechanicalResistance(void *param);

/**
 * @brief   读取驱动电机can报文
 * @param   msg 反馈报文
 * @param   local 车轮位置
 */
void Refresh_Wheels_Motor(struct rt_can_msg *msg, Wheel_local_e local);

/**
 * @brief   设定轮子的线速度大小
 * @param   local 车轮位置
 * @param   speed 设定速度，>0轮子带着底盘往前转，单位mm/s
 */
void Wheel_Speed_Set(Wheel_local_e local, float speed);

/**
 * @brief   改变车轮的控制模式
 * @param   mode  闭环还是不闭环
 * @return  None
 */
void Wheels_Change_Mode(Crtl_e mode);

/**
 * @brief   设置是否启用超级电容保护模式
 * @param   if_Sc  RT_TRUE,启用；RT_FALSE，关闭
 * @return  None
 */
void Wheels_If_ScProtect(rt_bool_t if_Sc);

/**
 * @brief   获取当前是否在低电量保护
 * @param   None
 * @return  rt_bool_t RT_TRUE,保护；RT_FALSE，未保护
 */
rt_bool_t Wheels_Read_ScProtectState(void);

/**
 * @brief    修改底盘四个电机的pid参数,暂时用在调试上
 * @param    kp-ki-kd    pid参数
 * @return   None
 */
extern void Wheels_Modify_Spid(float kp, float ki, float kd, float i_limit, float out_limit_up, float out_limit_down);

/**
 * @brief   读取轮子的速度设定值
 * @param   local 车轮位置
 */
float Wheel_Read_SetSpeed(Wheel_local_e local);

/**
 * @brief   读取轮子的当前速度
 * @param   local 车轮位置
 */
float Wheel_Read_NowSpeed(Wheel_local_e local);

/**
 * @brief 读取电机的电流设定值
 * @author fwlh
 * @param  local            车轮位置
 * @return float            设定电流
 */
extern float Wheel_Read_SetCurrent(Wheel_local_e local);

/**
 * @brief 读取电机的转速 PID 结构体
 * @author fwlh
 * @param  local            车轮位置
 * @return pid_t*           转速 PID 结构体
 */
extern pid_t *Wheel_Read_PID(Wheel_local_e local);

/**
 * @brief 读取底盘电机当前的离线情况
 * @author fwlh
 * @return rt_uint8_t   从低到高分别为：左前、左后、右后、右前, 对应位置 1 代表离线
 */
extern rt_uint8_t Read_Wheels_Offline_State(void);

#endif
