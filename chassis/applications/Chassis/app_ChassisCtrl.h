#ifndef __APP_CHASSISCTRL_H__
#define __APP_CHASSISCTRL_H__
#include <rtthread.h>
#include "mod_motion.h"

typedef rt_uint16_t SerialNum_e;

//底盘控制线程周期，单位ms
#define CHASSIS_CTRL_PERIOD 5

//数据源默认优先级（越大越高）
#define SE_MONITOR_LEVEL 0x100
#define SE_GIMBAL_LEVEL 0x010
#define SE_TEST_LEVEL 0x001

//数据源默认是否有效
#define SE_MONITOR_IF_VALID RT_FALSE
#define SE_GIMBAL_IF_VALID RT_TRUE
#define SE_TEST_IF_VALID RT_FALSE

// Ctrl_data_t结构体赋0初始化；推荐创建局部变量时，使用这种方式
#define CTRL_DATA_INIT_ZERO(mode) \
    {                             \
        mode, 0, 0, 0, 0, 0, 0, 0 \
    }
#define CTRL_DATA_ZERO(name, mode) Ctrl_data_info_t name = CTRL_DATA_INIT_ZERO(mode)

/*调度模式*/
typedef enum
{
    PREEMPTIVE, //抢占式
    FIXATION,   //固定不变式

} Schedule_mode_e;

/*数据源*/
typedef enum
{
    CS_GIMBAL = 0, //云台
    CS_TEST,       //测试模块
    CS_MONITOR,    //监视器
    SOURCE_NUM,    //数据源数量

} Ctrl_source_e;

/*控制输入数据,32字节*/
typedef struct
{
    Motion_mode_e motion_mode;
    Mot_base_t xyw;   // xy平移速度矢量,单位mm/s; 自转角速度,单位0.1°/s
    Vector2_t pos;    //偏心坐标点，单位(mm,mm)
    float dot_angvel; //绕点旋转角速度,单位0.1°/s
    float fol_angle;  //跟随角,单位°，范围(0,360°](俯视图下，底盘车头在云台枪管的逆时针方位 >0)

} Ctrl_data_t;

/*使用结构体+共用体实现，避免输入时三级成员的嵌套*/
typedef struct
{
    Motion_mode_e mode;
    float vel_x;
    float vel_y;
    float angvel;
    float pos_x;
    float pos_y;
    float dot_av;
    float fol_ang;

} Ctrl_data_info_t;

typedef union
{
    Ctrl_data_t data;
    Ctrl_data_info_t info;

} Ctrl_data_u;

/*控制源对象结构体*/
typedef struct
{
    SerialNum_e level;  //优先级,限制小于0x8000有效
    Ctrl_data_u ctrl;   //控制数据
    rt_bool_t if_valid; //是否有效

} Ctrl_source_t;

/*控制调度器结构体*/
typedef struct
{
    Schedule_mode_e ctrl_mode; //调度模式，固定或者抢占
    Ctrl_source_e fix_source;  //当前使用的控制源

} Ctrl_schedule_t;

/**
 * @brief   底盘控制初始化
 * @param   None
 * @return  rt_err_t 是否正常初始化
 * @author  lfp
 */
rt_err_t Chassis_Ctrl_Init(void);

/**
 * @brief   获取数据源最大优先级下标
 * @param   None
 * @return  Ctrl_source_e
 */
Ctrl_source_e Source_Get_MaxPrio(void);

/**
 * @brief   设置数据源优先级
 * @note    尽量不改优先级
 * @param   cs      数据源下标
 * @param   level   优先级
 * @return  None
 */
void Source_Set_Priority(Ctrl_source_e cs, SerialNum_e level);

/**
 * @brief   设置数据源为最大优先级
 * @note    不考虑溢出，尽量不使用该函数
 * @param   cs      数据源下标
 * @return  None
 */
void Source_Set_MaxPrio(Ctrl_source_e cs);

/**
 * @brief   数据源写入数据
 * @param   cs      数据源下标
 * @param   data    控制数据
 * @return  None
 */
void Source_Write_Data(Ctrl_source_e cs, Ctrl_data_info_t info);

/**
 * @brief   设置数据源状态
 * @param   cs      数据源下标
 * @param   state   数据源启动or关闭
 * @return  None
 */
void Source_Set_State(Ctrl_source_e cs, rt_bool_t state);

/**
 * @brief   获得控制模式
 * @param   None
 * @return  Schedule_mode_e
 */
Schedule_mode_e Schedule_Get_CtrlMode(void);

/**
 * @brief   设置控制模式
 * @param   mode    模式
 * @return  None
 */
void Schedule_Set_CtrlMode(Schedule_mode_e mode);

/**
 * @brief   获得固定数据源
 * @param   None
 * @return  Ctrl_source_e
 */
Ctrl_source_e Schedule_Get_FixSource(void);

/**
 * @brief   设置固定数据源
 * @param   source  数据源下标
 * @return  None
 */
void Schedule_Set_FixSource(Ctrl_source_e source);

#endif
