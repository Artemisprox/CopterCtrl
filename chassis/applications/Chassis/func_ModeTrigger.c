#include "func_ModeTrigger.h"
#include "drv_smooth.h"
#include "drv_wheel.h"

static Mode_trigger_t modetrig[MOTION_MODE_NUM] = 
{
    [SMALL_TOP...CHASSIS_STOP].Entry_Mode = RT_NULL,
    [SMALL_TOP...CHASSIS_STOP].Exit_Mode  = RT_NULL,
    [FOLLOW_GIMBAL].Entry_Mode  = Follow_Gimbal_Entry ,
    [FOLLOW_GIMBAL].Exit_Mode   = Follow_Gimbal_Exit  ,
    [OFFSET_FOLLOW].Entry_Mode  = Follow_Gimbal_Entry ,
    [OFFSET_FOLLOW].Exit_Mode   = Follow_Gimbal_Exit  ,                                        
    [CHASSIS_NOCTRL].Entry_Mode = Chassis_NoCtrl_Entry,
    [CHASSIS_NOCTRL].Exit_Mode  = Chassis_NoCtrl_Exit,
    [SPIN_XY].Exit_Mode = SpinXY_Stop_Exit,
    [CHASSIS_STOP].Exit_Mode = SpinXY_Stop_Exit,
};


/**
* @brief    NoCtrl模式进入函数
*/
static void Chassis_NoCtrl_Entry(void)
{
    Wheels_Change_Mode(NO_CTRL);
}

/**
* @brief    NoCtrl模式退出函数
*/
static void Chassis_NoCtrl_Exit(void)
{
    Wheels_Change_Mode(LOOP_CTRL);
    Smooth_Refresh_Nowvalue(WHEEL_XSPEED,0);//重置平滑值，默认退出时，xy速度为0
    Smooth_Refresh_Nowvalue(WHEEL_YSPEED,0);
}

/**
* @brief    Follow_Gimbal模式进入函数
*/
static void Follow_Gimbal_Entry(void)
{
    Smooth_Modify_Add(WHEEL_ACSPEED,SMOOTH_ADD_UP_AC_2,SMOOTH_ADD_DOWN_AC_2);//改大参数
}

/**
* @brief    Follow_Gimbal模式退出函数
*/
static void Follow_Gimbal_Exit(void)
{
    Smooth_Modify_Add(WHEEL_ACSPEED,SMOOTH_ADD_UP_AC_1,SMOOTH_ADD_DOWN_AC_1);//改小参数
}

/**
* @brief    Spin_XY,Stop模式退出函数
*/
static void SpinXY_Stop_Exit(void)
{
    Smooth_Refresh_Nowvalue(WHEEL_XSPEED,0);//重置平滑值，默认退出时，xy速度为0
    Smooth_Refresh_Nowvalue(WHEEL_YSPEED,0);
}


/**
* @brief    模式触发调度
* @param    now_mode    当前模式
* @return   None
* @author   lfp
*/
void ModeTrig_Scheduler(Motion_mode_e now_mode)
{
    static Motion_mode_e last_mode;

    //轮询所有模式
    for(int mm = 0; mm < MOTION_MODE_NUM; mm++)
    {
        //根据当前模式和上一次模式，判断跳变沿
        if(now_mode == mm && last_mode != mm){
            if(modetrig[mm].Entry_Mode != RT_NULL)
                (*modetrig[mm].Entry_Mode)();           //生成进入函数
        }
        else if(now_mode != mm && last_mode == mm){
            if(modetrig[mm].Exit_Mode != RT_NULL)
                (*modetrig[mm].Exit_Mode)();            //生成退出函数
        }
    }

    //赋值上一次模式
    if(now_mode != last_mode)
        last_mode = now_mode;
}

