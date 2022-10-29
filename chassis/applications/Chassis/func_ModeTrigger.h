#ifndef __FUNC_MODETRIGGER_H__
#define __FUNC_MODETRIGGER_H__
#include "app_ChassisCtrl.h"


/*模式触发结构体*/
typedef struct
{
	void (*Entry_Mode)(void);
	void (*Exit_Mode)(void);

} Mode_trigger_t;


/*静态函数声明*/
static void Chassis_NoCtrl_Entry(void);
static void Chassis_NoCtrl_Exit(void);
static void Follow_Gimbal_Entry(void);
static void Follow_Gimbal_Exit(void);
static void SpinXY_Stop_Exit(void);


/**
* @brief    模式触发调度
* @param    now_mode    当前模式
* @return   None
* @author   lfp
*/
void ModeTrig_Scheduler(Motion_mode_e now_mode);


#endif

