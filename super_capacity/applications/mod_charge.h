#ifndef __MOD_CHARGE_H__
#define __MOD_CHARGE_H__

#include "drv_thread.h"//需要线程优先级设定文件

#include "func_adc.h"//需要使用adc功能驱动
#include "func_dac.h"//需要使用dac功能驱动
#include "func_HW_Pin_Set.h"//需要引脚设置文件
#include "drv_test.h"

#include "drv_thread.h"//需要线程设置文件

#define MOD_CHG_I_SET_START (1.f)//控制未运行时的电流设定值

#define P_FIX_D_SET_DEF (0.10f)// 物理意义：按照功率变化速度进行抑制

#define PCE_CAL_MAX (1.8F) // 设置电源效率计算值上限，数值过大会导致控制震荡
#define PCE_CAL_MIN (0.30F) // 设置电源效率计算值上限，数值过大会导致控制震荡

#define POWER_EST_FILTER_SET (0.8f) // 新数据信任比例

// 启动充电控制，执行后使能充电模块，并进行功率闭环控制
extern int mod_charge_init(float P_set);

// 修改充电功率设定值
void Charge_Ctrl_set_P(float P_Set_New);

//读取当前充电功率设定值
rt_uint16_t Get_Charge_PowerSet(void);

#endif
