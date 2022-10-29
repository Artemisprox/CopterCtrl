#ifndef __APP_CHARGE_H__
#define __APP_CHARGE_H__

#include "mod_charge.h"
#include "mod_adc.h"
#include "mod_can.h"

#include "func_buzzer.h"

#define POWER_DEFAULT_SET (40)
#define CHARGE_SPEED_SAVE_SET (0.1f)//预留功率余量比例

#define CHARGE_POWER_MAX_SET (150)//超级电容充电功率限幅
#define CHARGE_ENERGYBUFF_MAX (60)//裁判系统缓冲能量上限
#define POWER_TEMP_RSV (25) // 预留的缓冲能量剩余量

extern rt_err_t CAP_Ctrl_App_Init(void);//超级电容总控程序初始化
extern void Robostate_NewData(RoboState_Type *Robostate); //处理新数据

//获取裁判系统通信有效性
rt_err_t Get_CAN_Return_Valid(void);

// 获取当前是否允许进行充电的指令, 返回值为真代表不允许充电
extern rt_uint8_t Get_Charge_Permission(void);

//获取底盘能量缓存数值，用于OLED显示
float Get_EnergyBuff(void);
#endif
