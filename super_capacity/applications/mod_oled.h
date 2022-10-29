#ifndef __MOD_OLED_H__
#define __MOD_OLED_H__

#include "func_oled.h"
#include "drv_thread.h"

#define SHOW_BLOCK_POSY (6)
#define SHOW_HERO_RM_POSX (9)
#define SHOW_HERO_RM_POSY (1)
#define SHOW_POWERSET_POSX (93)
#define SHOW_POWERSET_POSY (1)

#define SHOW_SQARETYPE_SOLID 0X0F
#define SHOW_SQARETYPE_HOLLOW 0X09
#define SHOW_SQARETYPE_CLEAR 0X00

//OLED显示程序模块初始化
extern void OLED_Mod_Init(void); 
//刷新进度条图形函数
void OLED_FreshEnergy(float EnergyPercentage, char Sqare_Type);
//刷新缓冲能量进度条图形
void OLED_FreshBuff(float BuffPercentage, char Sqare_Type);
//刷新功率设定值函数
void OLED_Fresh_PowerSet(rt_uint16_t PowerSet);

#endif
