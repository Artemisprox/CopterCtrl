#ifndef __MOD_ADC_H__
#define __MOD_ADC_H__

#include "func_adc.h"

#define CAP_SUPPLY_READY (1)

//电容电量状态结构体
typedef struct
{
    char Supply_Ready;              //标志位, 用于标明此时电容电量是否达到阈值
    float Energy_Percentage; //电量百分比, 达到阈值前后分别计算
    float Energy_Percentage_Real;//实际电量百分比，达到阈值前后统一计算
} Cap_Energy_Type;

//获取当前电容电量状态
extern void Get_CAP_Energy(Cap_Energy_Type *);

#endif
