#ifndef __FUNC_ADC_H__
#define __FUNC_ADC_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "drv_thread.h"

#include "drv_adc_user.h"//需要使用adc底层驱动
#include "func_HW_Pin_Set.h"//需要使用引脚设定驱动
#include "drv_HW_Select.h"

//adc基本数据结构体
typedef struct
{
    float I_IN;
    float V_IN;
    float I_CHG_IN;
    float I_CHG_OUT;
    float V_CAP;
    float I_SPLY_IN;
    float I_SPLY_OUT;
} __adc_Type;

//adc实际使用数据结构体
typedef struct
{
    char Ready;                //标志位, 用于标明此时数据是否已经正常写入
    __adc_Type UseDat;         //滤波、标定结果
    __adc_Type AdjDat;      //标定计算临时变量
} adc_Data_Type;

extern adc_Data_Type adc_data;

extern void adc_DataProcess(void);

#endif
