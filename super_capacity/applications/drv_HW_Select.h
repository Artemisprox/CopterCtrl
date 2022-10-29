#ifndef __DRV_HW_SELECT_H__
#define __DRV_HW_SELECT_H__

#include "drv_test.h"

#define HWS_CONTROLLER 10//选择超级电容控制板编号

#define HWSETTINGS_DAC_MAX 0.8f //DAC设定值限幅：最大15A左右，对应DAC输出电压1.0V

#define HWS_CHG 1//选择升降压模块号

#if (TEST_MEASURE && (USE_MEASURED_DATA == 0))

#define HW_CHIP_INTERNAL_VREF (1.145365f) // 标定时需要先测出此宏定义的数值，并填写实际值，再进行后续标定过程

#define ADC_SET_I_IN_K (1.0f)
#define ADC_SET_I_IN_D (0.0f)
#define ADC_SET_V_IN_K (1.0f)
#define ADC_SET_V_IN_D (0.0f)
#define ADC_SET_V_CAP_K (1.0f)
#define ADC_SET_V_CAP_D (0.0f)
#define ADC_SET_I_SPLY_OUT_K (1.0f)
#define ADC_SET_I_SPLY_OUT_D (0.0f)
#define ADC_SET_I_CHG_OUT_K (1.0f)
#define ADC_SET_I_CHG_OUT_D (0.0f)

//DAC标定数据：（国际单位制）
//X-Y~DAC设定值-DAC实际值
#define CHG_DAC_SET_FIX_K (1.0f)          //斜率
#define CHG_DAC_SET_FIX_D (0.0f)          //截距
#define SPLY_DAC_SET_FIX_K CHG_DAC_SET_FIX_K //斜率
#define SPLY_DAC_SET_FIX_D CHG_DAC_SET_FIX_D //截距

//X-Y~DAC实际值-实际输出电流
#define CHG_I_SET_FIX_K (15.601f)            //斜率
#define CHG_I_SET_FIX_D (0.132f)             //截距
#define SPLY_I_SET_FIX_K CHG_I_SET_FIX_K     //斜率
#define SPLY_I_SET_FIX_D CHG_I_SET_FIX_D     //截距

#else

    //1号超级电容控制板
    #if (HWS_CONTROLLER == 1)
        //adc标定数据 (标定时K取1，D取0，进行测量) 国际单位制
        //X-Y~实际值-程序值
        #define HW_CHIP_INTERNAL_VREF (1.145365f) //需要填写单片机芯片adc内部基准电压的数值，需要使用adc测得的基准电压、结合单片机供电电压计算得到，一般为1.2V左右
        
        #define ADC_SET_I_IN_K (0.9947f)
        #define ADC_SET_I_IN_D (-0.2081f)
        #define ADC_SET_V_IN_K (1.2863f)
        #define ADC_SET_V_IN_D (1.0572f)
        #define ADC_SET_V_CAP_K (0.9554f)
        #define ADC_SET_V_CAP_D (-0.4415f)
        #define ADC_SET_I_SPLY_OUT_K (1.0f)
        #define ADC_SET_I_SPLY_OUT_D (0.0f)
        #define ADC_SET_I_CHG_OUT_K (1.0f)
        #define ADC_SET_I_CHG_OUT_D (0.0f)


        //DAC标定数据：（国际单位制）
        //X-Y~DAC设定值-DAC实际值
        #define CHG_DAC_SET_FIX_K (0.9782f)          //斜率
        #define CHG_DAC_SET_FIX_D (0.0074f)          //截距
        #define SPLY_DAC_SET_FIX_K CHG_DAC_SET_FIX_K //斜率
        #define SPLY_DAC_SET_FIX_D CHG_DAC_SET_FIX_D //截距

    #endif
    //10号超级电容控制板
    #if (HWS_CONTROLLER == 10)
        //adc标定数据 (标定时K取1，D取0，进行测量) 国际单位制
        //X-Y~实际值-程序值
        #define HW_CHIP_INTERNAL_VREF (1.145365f) //需要填写单片机芯片adc内部基准电压的数值，需要使用adc测得的基准电压、结合单片机供电电压计算得到，一般为1.2V左右
        
        #define ADC_SET_I_IN_K (0.9947f)
        #define ADC_SET_I_IN_D (-0.2081f)
        #define ADC_SET_V_IN_K (0.9459f)
        #define ADC_SET_V_IN_D (-2.2863f)
        #define ADC_SET_V_CAP_K (0.9554f)
        #define ADC_SET_V_CAP_D (-0.4415f)
        #define ADC_SET_I_SPLY_OUT_K (1.0f)
        #define ADC_SET_I_SPLY_OUT_D (0.0f)
        #define ADC_SET_I_CHG_OUT_K (1.f)
        #define ADC_SET_I_CHG_OUT_D (0.f)


        //DAC标定数据：（国际单位制）
        //X-Y~DAC设定值-DAC实际值
        #define CHG_DAC_SET_FIX_K (0.9961f)          //斜率
        #define CHG_DAC_SET_FIX_D (-0.0003f)          //截距
        #define SPLY_DAC_SET_FIX_K CHG_DAC_SET_FIX_K //斜率
        #define SPLY_DAC_SET_FIX_D CHG_DAC_SET_FIX_D //截距

    #endif

    //X-Y~DAC实际值-实际输出电流
    #if (HWS_CHG == 1)
        #define CHG_I_SET_FIX_K (16.144f)        //斜率
        #define CHG_I_SET_FIX_D (-0.5062f)         //截距
        #define SPLY_I_SET_FIX_K CHG_I_SET_FIX_K //斜率
        #define SPLY_I_SET_FIX_D CHG_I_SET_FIX_D //截距
    #endif

#endif

// 电容设置
#define CAP_VOLTAGE_EPT (13.f)   //电容最低电压阈值，低于阈值将关闭底盘电机
#define CAP_VOLTAGE_FULL (25.f) //电容满电电压

#endif
