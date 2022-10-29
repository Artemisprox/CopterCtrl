#ifndef __DRV_DAC_H__
#define __DRV_DAC_H__

#include <rtthread.h>

    #ifdef RT_USING_DAC1

    //添加drv_dac.c-drv_dac.h时，需要同时向工程�?添加<stm32f1xx_hal_dac_ex.c>�?<stm32f1xx_hal_dac.c>
    //添加此文件后，DAC会�??加入rtt的自动初始化过程

    //用于DAC输出电压换算
    #ifndef HW_VOLTAGE_SUPPLY
        #define HW_VOLTAGE_SUPPLY (3.3F)
    #endif
    
    #define drv_dac_ch1vol_default (0.0F)
    #define drv_dac_ch2vol_default (0.0F)

    extern rt_err_t User_dac_write(rt_int8_t channel, float voltage_set);//设置DAC输出电压�?

    extern float Sply_Voltage;

#endif

#endif
