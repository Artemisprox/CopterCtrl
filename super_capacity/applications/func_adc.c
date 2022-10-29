#include "func_adc.h"

adc_Data_Type adc_data;

//标定数据计算
__INLINE static float ADC_adjust(float dat,float K,float D)
{
    dat -= D;
    if(dat<0)
    {
        return 0;
    }
    else
    {
        return dat / K;
    }
}

//对Adjdat中的数据进行标定计算，存放在UseDat中
void Adjust_Calculate_All()
{
    adc_data.UseDat.I_IN = ADC_adjust(adc_data.AdjDat.I_IN, ADC_SET_I_IN_K, ADC_SET_I_IN_D);
    adc_data.UseDat.V_IN = ADC_adjust(adc_data.AdjDat.V_IN, ADC_SET_V_IN_K, ADC_SET_V_IN_D);
    adc_data.UseDat.V_CAP = ADC_adjust(adc_data.AdjDat.V_CAP, ADC_SET_V_CAP_K, ADC_SET_V_CAP_D);
    adc_data.UseDat.I_CHG_OUT = ADC_adjust(adc_data.AdjDat.I_CHG_OUT, ADC_SET_I_CHG_OUT_K, ADC_SET_I_CHG_OUT_D);
    adc_data.UseDat.I_SPLY_OUT = ADC_adjust(adc_data.AdjDat.I_SPLY_OUT, ADC_SET_I_SPLY_OUT_K, ADC_SET_I_SPLY_OUT_D);
}

//adc采样及计算线程
void adc_DataProcess()
{
        //整个模块总数据
        adc_data.AdjDat.I_IN = adc_read(HW_ADC_NUM_I_IN) * HW_ADC_K_I_IN;
        adc_data.AdjDat.V_IN = adc_read(HW_ADC_NUM_V_IN) * HW_ADC_K_V_IN;
        //超级电容电压
        adc_data.AdjDat.V_CAP = adc_read(HW_ADC_NUM_V_CAP) * HW_ADC_K_V_CAP;

        //超级电容充电升降压模块数据
        adc_data.AdjDat.I_CHG_OUT = adc_read(HW_ADC_NUM_I_CHG_OUT) * HW_ADC_K_I_CHG_OUT;
        // adc_data.AdjDat.I_CHG_IN = adc_read(HW_ADC_NUM_I_CHG_IN) * HW_ADC_K_I_CHG_IN;
        //超级电容输出升降压模块数据
        adc_data.AdjDat.I_SPLY_OUT = adc_read(HW_ADC_NUM_I_SPLY_OUT) * HW_ADC_K_I_SPLY_OUT;
        // adc_data.AdjDat.I_SPLY_IN = adc_read(HW_ADC_NUM_I_SPLY_IN) * HW_ADC_K_I_SPLY_IN;

        //执行标定计算
        Adjust_Calculate_All();
}
