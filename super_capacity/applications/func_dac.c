#include "func_dac.h"
#include "drv_HW_Select.h"
//使用dac设定充电模块输出电流值，单位：A
float CHG_Set_Curr(float I_Set)
{
    static float I_Set_REM = 10000;
    float dac_set;
    if (I_Set == I_Set_REM)
    {//检查电流设定值是否发生变化
        return I_Set;
    }
    else
    {
        I_Set_REM = I_Set;
    }
    //按照标定数据进行单位换算
    dac_set = (I_Set - CHG_I_SET_FIX_D) / CHG_I_SET_FIX_K;

    //数据限幅
    if (dac_set > HWSETTINGS_DAC_MAX)
    {
        dac_set = HWSETTINGS_DAC_MAX;
    }
    else if (dac_set<0)
    {
        dac_set = 0;
    }

    //dac设定
    User_dac_write(HW_DAC_CHG_SETI_CH, (dac_set - CHG_DAC_SET_FIX_D) / CHG_DAC_SET_FIX_K);
    I_Set = dac_set * CHG_I_SET_FIX_K + CHG_I_SET_FIX_D;// 计算限幅之后实际设定的电流
    return I_Set;
}

//设定供电模块限流设定值
void SPLY_Set_Curr(float I_Set)
{
    static float I_Set_REM = 10000;
    float dac_set;
    if(I_Set==I_Set_REM)
    { //检查电流设定值是否发生变化
        return;
    }
    else
    {
        I_Set_REM = I_Set;
    }
    //按照标定数据进行单位换算
    dac_set = (I_Set - SPLY_I_SET_FIX_D) / SPLY_I_SET_FIX_K;

    //数据限幅
    if (dac_set > HWSETTINGS_DAC_MAX)
    {
        dac_set = HWSETTINGS_DAC_MAX;
    }
    else if (dac_set < 0)
    {
        dac_set = 0;
    }

    //dac设定
    User_dac_write(HW_DAC_SPLY_SETI_CH, (dac_set - SPLY_DAC_SET_FIX_D) / SPLY_DAC_SET_FIX_K);
}
