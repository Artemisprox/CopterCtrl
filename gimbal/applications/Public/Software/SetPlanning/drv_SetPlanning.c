#include "drv_SetPlanning.h"

#include "drv_utils.h"

// 执行设定值规划计算
void SetPlanning_Cal(SetPlanning_Str *Str)
{
    // 一些中间变量
    float Accl_Cal;

    // 记录计算时刻，仅供Debug
    Str->CalTick = rt_tick_get();

    // 对输出的位置和速度按照实际状态进行限幅
    if (Str->Output.pos - Str->Input.Now.pos > Str->Settings.POS_Error_Max)
    {
        Str->Output.pos = Str->Input.Now.pos + Str->Settings.POS_Error_Max;
        // Str->Output.spe = 0.f;
    }
    else if (Str->Output.pos - Str->Input.Now.pos < -Str->Settings.POS_Error_Max)
    {
        Str->Output.pos = Str->Input.Now.pos - Str->Settings.POS_Error_Max;
        // Str->Output.spe = 0.f;
    }

    Str->Temp.DeltaSpe = Str->Input.Set.spe - Str->Output.spe;
    Str->Temp.DeltaSpe_2 = Str->Temp.DeltaSpe * Str->Temp.DeltaSpe;
    Str->Temp.DeltaPos = Str->Input.Set.pos - Str->Output.pos;
    Str->Temp.DeltaPos_Temp = Str->Temp.DeltaSpe_2 / (2 * Str->Settings.Accl_Max * 0.95f);

    if (Str->Temp.DeltaPos * Str->Temp.DeltaSpe < 0)
    {
        if (Str->Temp.DeltaPos > 0)
        {
            if (Str->Temp.DeltaPos * 0.9f > Str->Temp.DeltaPos_Temp)
            {
                Accl_Cal = Str->Settings.Accl_Max;
            }
            else
            {
                Accl_Cal = -Str->Settings.Accl_Max;
            }
        }
        else
        {
            if (-Str->Temp.DeltaPos * 0.9f > Str->Temp.DeltaPos_Temp)
            {
                Accl_Cal = -Str->Settings.Accl_Max;
            }
            else
            {
                Accl_Cal = Str->Settings.Accl_Max;
            }
        }
    }
    else
    {
        if (Str->Temp.DeltaPos > 0)
        {
            Accl_Cal = Str->Settings.Accl_Max;
        }
        else
        {
            Accl_Cal = -Str->Settings.Accl_Max;
        }
    }
    Str->Temp.Accl_Now = Accl_Cal;
    Str->Temp.OutputTemp.spe = Str->Temp.Accl_Now * Str->Settings.dt + Str->Output.spe;
    if (Str->Temp.OutputTemp.spe > Str->Settings.Speed_Max)
    {
        Str->Temp.OutputTemp.spe = Str->Settings.Speed_Max;
    }
    else if (Str->Temp.OutputTemp.spe < -Str->Settings.Speed_Max)
    {
        Str->Temp.OutputTemp.spe = -Str->Settings.Speed_Max;
    }

    if ((fabsf(Str->Temp.DeltaPos) > fabsf(Str->Output.spe * Str->Settings.dt)) ||
        (fabsf(Str->Temp.DeltaSpe) > 2 * Str->Settings.Accl_Max * Str->Settings.dt))
    {
        Str->Output.spe = Str->Temp.OutputTemp.spe;
        Str->Output.pos += Str->Output.spe * Str->Settings.dt;
    }
    else
    {
        Str->Output.pos = Str->Input.Set.pos;
        Str->Output.spe = Str->Input.Set.spe;
    }
}

// 直接修改规划结果
void SetPlanning_SetOutput(SetPlanning_Str *Str, float PosSet)
{
    Str->Output.pos = PosSet;
}

// 初始化规划结构体（结构体, PID最大Error, 调整最大加速度, 计算周期）
void SetPlanning_Init(SetPlanning_Str *Str, SetPlanSettings_Str *Settings)
{
    // 初始化结构体内容
    Str->CalTick = 0;
    Str->Input.Now.pos = 0;
    Str->Input.Now.spe = 0;
    Str->Input.Set.pos = 0;
    Str->Input.Set.spe = 0;
    Str->Output.pos = 0;
    Str->Output.spe = 0;
    Str->Settings.Accl_Max = Settings->Accl_Max;
    Str->Settings.dt = Settings->dt;
    Str->Settings.POS_Error_Max = Settings->POS_Error_Max;
    Str->Settings.Speed_Max = Settings->Speed_Max;
}
