#include "mod_adc.h"
#include "math.h"

//低电量时剩余能量
static const float CAP_Energy_Low = 0.5f * CAP_VOLTAGE_EPT * CAP_VOLTAGE_EPT;
//电容充满后可供底盘使用的能量
static const float CAP_Energy_FullValid = 0.5f * CAP_VOLTAGE_FULL * CAP_VOLTAGE_FULL - 0.5f * CAP_VOLTAGE_EPT * CAP_VOLTAGE_EPT;
//满电能量
static const float Energy_Full = 0.5f * CAP_VOLTAGE_FULL * CAP_VOLTAGE_FULL;

struct
{
    float I_Obs;                 // 充电电流的观测量
    float E_Obs;                 // 超级电容实际电压的观测值
    float CAP_Res;               // 外部给定超级电容组的内阻
    float CAP_C;                 // 外部给定超级电容组的电容值
    float Observer_Gain;         // 观测器增益
    uint8_t init_flag;           // 本模块是否被初始化
    rt_tick_t Last_Observe_Tick; // 上一次运行状态观测器进行观测的时间
} V_CAP_Obs_Struct               // 用于观测超级电容电压的观测器结构体
    = {.I_Obs = 0.f,
       .E_Obs = CAP_VOLTAGE_EPT,
       .CAP_Res = 0.2f,
       .CAP_C = 6.67f,
       .Observer_Gain = 1.1f,
       .init_flag = 0,
       .Last_Observe_Tick = 0};

// 对超级电容电量进行观测的观测器
void SCAP_Observer(float VCap)
{
    rt_tick_t NowTick = rt_tick_get();
    if (isnan(VCap))
        return;
    // 初始化以后首次运行将实际测量电压输入给观测器
    if (!V_CAP_Obs_Struct.init_flag)
    {
        V_CAP_Obs_Struct.init_flag = 1;
        V_CAP_Obs_Struct.E_Obs = VCap;
        V_CAP_Obs_Struct.Last_Observe_Tick = NowTick;
        return;
    }
    if (NowTick < V_CAP_Obs_Struct.Last_Observe_Tick)
        return;
    V_CAP_Obs_Struct.I_Obs = (VCap - V_CAP_Obs_Struct.E_Obs) / V_CAP_Obs_Struct.CAP_Res;
    V_CAP_Obs_Struct.E_Obs += V_CAP_Obs_Struct.Observer_Gain * V_CAP_Obs_Struct.I_Obs * (NowTick - V_CAP_Obs_Struct.Last_Observe_Tick) / (V_CAP_Obs_Struct.CAP_C * 1000);
    V_CAP_Obs_Struct.Last_Observe_Tick = NowTick;
}

//计算电量百分比
void Get_CAP_Energy(Cap_Energy_Type *CAP_Energy_data)
{
    //当前能量
    float Energy_Now;
    SCAP_Observer(adc_data.UseDat.V_CAP);
    Energy_Now = V_CAP_Obs_Struct.E_Obs * V_CAP_Obs_Struct.E_Obs / 2;
    if (Energy_Now > CAP_Energy_Low)
    {
        //正在正常工作
        CAP_Energy_data->Supply_Ready = CAP_SUPPLY_READY;
        CAP_Energy_data->Energy_Percentage = (Energy_Now - CAP_Energy_Low) / CAP_Energy_FullValid;
        if (CAP_Energy_data->Energy_Percentage > 1)
            CAP_Energy_data->Energy_Percentage = 1;
        if (CAP_Energy_data->Energy_Percentage < 0)
            CAP_Energy_data->Energy_Percentage = 0;
    }
    else
    {
        //电容电量过低，返回百分比时用--当前能量/可工作最低能量表示
        CAP_Energy_data->Supply_Ready = !CAP_SUPPLY_READY;
        CAP_Energy_data->Energy_Percentage = Energy_Now / CAP_Energy_Low; // 计算当前能量占启动输出所需的最低能量的百分比
    }
    CAP_Energy_data->Energy_Percentage_Real = Energy_Now / Energy_Full; // 计算实际电容能量百分比
}
