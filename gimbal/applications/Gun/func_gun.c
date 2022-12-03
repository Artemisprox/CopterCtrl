#include "func_gun.h"
#include "robodata.h"

#include "drv_GunSettings.h"
#include "drv_Aimbot_Public.h"

#include "func_KeyCallback.h"
#include "drv_utils.h"
#include "func_GimbalSet.h"

GunState_E GunState;
Gun_Mode_E GunMode_Now = GUN_START;             // 未初始化时没有有效发射机构模式
Booster_State_E BoosterState_Now = BOOSTER_RST; // 拨弹盘默认没有对位

char GunSet_OverHeat_PermitFlag; // 置1表示允许超热量

static float Booster_Zero_POS;

rt_int16_t Fire_LocalCount; // 记录发射出的弹丸数（数据来源：拨弹盘、拨弹对位过程） 用于计算拨弹盘设定值

static rt_int32_t Last_Shoot_Tick = 0;

static Gun_Mode_E Gun_ModeSet_Now;

struct Fire_Flags_s
{
    uint8_t Fire_Flag;           // 开火标志位, 置1连续发弹, 置0停止发弹
    Gun_FireCTRL_E FireFlag_Set; // 记录设定的发弹标志
    uint8_t Aimbot_ShootLim_EN;  // 记录是否需要遵守自瞄给出的发弹限制条件
#if defined CORE_USING_INFANTRY
    Gun_FireCTRL_E FireFlag_Set_Last;        // 上次记录设定的发弹标志
    uint8_t VisualFineFire_Mode;             // 视觉精细控制发弹模式
    uint8_t VisualFineFire_Flag_NowToVisual; // 本次将要发送给视觉的发弹标志位
    uint8_t FireRequire_ToVisual;            // 发送给视觉的发弹请求
    int8_t Fire_Left;                        // 当前剩余的发弹量, 只有在能量机关模式下会被使用
    uint8_t Visual_Enter_Burst;              // 能量机关模式下进入连发模式
#endif
}; // 发弹控制结构体
static struct Fire_Flags_s Fire_Flags =
    {.FireFlag_Set = FIRE_OFF,
     .Fire_Flag = 0,
     .Aimbot_ShootLim_EN = 0,
#if defined CORE_USING_INFANTRY
     .VisualFineFire_Mode = 0,
     .FireRequire_ToVisual = 0,
     .Fire_Left = 0,
     .Visual_Enter_Burst = 0
#endif
};

extern char GunSet_AimbotShootFlag; // 由自瞄文件刷新的自瞄发射限制数据，可在自瞄时控制是否发弹

Gun_Mode_E Read_Now_Gun_Mode(void)
{
    return Gun_ModeSet_Now;
}

void Gun_mode_set(Gun_Mode_E Gun_mode_set)
{
    Gun_ModeSet_Now = Gun_mode_set;
}

#if defined CORE_USING_INFANTRY
// 开启视觉精细发弹控制
void FireCtrl_VisualFineFire_EN(char FineFire_EN)
{
    Fire_Flags.VisualFineFire_Mode = FineFire_EN;
}

// 精细发弹时重置发弹的判断标志位
void Visual_FineFire_FlagsRenew(int NewFlag)
{
    Fire_Flags.VisualFineFire_Flag_NowToVisual = NewFlag;
    Fire_Flags.FireFlag_Set_Last = Fire_Flags.FireFlag_Set;
    Fire_Flags.Fire_Left = 0;
    Fire_Flags.Visual_Enter_Burst = 0;
}
#endif

// 告知本地发射了一发弹丸
static void Visual_Fired_Recheck(void)
{
    Fire_LocalCount++;
#if defined CORE_USING_INFANTRY
    if (Fire_Flags.VisualFineFire_Mode)
    {
        Fire_Flags.VisualFineFire_Flag_NowToVisual ^= 1;
        // 检查现在是不是处于能量机关的连发模式
        if (Fire_Flags.Visual_Enter_Burst)
            --Fire_Flags.Fire_Left;
        else
            Fire_Flags.Fire_Flag = 0;
    }
#endif
}

Gun_Mode_E Gun_Mode_Last; // 用于记录上一次进入自瞄之前的射频模式
// 处理发弹指令
static void FireCmd_Check(void)
{
    // 获得视觉的发弹请求
    FireCtrl_AimbotLim_Set(!Exit_AimbotFlag);

#if defined CORE_USING_INFANTRY
    // 如果现在没有在自瞄模式中, 就记录一下现在的枪管射频模式
    if (Exit_AimbotFlag)
        Gun_Mode_Last = Read_Now_Gun_Mode();
    // 判断自瞄标志位修改射频
    if (!Exit_AimbotFlag && VisualFlag_TargetFound && VisualFlag_BurstShoot)
        Gun_mode_set(GUN_FAST);
    else if (!Exit_AimbotFlag && VisualFlag_TargetFound && !VisualFlag_BurstShoot)
        Gun_mode_set(GUN_SLOW);
    else if (Gun_Mode_Last != Read_Now_Gun_Mode())
        Gun_mode_set(Gun_Mode_Last);
    // 处理FireFlag
    if (!Fire_Flags.VisualFineFire_Mode)
    {
        // 此时不需要与视觉互通发弹信息, 只需要接收视觉给出的发弹指令
        if (Fire_Flags.FireFlag_Set == FIRE_ON)
        {
            // 如果设定的发弹状态为发弹 则检查是否开启了自瞄发弹限制
            if (Fire_Flags.Aimbot_ShootLim_EN)
                Fire_Flags.Fire_Flag = GunSet_AimbotShootFlag;
            else
                Fire_Flags.Fire_Flag = 1;
        }
        else
            Fire_Flags.Fire_Flag = 0;
    }
    // 此时需要与视觉给出的发弹信息做同步, 需要检查视觉现在是否锁定目标
    else if (VisualFlag_TargetFound)
    {
        // 检查现在是不是处于能量机关最后的连发模式
        if (!Fire_Flags.Visual_Enter_Burst)
        {
            // 此时操作手给出的发弹信息只有在跳变沿才能触发, 但是此时需要通知视觉等待视觉发弹
            if ((Fire_Flags.FireFlag_Set_Last == FIRE_OFF) && (Fire_Flags.FireFlag_Set == FIRE_ON))
                Fire_Flags.FireRequire_ToVisual ^= 1; // 告知视觉当前存在发弹请求
            Fire_Flags.FireFlag_Set_Last = Fire_Flags.FireFlag_Set;
            utils_write_bit(&VisualSend_Flags, AimFlag_ShootRequire, Fire_Flags.FireRequire_ToVisual);
            // 读取视觉的发弹命令, 注意这里要取控制精度检查以后的结果
            if (VisualFlag_RuneFire != Fire_Flags.VisualFineFire_Flag_NowToVisual)
						{
                // 没有得到爆发发弹指令, 正常开火
                if (!VisualFlag_RuneBurstShoot)
                    Fire_Flags.Fire_Flag = GunSet_AimbotShootFlag;
                // 得到爆发发弹指令, 连发 5 发后退出能量机关模式
                else
                {
                    Fire_Flags.Visual_Enter_Burst = 1;
                    Fire_Flags.Fire_Flag = 1;
                    Fire_Flags.Fire_Left = 5;
                }
							}
        }
        else
        {
            // 检查当前是否处于可以发弹的状态
            Fire_Flags.Fire_Flag = ((Fire_Flags.Fire_Left > 0) ? 1 : 0);
            // 如果已经结束
            if (Fire_Flags.Fire_Left <= 0)
                // 重新返回正常能量机关
                Fire_Flags.Visual_Enter_Burst = 0;
        }
    }
    // 置位发射标志位
    utils_write_bit(&VisualSend_Flags, AimFlag_Shoot, Fire_Flags.VisualFineFire_Flag_NowToVisual);
#elif defined CORE_USING_HERO
    // 处理FireFlag
    if (Fire_Flags.FireFlag_Set == FIRE_ON)
    {
        // 如果设定的发弹状态为发弹 则检查是否开启了自瞄发弹限制
        if (Fire_Flags.Aimbot_ShootLim_EN)
            Fire_Flags.Fire_Flag = GunSet_AimbotShootFlag;
        else
            Fire_Flags.Fire_Flag = 1;
    }
    else
        Fire_Flags.Fire_Flag = 0;
#endif
}

// 设置是否发弹 可直接使用鼠标左键按键数据
void Gun_FireSet(Gun_FireCTRL_E Fire_Set)
{
    Fire_Flags.FireFlag_Set = Fire_Set;
}

// 设置是否使用自瞄发弹限制条件
void FireCtrl_AimbotLim_Set(char AimbotLim_EN)
{
    Fire_Flags.Aimbot_ShootLim_EN = USE_SHOOT_LIMIT * AimbotLim_EN * VisualFlag_TargetFound;
}

// 刷新弹速设定值
// 输入单位：m/s
extern void Refresh_Muzzle_V(float V_New);
// 调整弹速设定情况
void Gun_SpeedSet(rt_int16_t SpeedSet)
{
    // 修改摩擦轮转速设定值
    Rub_speed_set(SpeedSet);
    // 按照设定的弹速情况向自瞄弹道计算更新弹速数据
    switch (SpeedSet)
    {
    case 0:
        Refresh_Muzzle_V(GUN_SPEED_REAL_FULL);
        break;
#ifndef CORE_USING_HERO
    case GUN_SPEED_SET_15:
        Refresh_Muzzle_V(GUN_SPEED_REAL_15);
        break;
    case GUN_SPEED_SET_18:
        Refresh_Muzzle_V(GUN_SPEED_REAL_18);
        break;
    case GUN_SPEED_SET_30:
        Refresh_Muzzle_V(GUN_SPEED_REAL_30);
        break;
    default:
        Refresh_Muzzle_V(GUN_SPEED_REAL_15);
        break;
#else
    case GUN_SPEED_SET_10:
        Refresh_Muzzle_V(GUN_SPEED_REAL_10);
        break;
    case GUN_SPEED_SET_16:
        Refresh_Muzzle_V(GUN_SPEED_REAL_16);
        break;
    default:
        Refresh_Muzzle_V(GUN_SPEED_REAL_10);
        break;
#endif
    }
}

#if defined CORE_USING_INFANTRY
static char LaunchPIDState = 0;          // 0表示需要闭环 1表示不需要闭环
static rt_int16_t Launch_StateCount = 0; // 用于拨弹电机角度闭环到达设定值一段时间后停止输出

// 控制选择性关闭拨弹电机闭环
static void Launch_PIDOFF_Ctrl(void)
{
    // 检查当前拨弹叉是否到达闭环设定值
    if (LaunchPIDState == 0)
    {
        // 此时电机正在进行闭环，只有角度误差稳定小于0.5度后才可以停止电机输出
        if ((fabsf(Read_Gun_Motor(LaunchMotor)->ang.err) < 0.5f) && (Fire_Flags.Fire_Flag == 0))
        { // 没在发弹且角度误差小
            Launch_StateCount++;
            if (Launch_StateCount > 500)
            { // 计满计数值之后切换到关闭闭环模式
                Launch_StateCount = 0;
                LaunchPIDState = 1;
            }
        }
        else
            // 如果PID不稳定，则清空计数值
            Launch_StateCount = 0;
    }
    else
    {
        // 此时电机没有闭环，如果电机角度误差大，则重启输出
        if (fabsf(Read_Gun_Motor(LaunchMotor)->ang.err) > 1.0f)
        {
            Launch_StateCount++;
            if ((Launch_StateCount > 50) || (fabsf(Read_Gun_Motor(LaunchMotor)->ang.err) > 3.0f))
            {
                // 进行过拨弹操作或较长时间有较大静差，应该启动闭环
                LaunchPIDState = 0;
                Launch_StateCount = 0;
            }
        }
        else
            Launch_StateCount = 0;
    }

    if (LaunchPIDState == 1)
        // 如果现在拨弹电机不需要闭环，则清空计算出的电流设定值
        LaunchMotor_SleepFlag = 1;
    else
        LaunchMotor_SleepFlag = 0; // 除此处外，当需要发弹或拨弹对位时也会开启拨弹PID输出
}
#endif

static char Booster_StuckFlag = 0; // 检测到卡弹时置1 需要在卡弹处理程序中卡弹处理成功时清零
static int16_t Booster_StuckCount = 0;
#if defined CORE_USING_INFANTRY
static float Booster_StuckPOS; // 卡弹位置
static Motor_CtrlMode_E MLaunch_CTRLMode_Rem_Stuck;
static Booster_StuckCtrlState_E Booster_StuckSolveState; // 卡弹处理流程记录标志位
static float MLaunch_ANGSet_Rem;                         // 用于卡弹处理时记录原角度闭环设定值
static float MLaunch_SPESet_Rem;                         // 用于卡弹处理时记录原速度闭环设定值
#elif defined CORE_USING_HERO
static Booster_StuckCtrlState_E Rub_stuck_flag = NO_STUCK; // 摩擦轮卡弹保护
static int Rub_Stuck_Count = 0;
static int Launch_Stuck_Count = 0;
static float RubStuck_SetSpeedRem;
static Booster_StuckCtrlState_E Booster_StuckSolveState; // 卡弹处理流程记录标志位
static float MLaunch_ANGSet_Rem;                         // 用于卡弹处理时记录原角度闭环设定值
#endif

// 读取当前发射机构卡弹状态, 返回真代表卡弹
int Read_Strike_Motor_Stuck_Status(void)
{
    return Booster_StuckCount;
}

// 摩擦轮电机卡弹检查
static void Rub_Motor_Stuck_Check(void)
{
    // 只有英雄需要进行摩擦轮卡弹检查
#if defined CORE_USING_HERO
    if (Rub_stuck_flag == NO_STUCK)
    {
        if (((fabsf(Read_Gun_Motor(RubMotorLeft)->spe.out) > 9000) && (abs(Read_Gun_Motor(RubMotorLeft)->dji.speed) < 10)) ||
            ((fabsf(Read_Gun_Motor(RubMotorRight)->spe.out) > 9000) && (abs(Read_Gun_Motor(RubMotorRight)->dji.speed) < 10)))
        { // 疑似卡弹
            Rub_Stuck_Count++;
            if (Rub_Stuck_Count > 800)
            { // 卡弹
                // 卡弹记录标志位置位
                Booster_StuckFlag = 1;
                Rub_Stuck_Count = 0;
                RubStuck_SetSpeedRem = Read_Gun_Motor(RubMotorLeft)->spe.set; // 记录倒转之前的初始设定转速
                                                                              // 卡弹倒转
#if (GUN_RUB_TOGGLE == 1)
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorLeft), 2000);
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorRight), -2000);
#else
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorRight), 2000);
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorLeft), -2000);
#endif
                Rub_stuck_flag = STUCK_BACK;
            }
        }
        else
        { // 未卡弹
            if (Rub_Stuck_Count > 0)
                Rub_Stuck_Count--;
            else
                Rub_Stuck_Count = 0;
        }
    }
#endif /* CORE_USING_HERO */
}

// 播弹盘电机卡弹检查
static void Launch_Motor_Stuck_Check(void)
{
    // 检查当前卡弹状态
    if (Booster_StuckSolveState == NO_STUCK)
    {
        if ((fabsf(Read_Gun_Motor(LaunchMotor)->spe.out) > 7000) &&
            (abs(Read_Gun_Motor(LaunchMotor)->dji.speed) < 5))
        {
            if (Booster_StuckCount < 200)
                Booster_StuckCount++;
            else
            { // 卡弹
                Booster_StuckCount = 0;

                // 检测到了卡弹，记录相关信息
#if defined CORE_USING_INFANTRY
                Booster_StuckSolveState = STUCK_START;
                MLaunch_CTRLMode_Rem_Stuck = CTRLMode_Motor[(int)LaunchMotor];
                Booster_StuckPOS = Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor));
                MLaunch_ANGSet_Rem = Motor_Read_SetAngle(Read_Gun_Motor(LaunchMotor));
                MLaunch_SPESet_Rem = Motor_Read_SetSpeed(Read_Gun_Motor(LaunchMotor));
#elif defined CORE_USING_HERO
                Booster_StuckSolveState = STUCK_STOP;
                MLaunch_ANGSet_Rem = Motor_Read_SetAngle(Read_Gun_Motor(LaunchMotor));
#endif

                // 卡弹记录标志位置位
                Booster_StuckFlag = 1;
            }
        }
        else
            // 未卡弹
            if (Booster_StuckCount > 0)
                Booster_StuckCount--;
    }
    else
        // 已经检测到卡弹，清空计数值，直接返回
        Booster_StuckCount = 0;
}

// 检测拨弹盘是否出现卡弹
static void Booster_Stuck_Check(void)
{
    // 英雄播弹叉对位时出现疑似卡弹可以不在这里处理
#if defined CORE_USING_HERO
    if (BoosterState_Now != BOOSTER_RDY)
        return;
#endif                          /* CORE_USING_HERO */
    Rub_Motor_Stuck_Check();    // 检查摩擦轮是否出现卡弹
    Launch_Motor_Stuck_Check(); // 播弹盘卡弹检查
}

#if defined CORE_USING_INFANTRY
static rt_int32_t StuckSolve_TickRec; // 记录卡弹处理过程一个状态开始的时刻，用于判断何时切换下一状态
#endif
// 卡弹处理流程
static void Booster_Stuck_Controller(char OverHeatFlag)
{
    rt_int32_t TickNow = rt_tick_get();

#if defined CORE_USING_INFANTRY
    if (OverHeatFlag)
    { // 当前枪口热量紧张，不进行正反转卡弹处理，直接停止拨弹盘闭环即可，等待热量恢复后再处理卡弹
        CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_CLR;
    }
    else
    {                                                     // 当前枪口热量正常，可以进行常规卡弹处理程序
        CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG; // 角度闭环
        switch (Booster_StuckSolveState)
        {
        case STUCK_START:
            // 刚进入卡弹处理程序，准备开始处理卡弹
            StuckSolve_TickRec = TickNow;
            Booster_StuckSolveState = STUCK_ANG_N1; // 第一次反转
            break;
        case STUCK_ANG_N1:
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), Booster_StuckPOS - 20); // 反转20度
            if (TickNow - StuckSolve_TickRec > 150)
            {
                // 时间到，切换下一状态
                Booster_StuckSolveState = STUCK_ANG_P1;
                StuckSolve_TickRec = TickNow;
            }
            break;
        case STUCK_ANG_P1:
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), Booster_StuckPOS + 20); // 正转20度
            if (TickNow - StuckSolve_TickRec > 150)
            {
                // 时间到，切换下一状态
                Booster_StuckSolveState = STUCK_ANG_N2;
                StuckSolve_TickRec = TickNow;
            }
            break;
        case STUCK_ANG_N2:
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), Booster_StuckPOS - 20); // 反转20度
            if (TickNow - StuckSolve_TickRec > 150)
            {
                // 时间到，切换下一状态
                Booster_StuckSolveState = STUCK_ANG_P2;
                StuckSolve_TickRec = TickNow;
            }
            break;
        case STUCK_ANG_P2:
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), Booster_StuckPOS + 20); // 正转20度
            if (TickNow - StuckSolve_TickRec > 150)
            {
                // 时间到，切换下一状态
                Booster_StuckSolveState = STUCK_RST;
                StuckSolve_TickRec = TickNow;
                Booster_StuckPOS = Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)); // 正转卡住的地方应为卡弹位置
            }
            break;
        case STUCK_RST:
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_CLR;
            if (TickNow - StuckSolve_TickRec > 800)
            {
                // 时间到，切换下一状态
                Booster_StuckSolveState = STUCK_ANG_N1;
                StuckSolve_TickRec = TickNow;
            }
            break;

        default:
            Booster_StuckSolveState = STUCK_START;
            break;
        }

        // 判断卡弹是否消除：判断到达卡弹位置后5度位置
        if (Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)) > Booster_StuckPOS + 5)
        {
            Last_Shoot_Tick = TickNow; // 卡弹处理结束，很可能发射出去了一个弹丸，记录当前时间为上次发弹时间
            Booster_StuckFlag = 0;     // 清空卡弹标志位
            // 恢复卡弹前的工作状态
            CTRLMode_Motor[(int)LaunchMotor] = MLaunch_CTRLMode_Rem_Stuck;
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), MLaunch_ANGSet_Rem);
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), MLaunch_SPESet_Rem);
        }
    }
#elif defined CORE_USING_HERO
    // 先处理摩擦轮卡弹
    switch (Rub_stuck_flag)
    {
    default:
    case NO_STUCK:
        break;
    case STUCK_BACK:
        Rub_Stuck_Count++;
        if ((Rub_Stuck_Count > 20) || abs(Read_Gun_Motor(RubMotorLeft)->dji.speed - Read_Gun_Motor(RubMotorRight)->dji.speed) > 1000)
        {
            Rub_Stuck_Count = 0;
#if (GUN_RUB_TOGGLE == 0)
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorLeft), 1000);
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorRight), -1000);
#else
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorLeft), -1000);
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorRight), 1000);
#endif
            Rub_stuck_flag = STUCK_FORWARD; // 正转
        }
        break;
    case STUCK_FORWARD: // 卡弹恢复检查
        Rub_Stuck_Count++;
        if (Rub_Stuck_Count > 50)
        {
            Rub_Stuck_Count = 0;
            if ((abs(Read_Gun_Motor(RubMotorLeft)->dji.speed) < 100) ||
                (abs(Read_Gun_Motor(RubMotorRight)->dji.speed) < 100))
            { // 疑似仍然卡弹
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorLeft), 0);
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorRight), 0);
                Rub_stuck_flag = STUCK_STOP; // 休息一段时间后重试
            }
            else
            {
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorRight), -RubStuck_SetSpeedRem);
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorLeft), RubStuck_SetSpeedRem);
                Rub_stuck_flag = NO_STUCK; // 退出防卡弹
            }
        }
    case STUCK_STOP:
        Rub_Stuck_Count++;
        if (Rub_Stuck_Count > 500)
        {
            Rub_Stuck_Count = 0;
#if (GUN_RUB_TOGGLE == 0)
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorLeft), 2000);
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorRight), -2000);
#else
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorRight), 2000);
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(RubMotorLeft), -2000);
#endif
            Rub_stuck_flag = STUCK_BACK; // 卡弹倒转
        }
        break;
    }
    // 摩擦轮卡弹时不需要处理播弹盘
    if (Rub_stuck_flag != NO_STUCK)
        return;
    if (OverHeatFlag)
    { // 当前枪口热量紧张，不进行正反转卡弹处理，直接停止拨弹盘闭环即可，等待热量恢复后再处理卡弹
        CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_CLR;
    }
    else
    { // 当前枪口热量正常，可以进行常规卡弹处理程序
        switch (Booster_StuckSolveState)
        {
        case STUCK_STOP:
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_CLR;
            Launch_Stuck_Count++;
            if (Launch_Stuck_Count > 500)
            {
                Booster_StuckSolveState = STUCK_BACK;
                Launch_Stuck_Count = 0;
            }
            break;
        case STUCK_BACK:
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), MLaunch_ANGSet_Rem - 40.f);
            Launch_Stuck_Count++;
            if (Launch_Stuck_Count > 100)
            {
                Booster_StuckSolveState = STUCK_FORWARD;
                Launch_Stuck_Count = 0;
            }
            break;
        case STUCK_FORWARD:
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), MLaunch_ANGSet_Rem + 5.f);
            Launch_Stuck_Count++;
            if (Launch_Stuck_Count > 200)
            {
                Booster_StuckSolveState = STUCK_STOP;
                Launch_Stuck_Count = 0;
            }
            // 卡弹恢复检查
            if ((fabs(Read_Gun_Motor(LaunchMotor)->ang.err) < 2.f) && (Read_Gun_Motor(LaunchMotor)->spe.out < 9000))
            {
                if (Launch_Stuck_Count < 2)
                {
                    Launch_Stuck_Count = 0;
                    Booster_StuckSolveState = NO_STUCK; // 退出防卡弹

                    // 恢复卡弹前的工作状态
                    CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
                    Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), MLaunch_ANGSet_Rem);
                }
                else
                    Launch_Stuck_Count -= 2;
            }
            break;

        default:
            Booster_StuckSolveState = STUCK_STOP;
            break;
        }
    }
    // 英雄摩擦轮与播弹盘均不卡弹才认为不卡弹
    if ((Rub_stuck_flag == NO_STUCK) && (Booster_StuckSolveState == NO_STUCK))
        Booster_StuckFlag = 0;
#endif /* CORE_USING_INFANTRY */
}

static int Rub_Shoot_CheckCount = 0; // 用于滞回比较判断是否检测到发弹
static float Rub_Speed_Rec = 0;      // 记录历史转速，用于判断发射时摩擦轮的减速情况
static char Rub_FindShoot = 0;       // 轮询检测的发射检测结果

// 每次摩擦轮检测到发射弹丸时标志位返回1，其它情况返回0
// 由于函数内有滞后滤波计算，调用者需要保持调用周期稳定
static void Rub_Shoot_Check(void)
{
    float Speed_Get;
    // 摩擦轮转速设定值为 0 时不做考虑
    if (!Read_Gun_Motor(RubMotorLeft)->spe.set)
    {
        Rub_Speed_Rec = 0;
        Rub_Shoot_CheckCount = 0;
    }
    else
    {                                                                   // 此时摩擦轮电机已经开启
        Speed_Get = abs(Read_Gun_Motor(RubMotorLeft)->dji.speed);       // 读取摩擦轮转速
        Rub_Speed_Rec = UTILS_LP_FAST(Rub_Speed_Rec, Speed_Get, 0.05f); // 滞后滤波计算

        if (fabsf(Rub_Speed_Rec - Read_Gun_Motor(RubMotorLeft)->spe.set) < SET_SPEDECMIN)
        {
            if (Rub_Speed_Rec > Speed_Get + SET_SPEDECMIN && Speed_Get < fabsf(Read_Gun_Motor(RubMotorLeft)->spe.set))
            {
                // 可能正在发弹，计数
                if (Rub_Shoot_CheckCount < SET_RUBSHOOT_COUNT + 2)
                    Rub_Shoot_CheckCount++;
                else if (Rub_Shoot_CheckCount > 0)
                    Rub_Shoot_CheckCount--;
            }
        }
    }
    // 按照识别数据返回相应数值
    if (Rub_Shoot_CheckCount >= SET_RUBSHOOT_COUNT)
        Rub_FindShoot = 1; // 正在发弹
    else
        Rub_FindShoot = 0; // 没在发弹
}

static rt_int16_t Gun_ShootCount_Remain; // 在发射控制的主体函数之前进行更新 Gun_CTRL_Routine() 中
#if defined CORE_USING_INFANTRY
static char Booster_ANGFix_Flag = 0; // 记录是否正在进行拨弹叉对位，对位时不向热量控制提交拨弹叉位置
#endif

static float Booster_LastShootANG;
// 按照当前电机角度和之前记录的电机角度，完成对位和相关角度数据的修改
static void Booster_FixApply(void)
{
    // 读取当前拨弹叉角度
    Booster_LastShootANG = Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor));
    Booster_Zero_POS = Booster_LastShootANG + (SET_BOOSTER_ADDRATIO * FIRE_ANGLE) - Fire_LocalCount * FIRE_ANGLE; // 计算拨弹叉对位过程中的变化量
}

static char IdatFixFlag;

#if defined CORE_USING_INFANTRY
static int Rub_Enable_CheckCount;
#elif defined CORE_USING_HERO
static int Launch_stuck_count = 0; // 用于记录对位过程中播弹叉是否卡住
#endif

static float BoosterNOWANG;
static char Booster_POSFix_StartRec; // 开始进行拨弹叉对位时候置1，用来保证单点鼠标时候能够直接完成拨弹叉对位过程
// 发射过程逻辑控制 返回值为0时可以切换射频 返回值为1时表示当前正在控制发弹，不能切换射频
static int Gun_ShootCtrl_Slow(void)
{
    rt_int32_t TickNow = rt_tick_get();
    char ShootDelay_OK; // 用于记录当前是否可以打下一发
    char Shoot_Find;

    if ((Last_Shoot_Tick > TickNow - 100) && (TickNow > 100))
    { // 如果上次发弹距离现在的时间短于100ms 则锁I
        if (IdatFixFlag == 0)
        {
            Motor_Write_SpeedPID_Idat_Fix(Read_Gun_Motor(RubMotorLeft));
            Motor_Write_SpeedPID_Idat_Fix(Read_Gun_Motor(RubMotorRight));
            IdatFixFlag = 1;
        }
    }
    else
    { // 恢复正常控制
        if (IdatFixFlag != 0)
        {
            Motor_Write_SpeedPID_Idat_Recover(Read_Gun_Motor(RubMotorLeft));
            Motor_Write_SpeedPID_Idat_Recover(Read_Gun_Motor(RubMotorRight));
            IdatFixFlag = 0;
        }
    }

    // 检查发射情况
    Shoot_Find = Rub_FindShoot;                                                                                                  // 读取发射识别情况
    BoosterNOWANG = ((SET_BOOSTER_ADDRATIO * FIRE_ANGLE) + Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)) - Booster_Zero_POS); // 读取当前拨弹叉相对角度

    if (BoosterNOWANG < 0)
        BoosterNOWANG = 0;

    // 没在进行拨弹叉对位，可以正常向热量控制提交拨弹叉相对角度
    if (BoosterState_Now == BOOSTER_RDY)
        Gun_BoosterPOS_Fresh(BoosterNOWANG / FIRE_ANGLE); // 更新拨弹数据

#if defined CORE_USING_HERO
    (void)Shoot_Find;
    Booster_POSFix_StartRec = 0;
    // 首先进行播弹叉对位的检查
    if (BoosterState_Now != BOOSTER_RDY)
    {
        // 播弹叉对位没有完成时先进行播弹叉对位
        switch (BoosterState_Now)
        {
        case BOOSTER_RST:
            // 拨弹盘没有对位
            // 第一次进入这里，开转速闭环倒转对位
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_SPE;
            Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), -40);
            if ((fabsf(Read_Gun_Motor(LaunchMotor)->spe.out) > 8000) && (abs(Read_Gun_Motor(LaunchMotor)->dji.speed) < 10))
            { // 卡死
                Launch_stuck_count++;
                if (Launch_stuck_count > 120)
                { // 到达堵转点
                    Launch_stuck_count = 0;
                    BoosterState_Now = BOOSTER_WAIT;
                    CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_SPE;
                    Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), 0);
                }
            }
            else
            { // 未卡
                Launch_stuck_count--;
                if (Launch_stuck_count < 0)
                    Launch_stuck_count = 0;
            }
            break;
        case BOOSTER_WAIT:
            // 等待对位完成
            if (abs(Read_Gun_Motor(LaunchMotor)->dji.speed) < 200)
            { // 等待电机放松后静止
                Launch_stuck_count++;
            }
            else
            {
                if (Launch_stuck_count > 0)
                    Launch_stuck_count--;
                else
                    Launch_stuck_count = 0;
            }
            if (Launch_stuck_count > 20)
            { // 对位完成
                Launch_stuck_count = 0;
                Booster_FixApply();
                CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
                Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), Booster_Zero_POS);
                BoosterState_Now = BOOSTER_RDY;
            }
            break;
        default:
            break;
        }
    }
#endif

    // 检测是否需要急停
    if (Gun_ShootCount_Remain < 0)
    {
        // 热量非常危险，立即停转
        CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_SPE; // 切换转速闭环
        Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), 0.0f);

        // 这里不修改角度闭环的设定值，所以以后恢复正常控制时可以直接切回角度闭环，无需修改角度设定值

        return 1; // 停转并直接退出程序
    }

    // 运行到这里说明当前热量没有超限，可发弹量应为>=0的值

    // 检查发射请求
    if ((Fire_Flags.Fire_Flag == 1) || (Booster_POSFix_StartRec == 1))
    { // 需要继续发弹 或 需要进行拨弹叉对位
#if defined CORE_USING_INFANTRY
        LaunchPIDState = 0;
        Launch_StateCount = 0;
        LaunchMotor_SleepFlag = 0; // 准备控制发弹或拨弹对位，确保没有关闭拨弹电机PID输出
#endif
        // 可以打下一发
        if (TickNow - Last_Shoot_Tick > GUN_SLOWMODE_PERIOD)
            ShootDelay_OK = 1;
        // 不能打下一发
        else
            ShootDelay_OK = 0;

        if ((ShootDelay_OK == 1) || (Booster_POSFix_StartRec == 1))
        { // 可以打下一发
            if (Gun_ShootCount_Remain > 0)
            { // 当前热量允许继续发射
                switch (BoosterState_Now)
                {
#if defined CORE_USING_INFANTRY
                case BOOSTER_RST:
                    // 拨弹盘没有对位
                    // 第一次进入这里，开转速闭环
                    Booster_POSFix_StartRec = 1;
                    if ((fabsf(Read_Gun_Motor(RubMotorLeft)->spe.err) < 100) && (Read_Gun_Motor(RubMotorLeft)->spe.set != 0))
                        Rub_Enable_CheckCount++;
                    else if (Rub_Enable_CheckCount > 0)
                        Rub_Enable_CheckCount--;
                    if (Rub_Enable_CheckCount > 250)
                    {
                        Booster_ANGFix_Flag = 1; // 置标志位，用于等待对位发射完成以及调整向热量控制更新拨弹叉位置的过程
                        CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_SPE;
                        Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), SET_MLAUNCH_BOOSTERCHECK_SPE * FIRE_ANGLE / 6); //按照宏定义设置进行转速闭环，用于进行拨弹对位
                        BoosterState_Now = BOOSTER_WAIT;
                        Rub_Enable_CheckCount = 0;
                    }
                    break;

                case BOOSTER_WAIT:
                    // 正在等待发射
                    if (Shoot_Find == 0)
                        // 还没等到发射，继续等待
                        Last_Shoot_Tick = TickNow; // 记录发射时刻
                    else
                    {
                        // 识别到了发射过程
                        Last_Shoot_Tick = TickNow; // 记录发射时刻 用于控制射频
                        Visual_Fired_Recheck();    // 增加一个发弹量
                        // 按照当前电机角度和之前记录的电机角度，完成对位和相关角度数据的修改
                        Booster_FixApply();
                        Booster_ANGFix_Flag = 0;
                        // 正常角度闭环至下一弹丸
                        CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
                        Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), FIRE_ANGLE * Fire_LocalCount + Booster_Zero_POS);

                        Booster_POSFix_StartRec = 0;

                        BoosterState_Now = BOOSTER_RDY; // 拨弹对位完成 恢复正常发弹控制状态
                    }
                    break;
#endif
                case BOOSTER_RDY:
                    // 发射机构对位正常，此时开始发射下一颗弹丸
                    CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG; // 使用角度闭环模式
					     			Booster_FixApply();
                    if (fabsf(Read_Gun_Motor(LaunchMotor)->ang.err) < FIRE_ANGLE * 0.5f)
                    {                           // 若闭环差距不大于0.5颗弹丸，则继续设定角度增量
                        Visual_Fired_Recheck(); // 增加一个发弹量
                        Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), FIRE_ANGLE * Fire_LocalCount + Booster_Zero_POS);
                        Last_Shoot_Tick = TickNow;
                    }
                    break;

                default:
                    BoosterState_Now = BOOSTER_RST;
                    return 1;
                }
            }
            else
                // 当前热量已满，不能继续发射 不修改任何数据，维持现状
                return 1;
        }
        else
        { // 刚刚进行过一次发射，需要等待一段时间再发射下一发
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), FIRE_ANGLE * Fire_LocalCount + Booster_Zero_POS);
            return 1;
        }
    }
#if defined CORE_USING_INFANTRY
    else
    { // 不需要继续发弹控制
        // 下面对电机误差不大的情况进行处理，使得电机只有拨弹瞬间进行闭环
        Launch_PIDOFF_Ctrl();
        return 0;
    }
#endif
    return 0;
}

#if defined CORE_USING_INFANTRY
static char FastShoot_Flag; // 高射频模式下，正在进行转拨弹叉速闭环

// 发射过程逻辑控制 返回值为0时可以切换射频 返回值为1时表示当前正在控制发弹，不能切换射频
static int Gun_ShootCtrl_Fast(void)
{
    rt_int32_t TickNow = rt_tick_get();
    float BoosterNOWANG;
    char Shoot_Find;

    if (Last_Shoot_Tick < TickNow - 100 && TickNow > 100)
    { // 如果上次发弹距离现在的时间短于100ms 则锁I
        if (IdatFixFlag == 0)
        {
            Motor_Write_SpeedPID_Idat_Fix(Read_Gun_Motor(RubMotorLeft));
            Motor_Write_SpeedPID_Idat_Fix(Read_Gun_Motor(RubMotorRight));
            IdatFixFlag = 1;
        }
    }
    else
    { // 恢复正常控制
        if (IdatFixFlag != 0)
        {
            Motor_Write_SpeedPID_Idat_Recover(Read_Gun_Motor(RubMotorLeft));
            Motor_Write_SpeedPID_Idat_Recover(Read_Gun_Motor(RubMotorRight));
            IdatFixFlag = 0;
        }
    }

    // 检查发射情况
    Shoot_Find = Rub_FindShoot; // 读取发射识别情况

    BoosterNOWANG = (SET_BOOSTER_ADDRATIO * FIRE_ANGLE) + Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)) - Booster_Zero_POS; // 读取当前拨弹叉相对角度

    if (BoosterNOWANG < 0)
        BoosterNOWANG = 0;
    if ((Booster_ANGFix_Flag == 0) && (BoosterState_Now == BOOSTER_RDY))
        // 没在进行拨弹叉对位，可以正常向热量控制提交拨弹叉相对角度
        Gun_BoosterPOS_Fresh(BoosterNOWANG / FIRE_ANGLE); // 更新拨弹数据

    // 检测是否需要急停
    if (Gun_ShootCount_Remain < 0)
    {
        // 热量非常危险，立即停转
        CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_SPE; // 切换转速闭环
        Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), 0.0f);

        // 这里不修改角度闭环的设定值，所以以后恢复正常控制时可以直接切回角度闭环，无需修改角度设定值

        return 1; // 急停后直接退出程序
    }

    // 运行到这里说明当前热量没有超限，可发弹量应为>=0的值

    // 检查发射请求
    if ((Fire_Flags.Fire_Flag == 1) || (Booster_POSFix_StartRec == 1))
    { // 需要继续发弹 或 需要进行拨弹叉对位
        LaunchPIDState = 0;
        Launch_StateCount = 0;
        LaunchMotor_SleepFlag = 0; // 准备控制发弹或拨弹对位，确保没有关闭拨弹电机PID输出

        if (Gun_ShootCount_Remain > 0)
        { // 当前热量允许继续发射
            switch (BoosterState_Now)
            {
            case BOOSTER_RST:
                // 拨弹盘没有对位
                // 第一次进入这里，开转速闭环

                Booster_POSFix_StartRec = 1;
                if ((fabsf(Read_Gun_Motor(RubMotorLeft)->spe.err) < 100) && (Read_Gun_Motor(RubMotorLeft)->spe.set != 0))
                    Rub_Enable_CheckCount++;
                else
                {
                    if (Rub_Enable_CheckCount > 0)
                        Rub_Enable_CheckCount--;
                }
                if (Rub_Enable_CheckCount > 10)
                {
                    Booster_ANGFix_Flag = 1; // 置标志位，用于等待对位发射完成以及调整向热量控制更新拨弹叉位置的过程
                    CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_SPE;
                    Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), SET_MLAUNCH_BOOSTERCHECK_SPE * FIRE_ANGLE / 6); //按照宏定义设置进行转速闭环，用于进行拨弹对位
                    Rub_Enable_CheckCount = 0;
                    BoosterState_Now = BOOSTER_WAIT;
                }
                break;

            case BOOSTER_WAIT:
                // 正在等待发射
                if (Shoot_Find == 0)
                {
                    // 还没等到发射，继续等待
                    Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), SET_MLAUNCH_BOOSTERCHECK_SPE * FIRE_ANGLE / 6); //按照宏定义设置进行转速闭环，用于进行拨弹对位
                    Last_Shoot_Tick = TickNow;                                                                            // 记录发射时刻
                }
                else
                {
                    // 识别到了发射过程
                    Last_Shoot_Tick = TickNow; // 记录发射时刻 用于控制射频
                    // 按照当前电机角度和之前记录的电机角度，完成对位和相关角度数据的修改
                    Visual_Fired_Recheck(); // 增加一个发弹量
                    Booster_FixApply();
                    Booster_ANGFix_Flag = 0;
                    // 正常角度闭环至下一弹丸
                    CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
                    Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), FIRE_ANGLE * Fire_LocalCount + Booster_Zero_POS);

                    BoosterState_Now = BOOSTER_RDY; // 拨弹对位完成 恢复正常发弹控制状态

                    Booster_POSFix_StartRec = 0;
                }
                break;

            case BOOSTER_RDY:
                // 发射机构对位正常，热量充足且需要发弹，此时开始发射下一颗弹丸
                CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_SPE;
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), GUN_FASTMODE_FRQ * FIRE_ANGLE / 6);
                Fire_LocalCount = 1 + (rt_int16_t)((Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)) - Booster_Zero_POS) / FIRE_ANGLE);
                FastShoot_Flag = 1;
                break;

            default:
                BoosterState_Now = BOOSTER_RST;
                break;
            }
        }
        else
        {
            if (BoosterState_Now == BOOSTER_RDY)
            {
                // 需要发弹 但当前热量已满，不能继续发射
                // 正常角度闭环至下一弹丸
                if (FastShoot_Flag)
                { // 如果当前正在进行快速发弹（转速闭环）
                    // 按照当前位置就近选择角度闭环位置
                    Fire_LocalCount = 1 + (rt_int16_t)((Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)) - Booster_Zero_POS) / FIRE_ANGLE);
                    FastShoot_Flag = 0;
                    CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
                    Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), FIRE_ANGLE * Fire_LocalCount + Booster_Zero_POS);
                }
            }
            else
            { // 正在进行拨弹叉对位，此时需要暂停对位
                CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_SPE;
                Motor_Write_SetSpeed_ABS(Read_Gun_Motor(LaunchMotor), 0);
            }
        }
        return 1; // 正在发射，不能切换射频，所以返回1
    }
    else
    { // 停止发弹
        // 正常角度闭环
        if (FastShoot_Flag)
        { // 如果当前正在进行快速发弹（转速闭环）
            // 按照当前位置就近选择角度闭环位置
            Fire_LocalCount = 1 + (rt_int16_t)((Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)) - Booster_Zero_POS) / FIRE_ANGLE);
            FastShoot_Flag = 0;
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), FIRE_ANGLE * Fire_LocalCount + Booster_Zero_POS);
        }

        // 下面对电机误差不大的情况进行处理，使得电机只有拨弹瞬间进行闭环
        Launch_PIDOFF_Ctrl();
        return 0;
    }
}
#endif

char Motor_OfflineFlag = 1; // 用于记录拨弹电机是否正常上电 正常时为0
// 发射机构电机离线检查 离线时返回1，正常时返回0
void Gun_MotorOffline_Check(void)
{
    rt_int32_t TickNow;
    TickNow = rt_tick_get();
    if ((TickNow - Read_Gun_Motor(LaunchMotor)->dji.FreshTick > 50) || (TickNow < 100))
        // 50ms没有收到过电机数据，说明电机应该已经离线，需要重新进行拨弹叉对位
        Motor_OfflineFlag = 1;
    else
        Motor_OfflineFlag = 0;
}

// 按照裁判系统的弹速上限更新摩擦轮转速设定值
void RubSpeedFresh(rt_int16_t RefLim)
{
#ifndef CORE_USING_HERO
    if (RefLim < 18)
        Gun_SpeedSet(GUN_SPEED_SET_15);
    else if (RefLim < 30)
        Gun_SpeedSet(GUN_SPEED_SET_18);
    else
        Gun_SpeedSet(GUN_SPEED_SET_30);
#else
    if (RefLim < 16)
        Gun_SpeedSet(GUN_SPEED_SET_10);
    else
        Gun_SpeedSet(GUN_SPEED_SET_16);
#endif
}

// 自动弹速处理程序，如果检测到裁判系统弹速修改，则修改当前设置的弹速
static char SpeedLim_FreshFlag = 1; // 如果检测到弹速修改，但是当前机器人没有开启摩擦轮，则置位等待摩擦轮开启
static rt_int16_t SpeedLim_Rec = 0;
static rt_uint8_t RefSpeedLim_Decrease_Times = 0; // 裁判系统端获取到弹速上限下降的计数值
static void Auto_SpeedUp_Check(void)
{
    float RubSpeed_Get;
    // 读取当前热量上限
    rt_int16_t SpeedLim = Read_Speed_Lim();
    // 已经有记录的裁判系统数据，可以进行弹速升级判断
    if ((SpeedLim_Rec < SpeedLim) || ((SpeedLim_Rec != SpeedLim) && (!Read_RefSpeed_Valid())))
    { // 出现了弹速升级
        SpeedLim_FreshFlag = 1;
        SpeedLim_Rec = SpeedLim;
        RefSpeedLim_Decrease_Times = 0;
    }
    else if (SpeedLim_Rec > SpeedLim)
    {
        // 从裁判系统端获取的弹速下降信息需要进行再三确认
        if (RefSpeedLim_Decrease_Times < 250)
            ++RefSpeedLim_Decrease_Times;
        else
        {
            SpeedLim_FreshFlag = 1;
            SpeedLim_Rec = SpeedLim;
            RefSpeedLim_Decrease_Times = 0;
        }
    }

    if (SpeedLim_FreshFlag)
    {                                       // 出现过弹速升级
        RubSpeed_Get = Rub_speed_ReadSet(); // 读取当前的弹速设定值
        if (RubSpeed_Get != 0)
        { // 当前已经开启摩擦轮
            SpeedLim_FreshFlag = 0;
            RubSpeedFresh(SpeedLim_Rec);
        }
    }
    else
    {
        RubSpeed_Get = Rub_speed_ReadSet(); // 读取当前的弹速设定值
        Gun_SpeedSet((int)RubSpeed_Get);    // 刷新视觉自瞄弹速
    }
}

// 外部接口：开摩擦轮(按照当前裁判系统弹速进行设定)
void Gun_RubEnable(void)
{
    if (SpeedLim_Rec > 0)
        RubSpeedFresh(SpeedLim_Rec);
    else
    { // 如果裁判系统没有数据，则按照最低弹速进行设定
#ifndef CORE_USING_HERO
        Gun_SpeedSet(GUN_SPEED_SET_15);
#else
        Gun_SpeedSet(GUN_SPEED_SET_10);
#endif
    }
}

static uint8_t gun_disable_request = 0;
// 外部接口：关闭发射机构
void Gun_Disable(void)
{
    gun_disable_request = 1;
    Gun_SpeedSet(0);
}

static rt_uint8_t Launch_Rub_CheckErr_Times = 0;
static float LastErrPOS; // 上次认为播弹叉异常时的播弹叉角度
// 本函数用于检查摩擦轮是否在播弹盘转动时仍未启动, 如果存在该现象直接开启摩擦轮
static void Booster_SafeState_Check(void)
{
    if (Rub_speed_ReadSet() == 0)
    {
        // 不允许出现播弹盘在转动但是摩擦轮转速设定值为 0 的情况, 否则开启摩擦轮
        if (Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)) - LastErrPOS > 5)
        {
            if (Launch_Rub_CheckErr_Times < 60)
                Launch_Rub_CheckErr_Times++;
        }
        else if (Launch_Rub_CheckErr_Times > 2)
            Launch_Rub_CheckErr_Times -= 2;
        else
            Launch_Rub_CheckErr_Times = 0;
        // 直接开启摩擦轮
        if (Launch_Rub_CheckErr_Times > 40)
            Gun_RubEnable();
    }
    // 记录当前的播弹叉角度
    else
        LastErrPOS = Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor));
}

// 检查发射机构模块是否需要重置
static void Gun_Reset_Check(void)
{
    if (Motor_OfflineFlag || gun_disable_request)
    { // 此时需要重置枪口数据，下次上电后重新进行拨弹控制和拨弹对位
        GunState = GUN_RST;
        if (!gun_disable_request)
            BoosterState_Now = BOOSTER_RST; // 需要重新进行拨弹对位

        BoosterRec_Reset();
        Fire_LocalCount = 0;
        gun_disable_request = 0; // 清空关闭请求
    }
}

char OverHeatFlag; // 超热量发弹允许标志位
// 枪管热量状态检查
static void Gun_Heat_Status_Check(void)
{
    Gun_ShootCount_Remain = Get_GunShootMAX_Now() - Fire_LocalCount; // 按照热量计算还能发射多少弹丸

    // 检查是否开启不控制热量模式
    if (GunSet_OverHeat_PermitFlag)
        Gun_ShootCount_Remain = 100; // 允许超热量，则此时认为可发射弹丸很多

    if (Gun_ShootCount_Remain <= 0)
        OverHeatFlag = 1;
    else
        OverHeatFlag = 0;
}

char ModeFresh_DisFlag = 0; // 为0时可以切换射频 为1时表示当前正在控制发弹，不能切换射频
// 发射机构发射动作决策
static void Gun_Launch_Action_Decide(void)
{
    // 判断发射机构当前状态
    switch (GunState)
    {
    // 刚上电，拨弹盘刷新设定值
    case GUN_RST:
        // 拨弹盘原地角度闭环
        if (Motor_OfflineFlag == 1)
            // 电机没有正常工作，拨弹不进行闭环控制
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_CLR; // 拨弹叉不闭环
        else
        { // 电机通信已恢复，准备进行正常控制

            // 如果是中途发射机构断电，则需要考虑已经发射的弹丸对应的角度，否则会导致热量控制出现故障
            CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG; // 拨弹盘角度闭环
            Motor_Write_SetAngle_ABS(Read_Gun_Motor(LaunchMotor), Motor_Read_NowAngle(Read_Gun_Motor(LaunchMotor)));
            Booster_StuckFlag = 0; // 此时刚上电不会出现卡弹，之前如果出现了卡弹标志位，则一般为电机没有上电导致
            ModeFresh_DisFlag = 1; // 电机刚上电, 此时不允许切换射频
#if defined CORE_USING_INFANTRY
            // 在这里需要处理拨弹电机相关的标志位，防止发射机构断电重启时程序工作异常
            FastShoot_Flag = 0; // 电机刚上电，没在进行快速发弹
#endif
            GunState = GUN_DONE; // 初始化完成后进入静止状态
        }
        break;
    case GUN_DONE:
        if (Booster_StuckFlag)
        { // 卡弹时 进行卡弹处理程序
            Booster_Stuck_Controller(OverHeatFlag);
            ModeFresh_DisFlag = 1;
        }
        else
#if defined CORE_USING_INFANTRY
            switch (GunMode_Now)
            {
            default:
                GunMode_Now = GUN_SLOW;
                // break; // ! ! ! 这里没有 break; ! ! !
            case GUN_SLOW:
                // 低射频模式
                // 检查热量情况
                ModeFresh_DisFlag = Gun_ShootCtrl_Slow();
                break;
            case GUN_FAST:
                // 高射频模式
                ModeFresh_DisFlag = Gun_ShootCtrl_Fast();
                break;
            }
#elif defined CORE_USING_HERO
            // 可发射状态，调用发射逻辑控制函数
            ModeFresh_DisFlag = Gun_ShootCtrl_Slow();
#endif
        break;
    }
}

// 在drv_strikeMotor轮询执行的发射机构动作规划程序
void Gun_CTRL_Routine(void)
{
    // 检查发射机构电机是否被正常初始化
    if (!Read_Gun_Inited())
        return;
    FireCmd_Check();            // 处理发弹指令
    Auto_SpeedUp_Check();       // 根据裁判系统数据自动处理弹速
    Booster_Stuck_Check();      // 检查是否出现卡弹
    Rub_Shoot_Check();          // 检查是否出现发弹
    Booster_SafeState_Check();  // 摩擦轮不转时判断当前是否需要强制开启摩擦轮
    Gun_MotorOffline_Check();   // 发射机构电机离线情况监视
    Gun_Reset_Check();          // 发射机构模块重置请求检查
    Gun_Heat_Status_Check();    // 热量状态检查
    Gun_Launch_Action_Decide(); // 根据发射机构当前状态进行发射动作决策
    // 发射机构模式切换
    if (ModeFresh_DisFlag == 0)
        if (Gun_ModeSet_Now != GunMode_Now)
            GunMode_Now = Gun_ModeSet_Now;
}

// 初始化发射机构
rt_err_t Gun_Init(void)
{
    // 初始化发射机构电机闭环
    StrikeMotor_init();
#if (DJI_STM32TypeC_USE_5VOUT)
    rt_pin_mode(DJI_STM32TypeC_5VCTRL_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(DJI_STM32TypeC_5VCTRL_PIN, PIN_HIGH);
#endif
    GunData_Init(); // 初始化本地热量
    GunState = GUN_RST;
    GunMode_Now = GUN_SLOW;                                                // 默认低射频
    Gun_ModeSet_Now = GUN_SLOW;                                            // 默认低射频
    BoosterState_Now = BOOSTER_RST;                                        // 刚上电时，拨弹盘处于未对位状态
    SpeedLim_Rec = LairerAttribute[(int)Unselected][(int)Level1].SpeedLim; // 初始化默认弹速上限
    IdatFixFlag = 0;                                                       // 清空锁 I 标志位
    Fire_LocalCount = 0;                                                   // 清空本地发弹数量
    Booster_StuckFlag = 0;                                                 // 清空卡弹标志位
    GunSet_OverHeat_PermitFlag = 0;                                        // 默认不允许超热量
#if defined CORE_USING_INFANTRY
    FastShoot_Flag = 0; // 默认使用低射频
#endif
    Gun_RubEnable(); // 上电默认开启摩擦轮

    CTRLRoutine_Set(Gun_CTRL_Routine); // 设置外部定时调用发射机构计算
    return RT_EOK;
}
