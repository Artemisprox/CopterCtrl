#ifndef __FUNC_GUN_H__
#define __FUNC_GUN_H__

#include "roboselect.h"
#include "drv_StrikeMotor.h" // 使用发射机构电机闭环相关程序

#include "drv_gundata.h" // 枪口热量，射速上限数据

#define DJI_STM32TypeC_USE_5VOUT (0) // 开启大疆C板5V输出
#define DJI_STM32TypeC_5VCTRL_PIN GET_PIN(C, 8)

#define GUN_SPEED_REAL_FULL (300.0f) // 摩擦轮关闭时，自瞄使用趋近于无限大的弹速进行计算，用于测试瞄准方向

#if defined CORE_USING_INFANTRY
// 发弹识别数据
#define SET_SPEDECMIN (75)               // 设置发弹识别阈值：速度突变量
#define SET_RUBSHOOT_COUNT (3)           // 设置发弹识别阈值：计数值
#define SET_MLAUNCH_BOOSTERCHECK_SPE (4) // 单位：弹丸/s
#elif defined CORE_USING_HERO
// 发弹识别数据
#define SET_SPEDECMIN (65)               // 设置发弹识别阈值：速度突变量
#define SET_RUBSHOOT_COUNT (2)           // 设置发弹识别阈值：计数值
#define SET_MLAUNCH_BOOSTERCHECK_SPE (2) // 单位：弹丸/s
#endif
// 机械相关数据
#define SNALL 0
#define FIRE_ANGLE 60

typedef enum
{
    FIRE_OFF = 0,
    FIRE_ON = 1,
} Gun_FireCTRL_E;

// 发射机构模式
typedef enum
{
    GUN_START = 1, // 正在初始化，无有效模式
    GUN_SLOW,      // 低射频模式
#ifndef CORE_USING_HERO
    GUN_FAST, // 高射频模式
#endif        /* CORE_USING_HERO */
} Gun_Mode_E;

// 发射机构总状态
typedef enum
{
    GUN_RST = 1, // 复位状态，需要初始化
    GUN_DONE,    // 发弹状态
} GunState_E;

// 发射动作单步状态
typedef enum
{
    FIRESTEP_OFF
} FireStep_E;

// 发射机构故障状态
typedef enum
{
    GUNERR_NONE = 1, // 正常工作
    GUNERR_STUCK,    // 卡弹
    GUNERR_PWROFF,   // 发射机构断电
} GunERROR_E;

// 拨弹盘对位状态
typedef enum
{
    BOOSTER_RST = 1,
    BOOSTER_WAIT, // 等待检测到发射
    BOOSTER_RDY,
} Booster_State_E;

// 拨弹盘对位状态
typedef enum
{
#if defined CORE_USING_INFANTRY
    NO_STUCK = 0, // 没有出现卡弹
    STUCK_START,  // 开始处理卡弹
    STUCK_ANG_N1, // 第一次反转
    STUCK_ANG_P1, // 第一次正转
    STUCK_ANG_N2, // 第二次反转
    STUCK_ANG_P2, // 第二次正转
    STUCK_RST,    // 电机休息
#elif defined CORE_USING_HERO
    NO_STUCK = 0,  // 没有出现卡弹
    STUCK_STOP,    // 卡弹停止(电机休息)
    STUCK_BACK,    // 卡弹倒转
    STUCK_FORWARD, // 卡弹正转
#endif
} Booster_StuckCtrlState_E;

#if defined CORE_USING_INFANTRY
// 精细发弹时重置发弹的判断标志位
extern void Visual_FineFire_FlagsRenew(int NewFlag);

// 读取视觉新发送来的发弹判断标志位
extern void Visual_FineFire_WriteNewFlag(int NewFlag);

// 开启视觉精细发弹控制
extern void FireCtrl_VisualFineFire_EN(char FineFire_EN);
#endif

// 读取当前发射机构卡弹状态, 返回真代表卡弹
extern int Read_Strike_Motor_Stuck_Status(void);

// 读取当前的枪管射速模式
extern Gun_Mode_E Read_Now_Gun_Mode(void);

// 置1表示允许超热量，置0恢复热量控制
extern char GunSet_OverHeat_PermitFlag;

// 外部接口：开摩擦轮
extern void Gun_RubEnable(void);

// 外部接口：关闭发射机构
extern void Gun_Disable(void);

// 调整弹速设定情况
extern void Gun_SpeedSet(rt_int16_t SpeedSet);

// 设置枪管射频 （部分情况下，如发弹过程中，射频修改有延迟）
extern void Gun_mode_set(Gun_Mode_E Gun_mode_set);

// 设置是否发弹 可直接使用鼠标左键按键数据
extern void Gun_FireSet(Gun_FireCTRL_E Fire_Set);

// 设置是否使用自瞄发弹限制条件
extern void FireCtrl_AimbotLim_Set(char AimbotLim_EN);

// 初始化发射机构
extern rt_err_t Gun_Init(void);
#endif
