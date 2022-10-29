#ifndef __DRV_GIMBALCOM_H__
#define __DRV_GIMBALCOM_H__
#include <rtdevice.h>

#define TX_G_PERIOD 10
#define MSG_READ_2BYTES(lo, hi) ((rt_int16_t)((msg->data[lo] << 8) + msg->data[hi])) //二字节读取宏

// ui选项显示索引
typedef enum
{
    MainMenu = 0,         // 当前处于主菜单
    ChassisModeMenu,      // 当前处于底盘模式菜单
    AimModeMenu,          // 当前处于自瞄模式菜单
    MiscMenu,             // 当前处于杂项菜单
    RobotResetMenu,       // 当前处于单片机复位菜单
    SCAPCtrlMenu,         // 当前处于超级电容控制菜单
    GunSpeedMenu,         // 当前处于弹速设置菜单
    PerformanceMenu,      // 当前处于机器人性能设置菜单
    PerformanceChooseMenu // 当前处于机器人性能选择菜单
} ui_option_display_e;

typedef enum
{
    UI_NO_FOLLOW = 0,     // 不跟随
    UI_FOLLOW_GIMBAL,     // 跟随
    UI_FOLLOWBACK_GIMBAL, // 反向跟随
    UI_SLOW_GYRO,         // 慢陀螺
    UI_FAST_GYRO,         // 快陀螺
    UI_MOVE_BACK,         // 倒车
    UI_CHASS_AUTO,        // 底盘自主
} ui_motion_mode_e;

typedef enum
{
    AIMBOT_MODE = 0,    // 普通自瞄
    AIMBUFF_CONST_MODE, // 小能量机关
    AIMBUFF_VAR_MODE,   // 大能量机关
    ROTATING_OUTPOST,   // 旋转击打前哨战
    STATIC_OUTPOST,     // 静止击打前哨战
    OUTPOST_MODE_F,     // 第三种击打前哨战的模式
    DANGLING_MODE       // 吊射模式
} ui_aimbot_mode_e;

typedef enum
{
    SINGLE_STRIKE = 0x00,
    TRIPLE_STRIKE = 0x01,
} strike_mode_e;

typedef enum
{
    AmmoBooster_Unselected = 0, // 未选择
    Outbreak_Priority,          // 爆发优先
    Cooling_Priority,           // 冷却优先
    BulletSpeed_Priority,       // 弹速优先
} Local_AmmoBooster_Mode_Enum;  // 枪管类型

typedef enum
{
    Chassis_Unselected = 0, // 未选择
    Power_Priority,         // 功率优先
    Blood_Priority,         // 血量优先
} Local_Chassis_Mode_Enum;  // 底盘类型

typedef enum
{
    Visual_Working_EOK = 0, // 工作正常
    Visual_Com_Failure,     // 通信异常
    Visual_Working_Failure, // 工作异常
} Visual_Error_State_Enum;

typedef enum
{
    My_Color_Red = 0,
    My_Color_Blue
} My_Color_Enum; // 当前己方颜色

typedef __packed struct
{
    // 工作状态数据
    unsigned ui_reset : 1;                // 重置 UI 标志位
    unsigned chassis_reset : 1;           // 底盘复位指令
    unsigned current_menu : 4;            // 当前所处菜单
    unsigned motion_mode : 3;             // 底盘模式
    unsigned now_viewing : 1;             // 当前是否在探头模式
    unsigned strike_mode : 1;             // 发射机构模式(高/低射频)
    unsigned magazine_status : 1;         // 弹舱盖设定状态
    unsigned heatlimit_status : 1;        // 热量状态(是否开启热量限制)
    unsigned aimbot_mode : 3;             // 瞄准模式
    unsigned self_color : 1;              // 当前己方颜色
    unsigned power_restrictions_lim : 1;  // 底盘功率限制(是否开启超级电容电量自动节省)
    unsigned rub_started : 1;             // 摩擦轮已开启
    unsigned now_client_control : 1;      // 当前使用客户端模式
    unsigned capacity_close_flag : 1;     // 当前是否关闭超级电容充电
    unsigned force_refsystem_offline : 1; // 当前是否强制裁判系统离线
    unsigned set_chassis_mode : 2;        // 当前设置的本地底盘模式
    unsigned set_ammobooster_mode : 2;    // 当前设置的本地发射机构模式
    unsigned set_level : 2;               // 设置的本地机器人等级
    unsigned tick_now : 8;                // 当前操作系统工作的 Tick/100, 使用时若到达 255 就不再变动
    // 离线模块数据
    unsigned visual_com_online : 1;      // 视觉通信是否正常
    unsigned visual_working_correct : 1; // 视觉工作是否正常
    unsigned yaw_motor_online : 1;       // Yaw 轴电机通信正常
    unsigned pitch_motor_online : 1;     // Pitch 轴电机通信正常
    unsigned right_rub_motor_online : 1; // 右侧摩擦轮电机通信正常
    unsigned left_rub_motor_online : 1;  // 左侧摩擦轮电机通信正常
    unsigned launch_motor_online : 1;    // 播弹盘电机通信正常
    unsigned strike_stuck : 1;           // 发射机构卡弹
} gimbal_msg_t;                          // 杂项数据包

extern rt_tick_t gimbal_msg_freshtick;

extern uint8_t MaxPower;

/**
 * @brief    初始化与云台通信，并控制底盘运动部分
 * @param [in]	无
 * @return   true:初始化成功	false:初始化失败
 */
rt_err_t Gimbal_Com_Init(void);

/***
 * @brief    处理云台发送的运动控制报文
 * @param    msg     can2报文
 * @return   None
 ***/
void Refresh_Ctldata(struct rt_can_msg *msg);

/***
 * @brief    处理云台发送的各类数据报文
 * @param    msg     can2报文
 * @return   None
 ***/
void Refresh_Gimbaldata(struct rt_can_msg *msg);

#endif
