#ifndef __FUNC_MODCTR_H_
#define __FUNC_MODCTR_H_

#include <rtdef.h>
#include <drv_remote.h>

typedef enum
{
    NO_FOLLOW = 0,     // 不跟随
    FOLLOW_GIMBAL,     // 跟随
    FOLLOWBACK_GIMBAL, // 反向跟随
    SLOW_GYRO,         // 慢陀螺
    FAST_GYRO,         // 快陀螺
    MOVE_BACK,         // 倒车
    CHASS_AUTO,        // 底盘自主
} motion_mode_e;

typedef enum
{
    AIMBOT_MODE = 0,    // 普通自瞄
    AIMBUFF_CONST_MODE, // 小能量机关
    AIMBUFF_VAR_MODE,   // 大能量机关
    ROTATING_OUTPOST,   // 旋转击打前哨战
    STATIC_OUTPOST,     // 静止击打前哨战
    OUTPOST_MODE_F,     // 第三种击打前哨战的模式
    DANGLING_MODE       // 吊射模式
} aimbot_mode_e;

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
} menu_e;

//typedef __packed struct
typedef struct
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
} mixed_msg_t;                           // 杂项数据包
//} __packed mixed_msg_t;

extern rt_int8_t FillMode_EN; // 补弹模式标志位
extern rt_int8_t motion_mode; // 底盘运动模式
extern rt_uint8_t GimbalSetPower;

// 进入/退出探头模式需要调用的函数
extern void Enter_Probe_Mode(int Enter);
// 进入一键回头模式时调用的函数
extern void Enter_MoveBack_Mode(void);
// 记录进入吊射模式时的底盘模式
extern void Dangling_RecNow_MotionMode(void);

extern void Refresh_RemoteSet_Gimbal(void);
extern void Chassis_RemoteCTR(RC_Ctrl_t *RC_Data_in);
extern void Computer_CTR(RC_Ctrl_t *RC_Data_in);
extern void ResetCmd_Write(int Reset_Flag);
extern void Write_Computer_Ctrl_Status(int Now_Computer_Ctrl);

/***
 * @brief    向底盘发送各类设定数据
 * @param
 * @retval   none
 * @author   dxy
 ***/
extern void chassis_data_send(void);

extern void ModCTR_Init(void); // 初始化运动控制用的平滑控制块

#endif
