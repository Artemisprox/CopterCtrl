#ifndef __DRV_AIMBOT_PUBULIC_H__
#define __DRV_AIMBOT_PUBULIC_H__

#include <rtdef.h>

// 一组姿态信息
typedef struct
{
    float Roll;
    float Pitch;
    float Yaw;
} AttitudeData_Type;

typedef enum
{
    My_Color_Red = 0,
    My_Color_Blue
} My_Color_Enum;

// 定义电控向视觉发送的标志位字节的内容
typedef enum
{
    AimFlag_MouseRightData = 0, // bit0：鼠标右键数据/遥控器是否开启自瞄
    AimFlag_MyColor = 1,        // bit1: 当前己方颜色
    AimFlag_Shoot = 6,          // bit6: 给视觉发出的发弹标志位, 该标志位跳变沿有效
    AimFlag_ShootRequire = 7,   // bit7: 给视觉发出的发弹请求标志位
} Aimbot_SendFlags_e;

typedef enum
{                                                // CAN发送报文ID
    VISUAL_MODE_ERR = 0x00,                      // 默认数值，表示当前没有收到过视觉的对时申请，无法获得当前视觉的工作模式
    VISUAL_MODE_AIMBOT_V2 = 0x01,                // 二代自瞄识别装甲板
    VISUAL_MODE_AIMBUFF_CONST_SPEED = 0x02,      // 击打匀速大风车
    VISUAL_MODE_AIMBUFF_VARY_SPEED = 0x03,       // 击打变速大风车
    VISUAL_MODE_AIMBUFF_ROTATING_OUTPOST = 0x04, // 动态击打前哨站
    VISUAL_MODE_AIMBUFF_STATIC_OUTPOST = 0x05,   // 静态击打前哨站
    VISUAL_MODE_AIMBUFF_OUTPOST_F = 0x06,        // 第三种击打前哨战的模式
} drv_VisualMode_e;

typedef struct
{
    rt_tick_t PredictedTime;
    AttitudeData_Type GimbalSet_Atti;
    AttitudeData_Type GimbalSet_Speed;
    rt_err_t State;
} Gimbal_SetCal_Type;

typedef struct
{
    rt_tick_t PredictedTime;
    AttitudeData_Type GimbalSet_Angle;
    AttitudeData_Type GimbalSet_Speed;
    rt_err_t State;
} Gimbal_SetReceive_Type;

extern drv_VisualMode_e Visual_Mode_Set; // 用于记录当前程序设定的视觉工作模式，如果出现模式不符，程序会自动通过通信调整视觉工作模式

extern char GunSet_AimbotShootFlag; // 由自瞄文件刷新的自瞄发射限制数据，可在自瞄时控制是否发弹，在FuncGun中使用

extern float Muzzle_V_REM; // 记录当前弹速
extern char Color_Myself;  // 己方颜色 0红，1蓝

extern char VisualSend_Flags; // 8个向视觉发送的标志位，与对时一起发送，10Hz

extern char VisualFlag_TargetFound;    // 视觉锁定目标标志
extern char VisualFlag_Fire;           // 运动预测准确标志
extern char VisualFlag_BurstShoot;     // 爆发攻击标志
extern char VisualFlag_RuneFire;       // 视觉的精选发弹控制, 仅在能量机关下使用
extern char VisualFlag_RuneBurstShoot; // 能量机关下进入 5 连发模式
extern char VisualFlag_ExitRune;       // 能量机关退出的标志位
extern char VisualFlag_WorkingCorrect; // 当前视觉程序正在正常运行

extern rt_tick_t Visual_LastFresh_Tick; // 上一次收到视觉数据的时间

extern char VisualMode_FB;                               // 视觉反馈的当前自瞄模式
extern float GimbalTolerance_Pitch, GimbalTolerance_Yaw; // 视觉反馈的自瞄精度要求

extern Gimbal_SetReceive_Type GimbalSet_Receive[2]; // 使用双缓冲
extern char Gimbal_Set_Cal_READ_Valid;              // 双缓冲队列可读队列号

#endif
