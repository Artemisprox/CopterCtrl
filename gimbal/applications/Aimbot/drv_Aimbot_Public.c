#include "drv_Aimbot_Public.h"

#include "robodata.h"

drv_VisualMode_e Visual_Mode_Set; // 用于记录当前程序设定的视觉工作模式，如果出现模式不符，程序会自动通过通信调整视觉工作模式

char GunSet_AimbotShootFlag = 0; // 由自瞄文件刷新的自瞄发射限制数据，可在自瞄时控制是否发弹，在FuncGun中使用

char VisualFlag_TargetFound = 0;    // 视觉锁定目标标志
char VisualFlag_Fire = 0;           // 运动预测准确标志
char VisualFlag_BurstShoot = 0;     // 爆发攻击标志
char VisualFlag_RuneFire = 0;       // 视觉的精选发弹控制, 仅在能量机关下使用
char VisualFlag_RuneBurstShoot = 0; // 能量机关下进入 5 连发模式
char VisualFlag_ExitRune = 0;       // 能量机关退出的标志位
char VisualFlag_WorkingCorrect = 0; // 当前视觉程序正在正常运行

char VisualMode_FB = 0; // 视觉反馈的当前自瞄模式

rt_tick_t Visual_LastFresh_Tick = 0; // 上一次收到视觉数据的时间

float Muzzle_V_REM; // 记录当前弹速

char VisualSend_Flags; // 8个向视觉发送的标志位，与对时一起发送，10Hz

char Color_Myself = (int)My_Color_Red; // 己方颜色 0红，1蓝

float GimbalTolerance_Pitch = 0, GimbalTolerance_Yaw = 0; // 当前视觉给出的允许控制精度

Gimbal_SetReceive_Type GimbalSet_Receive[2]; // 使用双缓冲
char Gimbal_Set_Cal_READ_Valid;              // 双缓冲队列可读队列号
