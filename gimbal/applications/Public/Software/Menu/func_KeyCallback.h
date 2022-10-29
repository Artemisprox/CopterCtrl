#ifndef __FUNC_KEYCALLBACK_H_
#define __FUNC_KEYCALLBACK_H_

#include <rtthread.h>
#include "drv_Key_Set.h"

extern int current_menu; // 当前所处的菜单

/**
 * @brief：自瞄模式-按下回调函数
 * @param [in]   无
 * @return：		无
 * @author：zzj
 */
extern void Aimbot_PressCallback(void);
/**
 * @brief：自瞄模式-弹起回调函数
 * @param [in]   无
 * @return：		无
 * @author：zzj
 */
extern void Aimbot_LoosenCallback(void);

/**
 * @brief 探头模式-按下回调函数
 * @author fwlh
 */
extern void Probe_PressCallback(void);
/**
 * @brief 探头模式-弹起回调函数
 * @author fwlh
 */
extern void Probe_LoosenCallback(void);

/**
 * @brief 放开超级电容电量限制-按下回调函数
 * @author fwlh
 */
extern void SCAP_Reserve_PressCallback(void);

/**
 * @brief：一维按键回调：进入探头模式
 * @param [in]   无
 * @return：		无
 * @author：ych-zzj
 */
void VIEW_ENTRY_Callback(void);
/**
 * @brief：一维按键回调：退出探头模式
 * @param [in]   无
 * @return：		无
 * @author：ych-zzj
 */
void VIEW_EXIT_Callback(void);

/**
 * @brief：主菜单回调：按住CTRL则修改枪口热量限制为无限，不按CTRL则修改发弹模式为三连发
 * @param [in]   无
 * @return：		无
 * @author：ych-zzj
 */
void MouseZ_P_Callback(int TrigSource);
/**
 * @brief：主菜单回调：按住CTRL则修改枪口热量限制为限制，不按CTRL则修改发弹模式为自动模式
 * @param [in]   无
 * @return：		无
 * @author：ych-zzj
 */
void MouseZ_N_Callback(int TrigSource);

// 用于记录是否在主菜单的标志位修改函数，在进入和退出主菜单时调用即可
extern void MainMenu_REC_Entry_Fun(int TrigSource);
extern void MainMenu_REC_Quit_Fun(int TrigSource);

/**
 * @brief：底盘运动模式-二维按键回调函数
 * @author：ych
 */
extern void MotionModeEntry_Callback(int TrigSource);
/**
 * @brief：底盘运动模式-二维按键回调函数
 * @author：ych
 */
extern void MotionModeSet_Callback(int TrigSource);
/**
 * @brief：底盘运动模式-二维按键回调函数
 * @author：ych
 */
extern void MotionModeQuit_Callback(int TrigSource);

/**
 * @brief 其他设置-二维按键回调函数
 * @author：ych
 */
extern void MiscEntry_Callback(int TrigSource);
/**
 * @brief 其他设置-二维按键回调函数
 * @author：ych
 */
extern void MiscSet_Callback(int TrigSource);
/**
 * @brief 其他设置-二维按键回调函数
 * @author：ych
 */
extern void MiscQuit_Callback(int TrigSource);

/**
 * @brief 单片机复位选择菜单的进入函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
extern void ResetEntry_Callback(int TrigSource);

/**
 * @brief 单片机复位选择菜单的设置函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
extern void Reset_Callback(int TrigSource);

/**
 * @brief 单片机复位选择菜单的退出函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
extern void ResetExit_Callback(int TrigSource);

/**
 * @brief 超级电容充电开关选择菜单的进入函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
extern void SCAP_Ctrl_Entry_Callback(int TrigSource);

/**
 * @brief 超级电容充电开关选择菜单的设置函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
extern void SCAP_Ctrl_Callback(int TrigSource);

/**
 * @brief 超级电容充电开关选择菜单的退出函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
extern void SCAP_Ctrl_Exit_Callback(int TrigSource);

/**
 * @brief 进入自瞄模式设置-二维按键回调函数
 * @author：ych
 */
extern void AimMode_Entry_Callback(int TrigSource);
/**
 * @brief 选择自瞄模式-二维按键回调函数
 * @author：ych
 */
extern void AimMode_Set_Callback(int TrigSource);
/**
 * @brief 退出自瞄模式设置-二维按键回调函数
 * @author：ych
 */
extern void AimMode_Quit_Callback(int TrigSource);

/**
 * @brief：弹速设置-二维按键回调函数
 * @author：zzj
 */
void GunSpeedSet_Entry_Callback(int TrigSource);
/**
 * @brief：弹速设置-二维按键回调函数
 * @author：zzj
 */
void GunSpeedSet_Set_Callback(int TrigSource);
/**
 * @brief：弹速设置-二维按键回调函数
 * @author：zzj
 */
void GunSpeedSet_Quit_Callback(int TrigSource);

extern void PerformanceEnter_Callback(int Trig);
extern void Performance_Callback(int Trig);
extern void PerformanceQuit_Callback(int Trig);
extern void PerformanceChooseEnter_Callback(int Trig_Key);
extern void PerformanceChoose_Callback(int Trig_Key);
extern void PerformanceChooseQuit_Callback(int Trig_Key);

#endif
