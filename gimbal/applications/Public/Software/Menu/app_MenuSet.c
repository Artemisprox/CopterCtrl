#include "app_MenuSet.h"
#include "func_KeyCallback.h"

Key_Menu_Str_t MainMenu;              // 主菜单
Key_Menu_Str_t ChassisModeMenu;       // 底盘模式选择菜单
Key_Menu_Str_t AimModeMenu;           // 自瞄模式选择菜单
Key_Menu_Str_t MiscMenu;              // 杂项设置选择菜单
Key_Menu_Str_t RobotResetMenu;        // 机器人复位选择菜单
Key_Menu_Str_t SCAPCtrlMenu;          // 超级电容充电开关菜单
Key_Menu_Str_t GunSpeedMenu;          // 弹速设置选择菜单
Key_Menu_Str_t PerformanceMenu;       // 机器人性能菜单
Key_Menu_Str_t PerformanceChooseMenu; // 机器人性能选择菜单

/**
 * @brief：直接触发的按键回调函数初始化
 * @param [in]   无
 * @return：		无
 * @author：zzj
 */
static void Key_Callback_cfg(void)
{
    Key_SetCallBack(AIMBOT_KEY, &Aimbot_PressCallback, &Aimbot_LoosenCallback);
    Key_SetCallBack(PROBE_KEY, &Probe_PressCallback, &Probe_LoosenCallback);
    Key_SetCallBack(SCAP_RESERVE_KEY, &SCAP_Reserve_PressCallback, RT_NULL);
}

// 初始化需要使用的机器人按键控制菜单
rt_err_t Ctrl_Menu_Init(void)
{
    int EXIT_Key = FOREWORD_KEY | BACK_KEY | LEFT_KEY | RIGHT_KEY | KeyEVT_MouseZ_N | KeyEVT_MouseZ_P;

    Key_Callback_cfg(); // 初始化非二维按键

    KeyCtrl_MenuStr_Init(&MainMenu); // 初始化主菜单结构体
    MenuCmd_SetMainMenu(&MainMenu);  // 设置为初始菜单
    KeyCtrl_MenuStr_Init(&ChassisModeMenu);
    KeyCtrl_MenuStr_Init(&AimModeMenu);
    KeyCtrl_MenuStr_Init(&MiscMenu);
    KeyCtrl_MenuStr_Init(&PerformanceMenu);
    KeyCtrl_MenuStr_Init(&PerformanceChooseMenu);
    KeyCtrl_MenuStr_Init(&SCAPCtrlMenu);

    // 设置用于记录是否在主菜单的函数
    MenuSet_Add_EntryFun(&MainMenu, KeyEVT_ALL, MainMenu_REC_Entry_Fun);
    MenuSet_Add_QuitFun(&MainMenu, KeyEVT_ALL, MainMenu_REC_Quit_Fun);

    // 底盘模式设置菜单结构
    MenuSet_Add_SubMenu(&MainMenu, &ChassisModeMenu, CHASSISMODE_KEY_ENTRY);
    MenuSet_Add_EntryFun(&ChassisModeMenu, KeyEVT_ALL, MotionModeEntry_Callback);
    MenuSet_Add_QuitFun(&ChassisModeMenu, CHASSISMODE_KEY_MODE_NO_FOLLOW | CHASSISMODE_KEY_MODE_FOLLOW | CHASSISMODE_KEY_MODE_SMALL_GYRO | CHASSISMODE_KEY_MODE_FAST_GYRO | CHASSISMODE_KEY_MODE_MOVE_BACK, MotionModeSet_Callback);
    MenuSet_Add_QuitFun(&ChassisModeMenu, KeyEVT_ALL, MotionModeQuit_Callback);
    MenuSet_Add_SubMenu(&ChassisModeMenu, &MainMenu, CHASSISMODE_KEY_MODE_NO_FOLLOW | CHASSISMODE_KEY_MODE_FOLLOW | CHASSISMODE_KEY_MODE_SMALL_GYRO | CHASSISMODE_KEY_MODE_FAST_GYRO | CHASSISMODE_KEY_MODE_MOVE_BACK | EXIT_Key);

    // 滚轮相关菜单结构
    MenuSet_Add_SubMenu(&MainMenu, &MainMenu, KeyEVT_MouseZ_N | KeyEVT_MouseZ_P);
    MenuSet_Add_QuitFun(&MainMenu, KeyEVT_MouseZ_P, MouseZ_P_Callback);
    MenuSet_Add_QuitFun(&MainMenu, KeyEVT_MouseZ_N, MouseZ_N_Callback);

    // 自瞄模式菜单结构
    MenuSet_Add_SubMenu(&MainMenu, &AimModeMenu, AIMMODE_SET_ENTRY);
    MenuSet_Add_EntryFun(&AimModeMenu, KeyEVT_ALL, AimMode_Entry_Callback);
    MenuSet_Add_QuitFun(&AimModeMenu, AIMMODE_SET_AIMBOT |
#ifdef CORE_USING_HERO
                                          AIMMODE_SET_ROTATING_OUTPOST | AIMMODE_SET_STATIC_OUTPOST | AIMMODE_SET_OUTPOST_F | AIMMODE_SET_DANGLING_MODE,
#else
                                          AIMMODE_SET_AIMBUFF_CONST_SPEED | AIMMODE_SET_AIMBUFF_VARY_SPEED,
#endif
                        AimMode_Set_Callback);
    MenuSet_Add_QuitFun(&AimModeMenu, KeyEVT_ALL, AimMode_Quit_Callback);
    MenuSet_Add_SubMenu(&AimModeMenu, &MainMenu, AIMMODE_SET_AIMBOT | EXIT_Key |
#ifdef CORE_USING_HERO
                                                     AIMMODE_SET_ROTATING_OUTPOST | AIMMODE_SET_STATIC_OUTPOST | AIMMODE_SET_OUTPOST_F | AIMMODE_SET_DANGLING_MODE);
#else
                                                     AIMMODE_SET_AIMBUFF_CONST_SPEED | AIMMODE_SET_AIMBUFF_VARY_SPEED);
#endif

    // 弹速选择菜单结构
    MenuSet_Add_SubMenu(&MainMenu, &GunSpeedMenu, GUNSPEED_SET_ENTRY);
    MenuSet_Add_EntryFun(&GunSpeedMenu, KeyEVT_ALL, GunSpeedSet_Entry_Callback);
    MenuSet_Add_QuitFun(&GunSpeedMenu, KeyEVT_ALL, GunSpeedSet_Quit_Callback);
#ifdef CORE_USING_HERO
    MenuSet_Add_QuitFun(&GunSpeedMenu, GUNSPEED_SET_10 | GUNSPEED_SET_16, GunSpeedSet_Set_Callback);
    MenuSet_Add_SubMenu(&GunSpeedMenu, &MainMenu, GUNSPEED_SET_10 | GUNSPEED_SET_16 | EXIT_Key);
#else
    MenuSet_Add_QuitFun(&GunSpeedMenu, GUNSPEED_SET_15 | GUNSPEED_SET_18 | GUNSPEED_SET_30, GunSpeedSet_Set_Callback);
    MenuSet_Add_SubMenu(&GunSpeedMenu, &MainMenu, GUNSPEED_SET_15 | GUNSPEED_SET_18 | GUNSPEED_SET_30 | EXIT_Key);
#endif

    // 杂项设置菜单结构
    MenuSet_Add_SubMenu(&MainMenu, &MiscMenu, MISC_KEY_ENTRY);
    MenuSet_Add_EntryFun(&MiscMenu, KeyEVT_ALL, MiscEntry_Callback);
#if defined CORE_USING_INFANTRY
    MenuSet_Add_QuitFun(&MiscMenu, MISC_KEY_UI_RST | MISC_KEY_HATCH_OPEN | MISC_KEY_HATCH_CLSE, MiscSet_Callback);
#elif defined CORE_USING_HERO
    MenuSet_Add_QuitFun(&MiscMenu, MISC_KEY_UI_RST, MiscSet_Callback);
#endif
    MenuSet_Add_QuitFun(&MiscMenu, KeyEVT_ALL, MiscQuit_Callback);
#if defined CORE_USING_INFANTRY
    MenuSet_Add_SubMenu(&MiscMenu, &MainMenu, MISC_KEY_UI_RST | MISC_KEY_HATCH_OPEN | MISC_KEY_HATCH_CLSE | EXIT_Key);
#elif defined CORE_USING_HERO
    MenuSet_Add_SubMenu(&MiscMenu, &MainMenu, MISC_KEY_UI_RST | EXIT_Key);
#endif

    // 复位单片机的第三维按键
    MenuSet_Add_SubMenu(&MiscMenu, &RobotResetMenu, MISC_KEY_RESET);
    // 功能上复位菜单项只有在按下 CTRL 才会生效
    MenuSet_Add_EntryFun(&RobotResetMenu, KeyEVT_ALL, ResetEntry_Callback);
    MenuSet_Add_QuitFun(&RobotResetMenu, MISC_KEY_GIMBAL_RESET | MISC_KEY_CHASSIS_RESET | MISC_KEY_TOTAL_RESET, Reset_Callback);
    MenuSet_Add_QuitFun(&RobotResetMenu, KeyEVT_ALL, ResetExit_Callback); // 因为需要提前按下 Ctrl 按键所以按下 Ctrl 时不能自己退出
    MenuSet_Add_SubMenu(&RobotResetMenu, &MainMenu, MISC_KEY_GIMBAL_RESET | MISC_KEY_CHASSIS_RESET | MISC_KEY_TOTAL_RESET | EXIT_Key);
    // 开关超级电容的第三维按键
    MenuSet_Add_SubMenu(&MiscMenu, &SCAPCtrlMenu, MISC_KEY_CLOSE_CAP);
    MenuSet_Add_EntryFun(&SCAPCtrlMenu, KeyEVT_ALL, SCAP_Ctrl_Entry_Callback);
    MenuSet_Add_QuitFun(&SCAPCtrlMenu, MISC_KEY_CAP_OPEN | MISC_KEY_CAP_CLSE, SCAP_Ctrl_Callback);
    MenuSet_Add_QuitFun(&SCAPCtrlMenu, KeyEVT_ALL, SCAP_Ctrl_Exit_Callback);
    MenuSet_Add_SubMenu(&SCAPCtrlMenu, &MainMenu, MISC_KEY_CAP_OPEN | MISC_KEY_CAP_CLSE | EXIT_Key);

    // 性能选择菜单
    MenuSet_Add_SubMenu(&MainMenu, &PerformanceMenu, PERFORMANCE_KEY_ENTRY);
    MenuSet_Add_EntryFun(&PerformanceMenu, KeyEVT_ALL, PerformanceEnter_Callback);
    MenuSet_Add_QuitFun(&PerformanceMenu, PERFORMANCE_KEY_LEVEL1 | PERFORMANCE_KEY_LEVEL2 | PERFORMANCE_KEY_LEVEL3, Performance_Callback);
    MenuSet_Add_QuitFun(&PerformanceMenu, KeyEVT_ALL, PerformanceQuit_Callback);
    MenuSet_Add_SubMenu(&PerformanceMenu, &MainMenu, PERFORMANCE_KEY_LEVEL1 | PERFORMANCE_KEY_LEVEL2 | PERFORMANCE_KEY_LEVEL3 | EXIT_Key);

    MenuSet_Add_SubMenu(&PerformanceMenu, &PerformanceChooseMenu, PERFORMANCE_TYPE_KEY_ENTRY);
    MenuSet_Add_EntryFun(&PerformanceChooseMenu, KeyEVT_ALL, PerformanceChooseEnter_Callback);
#ifdef CORE_USING_INFANTRY
    MenuSet_Add_QuitFun(&PerformanceChooseMenu, PERFORMANCE_TYPE_KEY_OUTBREAK | PERFORMANCE_TYPE_KEY_COOLING | PERFORMANCE_TYPE_KEY_BULLETSPEED | PERFORMANCE_TYPE_KEY_POWER | PERFORMANCE_TYPE_KEY_BLOOD | PERFORMANCE_TYPE_KEY_FORCED_OFFLINE | PERFORMANCE_TYPE_KEY_FORCED_ONLINE, PerformanceChoose_Callback);
    MenuSet_Add_QuitFun(&PerformanceChooseMenu, KeyEVT_ALL, PerformanceChooseQuit_Callback);
    MenuSet_Add_SubMenu(&PerformanceChooseMenu, &MainMenu, PERFORMANCE_TYPE_KEY_OUTBREAK | PERFORMANCE_TYPE_KEY_COOLING | PERFORMANCE_TYPE_KEY_BULLETSPEED | PERFORMANCE_TYPE_KEY_POWER | PERFORMANCE_TYPE_KEY_BLOOD | PERFORMANCE_TYPE_KEY_FORCED_OFFLINE | PERFORMANCE_TYPE_KEY_FORCED_ONLINE | EXIT_Key);
#elif defined CORE_USING_HERO
    MenuSet_Add_QuitFun(&PerformanceChooseMenu, PERFORMANCE_TYPE_KEY_OUTBREAK | PERFORMANCE_TYPE_KEY_BULLETSPEED | PERFORMANCE_TYPE_KEY_POWER | PERFORMANCE_TYPE_KEY_BLOOD | PERFORMANCE_TYPE_KEY_FORCED_OFFLINE | PERFORMANCE_TYPE_KEY_FORCED_ONLINE, PerformanceChoose_Callback);
    MenuSet_Add_QuitFun(&PerformanceChooseMenu, KeyEVT_ALL, PerformanceChooseQuit_Callback);
    MenuSet_Add_SubMenu(&PerformanceChooseMenu, &MainMenu, PERFORMANCE_TYPE_KEY_OUTBREAK | PERFORMANCE_TYPE_KEY_BULLETSPEED | PERFORMANCE_TYPE_KEY_POWER | PERFORMANCE_TYPE_KEY_BLOOD | PERFORMANCE_TYPE_KEY_FORCED_OFFLINE | PERFORMANCE_TYPE_KEY_FORCED_ONLINE | EXIT_Key);
#endif

    Key_Menu_Start();
    return RT_EOK;
}
