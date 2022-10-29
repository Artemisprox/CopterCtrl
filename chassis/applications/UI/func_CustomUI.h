#ifndef __DRV_CUSTOMUI_FUNC_H
#define __DRV_CUSTOMUI_FUNC_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_CustomUI_basic.h"

// /***
// * @param progress_total   绘制总进度
// * @param progress_now   当前所处进度
// ***/
// #define UI_START                         \
//     static rt_int8_t progress_total = 0; \
//     rt_int8_t progress_now = 0;          \


// #define UI_END      \
//     progress_total = 0;\

/***
* @name
* @brief    //每个UI绘制函数前都要调用此宏函数,用于现场保护
* @param func   执行函数
* @retval   UI_OK       操作成功
* @retval   UI_FULL     发送队列已满
* @retval   UI_CHAR     发送字节成功
* @author dxy
***/
#define UI_ASSERT(func)                                          \
    do                                                           \
    {                                                            \
        switch (func)                                            \
        {                                                        \
        case UI_OK:                                              \
            break;                                               \
        case UI_FULL:                                            \
            rt_mutex_release(&custom_ui_mutex);                  \
            rt_sem_take(&custom_ui_sem, RT_WAITING_FOREVER);     \
            rt_mutex_take(&custom_ui_mutex, RT_WAITING_FOREVER); \
            switch (func)                                            \
            {                                                        \
                case UI_OK:                                              \
                    break;                                               \
                case UI_CHAR:                                            \
                    rt_mutex_release(&custom_ui_mutex);                  \
                    rt_sem_take(&custom_ui_sem, RT_WAITING_FOREVER);     \
                    rt_mutex_take(&custom_ui_mutex, RT_WAITING_FOREVER); \
                    break;                                               \
            }\
            break;                                               \
        case UI_CHAR:                                            \
            rt_mutex_release(&custom_ui_mutex);                  \
            rt_sem_take(&custom_ui_sem, RT_WAITING_FOREVER);     \
            rt_mutex_take(&custom_ui_mutex, RT_WAITING_FOREVER); \
            break;                                               \
        }                                                         \
    } while (0)\

extern void UI_Clear_Init_Flag(void);
extern void UI_Set_Init_Flag(void);

extern ui_basic_e Draw_test(void *param);
extern ui_basic_e Fill_Full_Screen(void *param);

extern ui_basic_e Clear_Screen(void *param);

extern ui_basic_e Draw_SC_Bar_dyn(void *param);
extern ui_basic_e Draw_SC_Bar_bkgd(void *param);

extern ui_basic_e Draw_HERO_Cream_Help_bkgd(void *param);

extern ui_basic_e Draw_Strike_Mode(void *param);

extern ui_basic_e Draw_AuxAim_Crosshair(void *param);
extern ui_basic_e Draw_AuxAim_Ballistic_Line(void *param);

extern ui_basic_e Draw_Attitude_Angle_dyn(void *param);
extern ui_basic_e Draw_Attitude_Angle_bkgd(void *param);

extern ui_basic_e Draw_Bullet_Speed_dyn(void *param);
extern ui_basic_e Draw_Bullet_Speed_bkgd(void *param);

extern ui_basic_e Draw_Remain_Ammo_dyn(void *param);
extern ui_basic_e Draw_Remain_Ammo_bkgd(void *param);

extern ui_basic_e Draw_Motion_Mode_dyn(void *param);
extern ui_basic_e Draw_Motion_Mode_bkgd(void *param);

extern ui_basic_e Draw_Magazine_dyn(void *param);
extern ui_basic_e Draw_Magazine_bkgd(void *param);

extern ui_basic_e Draw_Monitor_dyn(void *param);
extern ui_basic_e Draw_Monitor_bkgd(void *param);

extern ui_basic_e Draw_Enemy_Orientation_dyn(void *param);

extern ui_basic_e Draw_Building_Attacked_dyn(void *param);

extern ui_basic_e Draw_LaneLine_bkgd(void *param);

extern ui_basic_e Draw_HeatLimit_dyn(void *param);
extern ui_basic_e Draw_HeatLimit_bkgd(void *param);

extern ui_basic_e Draw_Aimbot_Mode_dyn(void *param);
extern ui_basic_e Draw_Aimbot_Mode_bkgd(void *param);

extern ui_basic_e Draw_Aimbot_Color_dyn(void *param);
extern ui_basic_e Draw_Aimbot_Color_bkgd(void *param);

extern ui_basic_e Draw_Velocity_dyn(void *param);
extern ui_basic_e Draw_Velocity_bkgd(void *param);

//extern ui_basic_e Draw_Height_dyn(void *param);
//extern ui_basic_e Draw_Height_bkgd(void *param);

extern ui_basic_e Draw_PITCH_dyn(void *param);
extern ui_basic_e Draw_PITCH_bkgd(void *param);
extern ui_basic_e Draw_option(void *param);

extern ui_basic_e UI_Draw_END(void *param);

#endif
