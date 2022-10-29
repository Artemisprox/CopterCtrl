#ifndef __CUSTOMUI_H
#define __CUSTOMUI_H

//#include "struct_typedef.h"
#include "drv_CustomUI_basic.h"
#include "mod_RefSystem.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "stdbool.h"
#include "HRef_ID.h"

extern rt_uint8_t ui_reset;

extern uint8_t scope_draw(custom_ui_buff_t *ui, float pitch, float yaw, float roll,
                          int16_t auxaim_x, int16_t auxaim_y, uint32_t ammo_1, uint16_t ammo_2, uint8_t gun_mode, uint16_t gun_speed);
extern void ScopeInit(uint16_t offset_x, uint16_t offset_y, uint8_t have_ammo2_);
extern void referee_intercom_task(void* parameter);
extern rt_err_t UI_Init(void);
	
#endif
