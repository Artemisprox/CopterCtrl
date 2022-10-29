#ifndef __FUNC_DAC_H__
#define __FUNC_DAC_H__

#include "drv_dac.h"
#include "func_HW_Pin_Set.h"

#include "drv_HW_Select.h"


//使用dac设定充电模块输出电流值--输出经过标定校正
extern float CHG_Set_Curr(float I_Set);

//使用dac设定供电升降压模块输出电流限流值
extern void SPLY_Set_Curr(float I_Set);

#endif
