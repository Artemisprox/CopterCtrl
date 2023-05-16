#ifndef _DRV_GPIO_CTRL_H_
#define _DRV_GPIO_CTRL_H_

#include "rtdef.h"
#include <board.h>

#define GREEN_LIGHT GET_PIN(B, 12)
#define RAD_LIGHT GET_PIN(B, 13)

#define PILOT_LAMP2 GET_PIN(B, 14)

typedef enum
{
    Open = 0,  // 打开气泵
    Close,     // 关闭气泵
} GPIO_Mode_e; // 吸盘状态枚举体

/**
 * @brief GPIO口模式设置
 * @param pin
 * @param mode
 */
void GPIO_State_Set(rt_base_t pin,
                    GPIO_Mode_e mode);

#endif //_DRV_GPIO_CTRL_H_
