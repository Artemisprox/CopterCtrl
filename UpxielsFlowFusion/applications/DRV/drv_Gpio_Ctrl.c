/**
 * @file drv_Gpio_Ctrl.c
 * @brief 自定义控制器指示灯的控制
 * @author mylj
 * @version 1.0
 * @date 2023-05-06
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_Gpio_Ctrl.h"

/**
 * @brief GPIO口模式设置
 * @param pin
 * @param mode
 */
void GPIO_State_Set(rt_base_t pin,
                    GPIO_Mode_e mode)
{
    switch (mode)
    {
    case Open:
        rt_pin_write(pin, PIN_LOW);
        break;
    case Close:
        rt_pin_write(pin, PIN_HIGH);
        break;

    default:
        break;
    }
}

/**
 * @brief GPIO初始化
 */
static int GPIO_Init(void)
{
    rt_pin_mode(GREEN_LIGHT, PIN_MODE_OUTPUT);
    rt_pin_mode(RAD_LIGHT, PIN_MODE_OUTPUT);
    
    rt_pin_write(GREEN_LIGHT, PIN_LOW);
    rt_pin_write(RAD_LIGHT, PIN_LOW);
    return 0;
}
INIT_BOARD_EXPORT(GPIO_Init);
