#ifndef KEY_TEST_H
#define KEY_TEST_H

#include <rtthread.h>

/* 引脚编号，通过查看设备驱动文件drv_gpio.c确定 */
#define KEY1_PIN_NUM GET_PIN(B, 9)
#define KEY2_PIN_NUM GET_PIN(C, 10)
#define BEEP_PIN_NUM GET_PIN(C, 13)

#define PWM_DEV_NAME "pwm3" /* PWM设备名称 */

#define RGB_G 1 /* PWM通道 */
#define RGB_R 2 /* PWM通道 */
#define RGB_B 3 /* PWM通道 */

typedef enum
{
    BEEP_KEY,
    RGB_G_KEY,
    RGB_R_KEY,
    RGB_B_KEY
} Mode_e;

void KEY_Init(void);

#endif /* KEY_TEST_H */
