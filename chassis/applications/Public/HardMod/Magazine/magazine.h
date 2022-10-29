#ifndef __MAGAZINE_H__
#define __MAGAZINE_H__

#include "rtthread.h"
#include <rtdevice.h>

#define PWM_DEV_NAME        "pwm1"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL     2       /* PWM通道 */

#define PER_OPEN    5   /*开弹仓*/
#define PER_CLOSE   70  /*关弹仓*/



/**
 * @brief  打开弹仓
 * @param  None
 * @return None
 */
void Maga_Open(void);

/**
 * @brief  关闭弹仓
 * @param  None
 * @return None
 */
void Maga_Close(void);

#endif

