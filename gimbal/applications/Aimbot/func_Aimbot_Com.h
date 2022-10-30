#ifndef __func_Aimbot_Com_H__
#define __func_Aimbot_Com_H__

#include <rtdef.h>

#define LED_CTRL_EN (0)

// 初始化视觉通信
extern int Visual_Com_Init(void);

// 外部调用，用于修改当前发给视觉的鼠标右键标志位 按下为1
extern void Aimbot_FreshMouseClick(char ClickData);

// CAN接收：标志位报文
extern rt_err_t VisualCom_Receive_Flag(rt_uint8_t rxmsg[]);
// CAN接收：云台设定值报文
extern rt_err_t VisualCom_Receive_Atti(rt_uint8_t rxmsg[]);

#endif
