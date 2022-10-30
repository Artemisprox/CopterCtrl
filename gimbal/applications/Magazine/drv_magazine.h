#ifndef __DRV_MAGAZINE_H__
#define __DRV_MAGAZINE_H__

#define SERVO_OPEN 1
#define SERVO_CLOSE 0

/**
 * @brief  舵机（弹仓门开关）初始化 初始化后会按照.c中设定的默认状态启动舵机控制
 * @param  duty 占空比（0-1000对于0%-%100）
 */
void Magazine_servo_init(void);
/**
 * @brief  弹仓舵机角度设定
 * @param  Set_Percent 舵机信号比例，0-1对应舵机整个转动范围
 */
void Magazine_servo_set(int OpenFlag);

#endif
