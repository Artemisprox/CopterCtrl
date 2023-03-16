#include <rtthread.h>
#include "drv_Queue.h"

#define LIST_LEN 10

/*报错列表*/
#define ATTI_LOST      0x01
#define RC_LOST        0x02
#define HEIGHT_LOST    0x03
#define POS_LOST       0x04
#define BATTERY_LOW    0x05

#define CHECK_BATTERY    0x11
#define GIMBAL_LOST      0x12
#define YAW_MOTOR_LOST   0x13
#define PITCH_MOTOR_LOST 0X14

#define THROTTLE_HIGH    0x21


extern void error_write(int error_ID);
extern void error_read(void);
extern void error_handle_init(void);
