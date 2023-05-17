#ifndef __CAN_RECEIVE_H__
#define __CAN_RECEIVE_H__

#include <rtthread.h>
#include <rtdevice.h>

extern void can_rec(struct rt_can_msg *msg);

#endif
