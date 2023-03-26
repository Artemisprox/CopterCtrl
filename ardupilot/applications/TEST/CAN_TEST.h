#ifndef CAN_TEST_H
#define CAN_TEST_H

#include <rtdevice.h>

#define THREAD_PRIO_TEST_CAN1RX 5
#define THREAD_PRIO_TEST_CAN2RX 5

rt_err_t CAN_Init(void);

#endif /* CAN_TEST_H */
