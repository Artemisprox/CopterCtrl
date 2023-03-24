#ifndef CAN_TEST_H
#define CAN_TEST_H

#include <rtdevice.h>

#define CAN_TEST 1

#define THREAD_PRIO_TEST_CAN1RX 5
#define THREAD_PRIO_TEST_CAN2RX 5

rt_err_t TEST_CAN_init(void);

#endif /* CAN_TEST_H */
