#include <rtthread.h>
#define Mavlink_rec_com "uart2"
#define Mavlink_send_com "uart3"

extern void rtt_mavlink_write(const char *buf, uint16_t len);
