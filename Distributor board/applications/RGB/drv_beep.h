#include <rtdevice.h>

#define BEEP_PIN_NUM GET_PIN(A,7)

extern void BEEP_init(void);
extern void beep_set_high(void);
extern void beep_set_low(void);