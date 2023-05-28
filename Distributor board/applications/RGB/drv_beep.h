#include <rtdevice.h>

#define BEEP_PIN_NUM GET_PIN(B,4)

extern void BEEP_init(void);
extern void beep_set_high(void);
extern void beep_set_low(void);