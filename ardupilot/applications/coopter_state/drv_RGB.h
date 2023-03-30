#include <rtthread.h>

#define SLOW_TIMES    100
#define QUICK_TIMES   30
typedef enum 
{
	red_slow = 1,
	red_keep ,
	blue_quick,
	blue_slow,
	blue_keep,
	green_keep,
	green_quick
}RGB_types;

typedef struct 
{
	float red;
	float blue;
	float green;
}RGB_light;

extern void red_threetimes(void);
extern void red_blink_slowly(void);
extern void red_keepon(void);
extern void blue_quickly(void);
extern void blue_slowly(void);
extern void blue_keepon(void);
extern void green_keepon(void);
extern void green_quickly(void);
extern void MainRGB_init(void);
