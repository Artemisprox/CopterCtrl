#include "func_RGBctrl.h"
#include "drv_RGB.h"

void RGB_init_set(void)
{
    RGB_Colour_Set(RGB_A,0,0,0);
    RGB_Colour_Set(RGB_B,0,0,0);
    RGB_Colour_Set(RGB_C,0,0,1);
    RGB_Colour_Set(RGB_D,1,1,1);
}
