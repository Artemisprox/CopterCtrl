#include "func_RGBctrl.h"
#include "drv_RGB.h"

void RGB_init_set(void)
{
    RGB_Colour_Set(RGB_A,500,500,500);
    RGB_Colour_Set(RGB_B,500,500,500);
    RGB_Colour_Set(RGB_C,500,500,500);
    RGB_Colour_Set(RGB_D,500,500,500);
}
