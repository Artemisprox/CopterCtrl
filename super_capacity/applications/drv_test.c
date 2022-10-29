#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "drv_test.h"
#include "drv_dac.h"

#include "func_HW_Pin_Set.h"
#include "func_buzzer.h"

#include "func_adc.h"


#if (TEST_MEASURE)

static int fori,Enable;

// DAC 参数测定模式
#if (TEST_SELECT==0)
void Test_Measure_Init()
{

    fori = 0;//从0V开始测量
    Enable = 0;

    while (1)
    {
        set_buzzer(2800, 0.3);
        rt_thread_delay(90);
        set_buzzer(0, 0);
        if(Enable==1)
        {
            HWFUN_CHG_EN_PIN_EN;
            HWFUN_SPLY_EN_PIN_EN;
            HWFUN_LED0_1_ON;
        }
        else
        {
            HWFUN_CHG_EN_PIN_DIS;
            HWFUN_SPLY_EN_PIN_DIS;
            HWFUN_LED0_1_OFF;
        }
        User_dac_write(1, fori / 100.0f);
        User_dac_write(2, fori / 100.0f);

        rt_thread_delay(150); // 刚切换到新的设定值时不检测按键输入

        while (1)
        {// 等待按键
            if (rt_pin_read(HW_KEY_UP) == PIN_LOW || rt_pin_read(HW_KEY_LE) == PIN_LOW)
            {
                fori++;
                if(fori>200)
                {
                    fori = 200;
                }
                break;
            }
            else if (rt_pin_read(HW_KEY_DO) == PIN_LOW || rt_pin_read(HW_KEY_RI) == PIN_LOW)
            {
                fori--;
                if (fori < 0)
                {
                    fori = 0;
                }
                break;
            }
            if(rt_pin_read(HW_KEY_PR) == PIN_LOW)
            {
                Enable = !Enable;
                break;
            }
            rt_thread_delay(5);
            adc_DataProcess();
        }
    }
}
#endif



#endif
