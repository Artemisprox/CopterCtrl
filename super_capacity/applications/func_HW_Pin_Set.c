#include "func_HW_Pin_Set.h"

const float HW_ADC_K_I_IN = _HW_ADC_K_I_IN;
const float HW_ADC_K_I_CHG_IN = _HW_ADC_K_I_CHG_IN;
const float HW_ADC_K_I_CHG_OUT = _HW_ADC_K_I_CHG_OUT;
const float HW_ADC_K_I_SPLY_IN = _HW_ADC_K_I_SPLY_IN;
const float HW_ADC_K_I_SPLY_OUT = _HW_ADC_K_I_SPLY_OUT;

static char HW_PIN_SPLY_TOGGLE = 0;

void _HWFUN_SPLY_EN_PIN_EN()
{
    if (HW_PIN_SPLY_TOGGLE)
    {
        __HWFUN_SPLY_EN_PIN_DIS;
    }
    else
    {
        __HWFUN_SPLY_EN_PIN_EN;
    }
}
void _HWFUN_SPLY_EN_PIN_DIS()
{
    if (HW_PIN_SPLY_TOGGLE)
    {
        __HWFUN_SPLY_EN_PIN_EN;
    }
    else
    {
        __HWFUN_SPLY_EN_PIN_DIS;
    }
}

int HW_USE_PIN_INIT(void)
{
    //输入GPIO
        rt_pin_mode(HW_KEY_LE, PIN_MODE_INPUT);
        rt_pin_mode(HW_KEY_RI, PIN_MODE_INPUT);
        rt_pin_mode(HW_KEY_UP, PIN_MODE_INPUT);
        rt_pin_mode(HW_KEY_DO, PIN_MODE_INPUT);
        rt_pin_mode(HW_KEY_PR, PIN_MODE_INPUT);

        rt_pin_mode(HW_KEY1, PIN_MODE_INPUT);
        rt_pin_mode(HW_KEY2, PIN_MODE_INPUT);
        rt_pin_mode(HW_KEY3, PIN_MODE_INPUT);
        rt_pin_mode(HW_KEY4, PIN_MODE_INPUT);

        if (rt_pin_read(HW_KEY4)==PIN_LOW)
        {
            HW_PIN_SPLY_TOGGLE = 1;
        }

    //输出GPIO
        rt_pin_mode(HW_CHG_EN_PIN, PIN_MODE_OUTPUT); //充电控制接口需要推挽
        if (HW_PIN_SPLY_TOGGLE)
        {//根据拨码开关选择的硬件类型来执行对应的初始化
            rt_pin_mode(HW_SPLY_EN_PIN, PIN_MODE_OUTPUT_OD); //供电控制接口需要开漏
        }
        else
        {
            rt_pin_mode(HW_SPLY_EN_PIN, PIN_MODE_OUTPUT); //供电控制接口需要推挽
        }
        rt_pin_mode(HW_EXPANDIO_A15_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_EXPANDIO_C10_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_EXPANDIO_C11_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_EXPANDIO_C12_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_LED1_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_LED2_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_LED3_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_LED0_1_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_LED0_2_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_LED0_3_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_LED0_4_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(HW_LEDKEY_PIN, PIN_MODE_OUTPUT);
        //以上引脚初始化后默认低电平，下面对各引脚电平进行修改
        rt_pin_write(HW_LED1_PIN, HW_LED_OFF);
        rt_pin_write(HW_LED2_PIN, HW_LED_OFF);
        rt_pin_write(HW_LED3_PIN, HW_LED_OFF);
        rt_pin_write(HW_LED0_1_PIN, HW_LED_OFF);
        rt_pin_write(HW_LED0_2_PIN, HW_LED_OFF);
        rt_pin_write(HW_LED0_3_PIN, HW_LED_OFF);
        rt_pin_write(HW_LED0_4_PIN, HW_LED_OFF);
        rt_pin_write(HW_LEDKEY_PIN, HW_LED_OFF);
        HWFUN_CHG_EN_PIN_DIS;
        HWFUN_SPLY_EN_PIN_DIS;
		
	return RT_EOK;
}
INIT_DEVICE_EXPORT(HW_USE_PIN_INIT);
