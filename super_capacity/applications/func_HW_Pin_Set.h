#ifndef __HW_PIN_SET_H__
#define __HW_PIN_SET_H__

#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"

/* DAC输出通道设置 */
#define HW_DAC_CHG_SETI_CH (1)
#define HW_DAC_SPLY_SETI_CH (2)

/* ADC计算设置 */
//ADC_CH: ADC通道
//ADC_K: ADC采集到的电压(单位:V)*ADC_K = 该通道需要测量的量的值

//核心adc数据需要在func_adc中标定, 此处填写理论值即可
#define HW_ADC_CH_I_IN (13)             //模块输入电流检测通道
#define HW_ADC_NUM_I_IN (2)
#define _HW_ADC_K_I_IN (1/20.0F/0.0125F) //换算至单位：A

#define HW_ADC_CH_V_CAP (2) //电容电压检测通道
#define HW_ADC_NUM_V_CAP (3)
#define HW_ADC_K_V_CAP (105.1f/5.1f)    //换算超级电容组供电电压至单位：V

#define HW_ADC_CH_V_IN (12)             //模块供电电压检测通道
#define HW_ADC_NUM_V_IN (1)
#define HW_ADC_K_V_IN (11.0F)           //换算模块供电电压至单位：V

// 模块数据
#define HW_ADC_CH_I_CHG_IN (6)                      //超级电容充电模块-输入电流反馈通道
#define HW_ADC_NUM_I_CHG_IN (5)                     //超级电容充电模块-输入电流反馈通道
#define _HW_ADC_K_I_CHG_IN (1 / 0.05f * 11.0F)    //换算至单位：A

#define HW_ADC_CH_I_CHG_OUT (14)                    //超级电容充电模块-输出电流反馈通道
#define HW_ADC_NUM_I_CHG_OUT (4)                    //超级电容充电模块-输出电流反馈通道
#define _HW_ADC_K_I_CHG_OUT (1 / 0.03f / 11.f)      //换算至单位：A

#define HW_ADC_CH_I_SPLY_IN (15)                        //超级电容升压输出模块-输入电流反馈通道
#define HW_ADC_NUM_I_SPLY_IN (7)                        //超级电容升压输出模块-输入电流反馈通道
#define _HW_ADC_K_I_SPLY_IN (1 / 0.05f * 1.32f / 0.42f) //换算至单位：A

#define HW_ADC_CH_I_SPLY_OUT (9)                 //超级电容升压输出模块-输出电流反馈通道
#define HW_ADC_NUM_I_SPLY_OUT (6)                 //超级电容升压输出模块-输出电流反馈通道
#define _HW_ADC_K_I_SPLY_OUT (1 / 0.03f * 11.0f) //换算至单位：A

extern const float HW_ADC_K_I_IN;       //控制模块输入电流换算比例
extern const float HW_ADC_K_I_CHG_IN;   //充电模块输入电流换算比例
extern const float HW_ADC_K_I_CHG_OUT;  //充电模块输出电流换算比例
extern const float HW_ADC_K_I_SPLY_IN;  //供电模块输入电流换算比例
extern const float HW_ADC_K_I_SPLY_OUT; //供电模块输出电流换算比例

//输出GPIO设置
#define HW_CHG_EN_PIN GET_PIN(A, 7)
#define HW_SPLY_EN_PIN GET_PIN(B, 0)

#define HW_EXPANDIO_A15_PIN GET_PIN(A, 15)
#define HW_EXPANDIO_C10_PIN GET_PIN(C, 10)
#define HW_EXPANDIO_C11_PIN GET_PIN(C, 11)
#define HW_EXPANDIO_C12_PIN GET_PIN(C, 12)

#define HW_LED1_PIN GET_PIN(B, 9)
#define HW_LED2_PIN GET_PIN(B, 8)
#define HW_LED3_PIN GET_PIN(B, 7)

// 用于电量显示的LED
#define HW_LED0_1_PIN GET_PIN(B, 15)
#define HW_LED0_2_PIN GET_PIN(B, 14)
#define HW_LED0_3_PIN GET_PIN(B, 13)
#define HW_LED0_4_PIN GET_PIN(B, 12)

// 五轴按键旁LED
#define HW_LEDKEY_PIN GET_PIN(C, 0)

#define HW_LED_ON PIN_LOW
#define HW_LED_OFF PIN_HIGH

#define HW_OLED_SCL GET_PIN(B, 10)
#define HW_OLED_SDA GET_PIN(B, 11)

//输出GPIO快捷操作
#define HWFUN_CHG_EN_PIN_EN 	rt_pin_write(HW_CHG_EN_PIN, PIN_HIGH)
#define HWFUN_CHG_EN_PIN_DIS 	rt_pin_write(HW_CHG_EN_PIN, PIN_LOW)
#define HWFUN_SPLY_EN_PIN_DIS   _HWFUN_SPLY_EN_PIN_DIS()
#define HWFUN_SPLY_EN_PIN_EN    _HWFUN_SPLY_EN_PIN_EN()
#define __HWFUN_SPLY_EN_PIN_DIS rt_pin_write(HW_SPLY_EN_PIN, PIN_LOW)
#define __HWFUN_SPLY_EN_PIN_EN  rt_pin_write(HW_SPLY_EN_PIN, PIN_HIGH)
#define HWFUN_LED1_ON	 		rt_pin_write(HW_LED1_PIN, HW_LED_ON)
#define HWFUN_LED2_ON 			rt_pin_write(HW_LED2_PIN, HW_LED_ON)
#define HWFUN_LED3_ON 			rt_pin_write(HW_LED3_PIN, HW_LED_ON)
#define HWFUN_LED1_OFF 			rt_pin_write(HW_LED1_PIN, HW_LED_OFF)
#define HWFUN_LED2_OFF 			rt_pin_write(HW_LED2_PIN, HW_LED_OFF)
#define HWFUN_LED3_OFF 			rt_pin_write(HW_LED3_PIN, HW_LED_OFF)
#define HWFUN_LED0_1_OFF        rt_pin_write(HW_LED0_1_PIN, HW_LED_OFF)
#define HWFUN_LED0_2_OFF        rt_pin_write(HW_LED0_2_PIN, HW_LED_OFF)
#define HWFUN_LED0_3_OFF        rt_pin_write(HW_LED0_3_PIN, HW_LED_OFF)
#define HWFUN_LED0_4_OFF        rt_pin_write(HW_LED0_4_PIN, HW_LED_OFF)
#define HWFUN_LED0_1_ON         rt_pin_write(HW_LED0_1_PIN, HW_LED_ON)
#define HWFUN_LED0_2_ON         rt_pin_write(HW_LED0_2_PIN, HW_LED_ON)
#define HWFUN_LED0_3_ON         rt_pin_write(HW_LED0_3_PIN, HW_LED_ON)
#define HWFUN_LED0_4_ON         rt_pin_write(HW_LED0_4_PIN, HW_LED_ON)
#define HWFUN_LEDKEY_ON         rt_pin_write(HW_LEDKEY_PIN, HW_LED_ON)
#define HWFUN_LEDKEY_OFF        rt_pin_write(HW_LEDKEY_PIN, HW_LED_OFF)

//五轴按键GPIO设置
#define HW_KEY_LE GET_PIN(B, 4)
#define HW_KEY_RI GET_PIN(B, 6)
#define HW_KEY_DO GET_PIN(B, 5)
#define HW_KEY_UP GET_PIN(D, 2)
#define HW_KEY_PR GET_PIN(B, 3)

//拨码开关GPIO设置
#define HW_KEY1 GET_PIN(C, 7)
#define HW_KEY2 GET_PIN(C, 8)
#define HW_KEY3 GET_PIN(C, 9)
#define HW_KEY4 GET_PIN(A, 8)

extern void _HWFUN_SPLY_EN_PIN_DIS(void);
extern void _HWFUN_SPLY_EN_PIN_EN(void);

#endif
