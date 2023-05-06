#include "drv_HWTimer.h"
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim11;

// 获取当前计数值
int TIM11_GetCNT(void)
{
    return TIM11->CNT;
}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 9 - 1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim11);
	HAL_TIM_Base_Start(&htim11);

}
