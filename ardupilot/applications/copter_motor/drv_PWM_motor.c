#include "drv_PWM_motor.h"
#include "stm32f4xx_hal.h"
#include "main.h"

TIM_HandleTypeDef htim1;

//channel ʹ��HAL���Դ��ĺ궨�� �� duty ����ռ�ձȲ�������Χ0-1
void MX_TIM_DUTY(TIM_TypeDef *TIMx , uint32_t channel , float duty)
{
	if(duty > 1 || duty <0)
		return ;
	
	//���ݳ�ʼ����Ϣ��ռ�ձȽ��л���
	uint32_t ARR;
	ARR = TIMx->ARR+1;
	uint32_t CCR;
	CCR = duty*ARR;
	
	switch(channel)
	{
		case 0x00000000U : TIMx->CCR1 = CCR ; break;
		case 0x00000004U : TIMx->CCR2 = CCR ; break;
		case 0x00000008U : TIMx->CCR3 = CCR ; break;
		case 0x0000000CU : TIMx->CCR4 = CCR ; break;
		case 0x0000003CU : TIMx->CCR1 =CCR; TIMx->CCR2 =CCR; TIMx->CCR3=CCR; TIMx->CCR4 = CCR;break;
		default : return;
	}
}

int MX_TIM1_PWM_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 18 - 1 ;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 25000 - 1 ;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    return 1;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    return 1;
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    return 1;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    return 1;
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    return 1;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    return 1;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    return 1;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    return 1;
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    return 1;
  }

  HAL_TIM_MspPostInit(&htim1);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

	//HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_ALL);
	return 0;
}


//void MX_TIM8_PWM_Init(void)
//{

//  /* USER CODE BEGIN TIM8_Init 0 */
//	TIM_HandleTypeDef htim8 = {0};
//  /* USER CODE END TIM8_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//	
//	//__HAL_RCC_TIM1_CLK_ENABLE();
//  /* USER CODE BEGIN TIM8_Init 1 */

//  /* USER CODE END TIM8_Init 1 */
//  htim8.Instance = TIM8;
//  htim8.Init.Prescaler = 0;
//  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim8.Init.Period = 65535;
//  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim8.Init.RepetitionCounter = 0;
//  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
//  {
//    while(1);
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
//  {
//    while(1);
//  }
//  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
//  {
//    while(1);
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
//  {
//    while(1);
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0x8000;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    while(1);
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    while(1);
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//  {
//    while(1);
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    while(1);
//  }
//  /* USER CODE BEGIN TIM8_Init 2 */

//  /* USER CODE END TIM8_Init 2 */
//  HAL_TIM_MspPostInit(&htim8);

//	
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
//	//HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
//}
