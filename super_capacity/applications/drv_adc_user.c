#include "drv_adc_user.h"
#include "drv_HW_Select.h"

static ADC_HandleTypeDef hadc1; // ADC外设
DMA_HandleTypeDef hdma_adc1;	// dma外设
static TIM_HandleTypeDef htim8; // 用于触发adc采样的tim

static rt_uint16_t adc_data_Buff[ADC_BUFF_LEN][8]; // 用于存放adc采样数据的数组

static rt_int16_t adc_First_Sample = 1;
static float adc_data_out[8]; // 用于进行滞后滤波计算的数组

static int adc_Valid = 0;//标记当前ADC是否已经有可靠数据

float Sply_Voltage = 3.3f;

char ADC_Fresh = 0; // 用于看门狗读取的标志位，每次ADC刷新数据时置1，看门狗读取时清零

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable or disable the remapping of ADC1_ETRGREG:
  * ADC1 External Event regular conversion is connected to TIM8 TRG0
  */
  __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

// HAL-ADC1传输完成中断
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	int fori, fori_ch;
	if (hadc == &hadc1)
	{
		//此时完成了几次采集，可以进行滤波计算
		for (fori = 0; fori < ADC_SAMPLE_TIMES; fori++)
		{ // 对采集到的数据进行低通滤波
			if (adc_First_Sample)
			{ // 首轮数据
				adc_First_Sample = 0;
				for (fori_ch = 0; fori_ch < 8; fori_ch++)
				{
					adc_data_out[fori_ch] = adc_data_Buff[fori][fori_ch];
				}
			}
			else
			{
				for (fori_ch = 0; fori_ch < 8; fori_ch++)
				{
					adc_data_out[fori_ch] = adc_data_Buff[fori][fori_ch] * ADC_FILTER_RATE + adc_data_out[fori_ch] * (1-ADC_FILTER_RATE);
				}
			}
		}
		Sply_Voltage = HW_CHIP_INTERNAL_VREF * 4095.0f / adc_data_out[7];
		if (adc_Valid<10000)
			adc_Valid++;// 为防止数据溢出，这里进行限幅

		ADC_Fresh = 1;
	}
}

// 读取对应编号的通道的adc转换结果
float adc_read(char Ch_Num)
{
	if(adc_Valid>0)
	{// 如果ADC已经正常处理过数据
		return adc_data_out[Ch_Num - 1] / adc_data_out[7] * HW_CHIP_INTERNAL_VREF;
	}
	return 0.0f;//如果没有处理过数据，则返回0
}

// 等待ADC初始化完成
static void adc_WaitForInit()
{
	while(adc_Valid<100)
	{
		rt_thread_delay(10);
	}
}

extern DMA_HandleTypeDef hdma_adc1;

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

	/* USER CODE END DMA1_Channel1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_adc1);
	/* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

	/* USER CODE END DMA1_Channel1_IRQn 1 */
}

// 初始化adc外设驱动
void adc_dev_init(void)
{
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM8_Init();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data_Buff, 8 * ADC_SAMPLE_TIMES); //启动DMA
	HAL_TIM_Base_Start_IT(&htim8);												//启动定时器，触发ADC-DMA采样
	adc_WaitForInit();
}
