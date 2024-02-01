/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint32_t ch1_rising = 0, ch2_rising = 0, ch3_rising = 0, ch4_rising = 0, ch5_rising = 0, ch6_rising = 0;
volatile uint32_t ch1_falling = 0, ch2_falling = 0, ch3_falling = 0, ch4_falling = 0, ch5_falling = 0, ch6_falling = 0;
volatile uint32_t pre_ch1 = 0, ch1 = 0, pre_ch2 = 0, ch2 = 0, pre_ch3 = 0, ch3 = 0, pre_ch4 = 0, ch4 = 0, pre_ch5 = 0, ch5 = 0, pre_ch6 = 0, ch6 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)  // interrupt TIM1 biriminden geliyorsa gir
	{
		switch(htim->Channel) // aktif kanal hamgisiyse, o kanalın case'ine git
		{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			if((TIM1->CCER & TIM_CCER_CC1P)==0) // kanalin aktif olmasi kesmenin oradan gelecegi anlamina gelmez/gpio pinini kontrol et
			{
				ch1_rising = TIM1->CCR1; // yukselen kenar degerini kaydet
				TIM1->CCER |= TIM_CCER_CC1P; // polariteyi düsen kenar olarak degistir
			}

			else
			{
				ch1_falling = TIM1->CCR1;
				pre_ch1 = ch1_falling - ch1_rising; // dusen kenar degerini kaydet ve yukselen kenar degerinden cikar
				if(pre_ch1 < 0)pre_ch1 += 0xFFFF;// eger sonuc negatifse taban tumleme yap
				if(pre_ch1 < 2010 && pre_ch1 > 990)ch1=pre_ch1;
				TIM1->CCER &= ~TIM_CCER_CC1P; // polariteyi yukselen kenar olarak ayarla

				/*
				 * ch1_rising 65000
				 * ch1 falling 570
				 * pre_ch1 = pre_ch1 falling - pre_ch1_rising = 570 - 65000 = -64430
				 * pre_ch1 +=0xFFFF(65536) --> 1106
				 */
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			if((TIM1->CCER & TIM_CCER_CC2P)==0)
			{
				ch2_rising = TIM1->CCR2;
				TIM1->CCER |= TIM_CCER_CC2P;
			}
			else
			{
				ch2_falling = TIM1->CCR2;
				pre_ch2 = ch2_falling - ch2_rising;
				if(pre_ch2 < 0)pre_ch2 += 0xFFFF;
				if(pre_ch2 < 2010 && pre_ch2 > 990)ch2=pre_ch2;
				TIM1->CCER &= ~TIM_CCER_CC2P;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			if((TIM1->CCER & TIM_CCER_CC3P)==0)
			{
				ch3_rising = TIM1->CCR3;
				TIM1->CCER |= TIM_CCER_CC3P;
			}
			else
			{
				ch3_falling = TIM1->CCR3;
				pre_ch3 = ch3_falling - ch3_rising;
				if(pre_ch3 < 0)pre_ch3 += 0xFFFF;
				if(pre_ch3 < 2010 && pre_ch3 > 990)ch3=pre_ch3;
				TIM1->CCER &= ~TIM_CCER_CC3P;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			if((TIM1->CCER & TIM_CCER_CC4P)==0)
			{
				ch4_rising = TIM1->CCR4;
				TIM1->CCER |= TIM_CCER_CC4P;
			}
			else
			{
				ch4_falling = TIM1->CCR4;
				pre_ch4 = ch4_falling - ch4_rising;
				if(pre_ch4 < 0)pre_ch4 += 0xFFFF;
				if(pre_ch4 < 2010 && pre_ch4 > 990)ch4=pre_ch4;
				TIM1->CCER &= ~TIM_CCER_CC4P;
			}
			break;
		default:
			break;
		}
	}
	else if(htim->Instance == TIM3) {
		switch(htim->Channel) // aktif kanal hamgisiyse, o kanalın case'ine git
		{
			case HAL_TIM_ACTIVE_CHANNEL_1:
				if((TIM3->CCER & TIM_CCER_CC1P)==0) // kanalin aktif olmasi kesmenin oradan gelecegi anlamina gelmez/gpio pinini kontrol et
				{
					ch5_rising = TIM3->CCR1; // yukselen kenar degerini kaydet
					TIM3->CCER |= TIM_CCER_CC1P; // polariteyi düsen kenar olarak degistir
				}

				else
				{
					ch5_falling = TIM3->CCR1;
					pre_ch5 = ch5_falling - ch5_rising; // dusen kenar degerini kaydet ve yukselen kenar degerinden cikar
					if(pre_ch5 < 0)pre_ch5 += 0xFFFF;// eger sonuc negatifse taban tumleme yap
					if(pre_ch5 < 2010 && pre_ch5 > 990)ch5=pre_ch5;
					TIM3->CCER &= ~TIM_CCER_CC1P; // polariteyi yukselen kenar olarak ayarla

					/*
					 * ch1_rising 65000
					 * ch1 falling 570
					 * pre_ch1 = pre_ch1 falling - pre_ch1_rising = 570 - 65000 = -64430
					 * pre_ch1 +=0xFFFF(65536) --> 1106
					 */
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				if((TIM3->CCER & TIM_CCER_CC2P)==0)
				{
					ch6_rising = TIM3->CCR2;
					TIM3->CCER |= TIM_CCER_CC2P;
				}
				else
				{
					ch6_falling = TIM3->CCR2;
					pre_ch6 = ch6_falling - ch6_rising;
					if(pre_ch6 < 0)pre_ch6 += 0xFFFF;
					if(pre_ch6 < 2010 && pre_ch6 > 990)ch6=pre_ch6;
					TIM3->CCER &= ~TIM_CCER_CC2P;
				}
				break;
			default:
				break;
		}
	}
}
/*
#define TIMCLOCK   180000000
#define PRESCALAR  180
char channel_array[4]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};

uint32_t IC_Val1[4]={0,0,0,0};
uint32_t IC_Val2[4]={0,0,0,0};
uint32_t Difference[4];
int Is_Captured[4]={0,0,0,0};


float frequency[4];
uint32_t usWidth[4];
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (Is_Captured[0]==0) // if the first rising edge is not captured
		{
			IC_Val1[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value

			Is_Captured[0] = 1;  // set the first captured as true
		}

		else   // If the first rising edge is captured, now we will capture the second edge
		{
			IC_Val2[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

		    if (IC_Val2[0] > IC_Val1[0])
			{
		    	Difference[0] = IC_Val2[0]-IC_Val1[0];
			}

			else if (IC_Val1[0] > IC_Val2[0])
			{
				Difference[0] =IC_Val1[0]*(-1)-IC_Val2[0];
			}
			else{
				Difference[0] =0;
			}

			float refClock = TIMCLOCK/(PRESCALAR);
			float mFactor = 1000000/refClock;
			frequency[0] =(refClock/Difference[0]);
			usWidth[0] = Difference[0]*mFactor;
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_Captured[0] = 0; // set it back to false
		}
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (Is_Captured[1]==0) // if the first rising edge is not captured
				{
					IC_Val1[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
					Is_Captured[1] = 1;  // set the first captured as true
				}

		else   // If the first rising edge is captured, now we will capture the second edge
		{
			IC_Val2[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value

			if (IC_Val2[1] > IC_Val1[1])
			{
			Difference[1] = IC_Val2[1]-IC_Val1[1];
			}

			else if (IC_Val1[1] > IC_Val2[1])
			{
				Difference[1] =IC_Val1[1]*(-1)-IC_Val2[1];
			}
			else{
							Difference[1] =0;
						}
			float refClock = TIMCLOCK/(PRESCALAR);
			float mFactor = 1000000/refClock;
			frequency[1] =(refClock/Difference[1]);
			usWidth[1] = Difference[1]*mFactor;
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_Captured[1] = 0; // set it back to false
		}
	}*/

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
/*	for(int i=0; i<4; i++) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if(i<0)
				i=0;
			if (Is_Captured[i]==0) // if the first rising edge is not captured
			{
				IC_Val1[i] = HAL_TIM_ReadCapturedValue(htim,channel_array[i]); // read the first value
				Is_Captured[i] = 1;  // set the first captured as true
			}

			else   // If the first rising edge is captured, now we will capture the second edge
			{
				IC_Val2[i/4] = HAL_TIM_ReadCapturedValue(htim,channel_array[i]);  // read second value

				if (IC_Val2[i] > IC_Val1[i])
				{
					Difference[i] = IC_Val2[i]-IC_Val1[i];
				}

				else if (IC_Val1[i] > IC_Val2[i])
				{
					Difference[i] = (0xffffffff - IC_Val1[i]) + IC_Val2[i];
				}

				float refClock = TIMCLOCK/(PRESCALAR);
				float mFactor = 1000000/refClock;
				frequency[i] =(refClock/Difference[i]);
				usWidth[i] = Difference[i]*mFactor;
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
				Is_Captured[i] = 0; // set it back to false
			}
		}
	}
*/
	/*if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {

		if (Is_second_Captured==0) // if the first rising edge is not captured
				{
					IC_S_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
					Is_second_Captured = 1;  // set the first captured as true
				}

		else   // If the first rising edge is captured, now we will capture the second edge
		{
			IC_S_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value

			if (IC_S_Val2 > IC_S_Val1)
			{
			Difference = IC_S_Val2-IC_S_Val1;
			}

			else if (IC_S_Val1 > IC_S_Val2)
			{
				Difference = (0xffffffff - IC_S_Val1) + IC_S_Val2;
			}

			float refClock = TIMCLOCK/(PRESCALAR);
			float mFactor = 1000000/refClock;
			frequency_2 =(refClock/Difference);
			usWidth_2 = Difference*mFactor;
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_second_Captured = 0; // set it back to false
		}
	}
}*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // TIMX->DIER, CCXIE bitini aktif eder.
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // TIMX->DIER, CCXIE bitini aktif eder.
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
