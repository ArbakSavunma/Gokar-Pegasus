
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define MPU6050_ADDR 0x68<<1
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C
#define LPF_REG 0x1A
#define KP_roll 0.5
#define KI_roll 0.001
#define KD_roll 500
#define KP_pitch 0.5
#define KI_pitch 0.001
#define KD_pitch 500
#define KP_yaw 0.5
#define KI_yaw 0.001
#define KD_yaw 500
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t data;
uint8_t buffer[2],tuffer[6],cuffer[6]; // ham gyro ve acc değerlerinin tutulduğu listeler
int16_t gyro_raw[3],acc_raw[3];        // anlamlandırılmış ham değerler
float gyro_cal[3],acc_cal[3]; // kalibrasyon offsetlerini tutan liste
int16_t acc_total_vector;
float angle_pitch_gyro, angle_roll_gyro, angle_yaw_gyro;
float angle_pitch_acc, angle_roll_acc, angle_yaw_acc, angle_pitch_acc_sum, angle_roll_acc_sum, angle_yaw_acc_sum, acc_total_vector_sum;
float angle_pitch,angle_roll, angle_yaw;
int16_t raw_temp;
float temp;
int i;
float prevtime,prevtime1,time1,elapsedtime1,prevtime2,time2,elapsedtime2;
HAL_StatusTypeDef set_gyro;
double roll_pid_d, roll_pid_i, roll_last_error, roll_control_signal;
double pitch_pid_d, pitch_pid_i, pitch_last_error, pitch_control_signal;
double yaw_pid_d, yaw_pid_i, yaw_last_error, yaw_control_signal;
int32_t throttle=0;
int32_t motpower1=0, motpower2=0, motpower3=0, motpower4=0;
/* USER CODE END PV */
int32_t prewYaw=0, prewRoll=0, prewPitch=0;
volatile uint32_t ch1_rising = 0, ch2_rising = 0, ch3_rising = 0, ch4_rising = 0;
volatile uint32_t ch1_falling = 0, ch2_falling = 0, ch3_falling = 0, ch4_falling = 0;
volatile uint32_t pre_ch1 = 0, ch1 = 0, pre_ch2 = 0, ch2 = 0, pre_ch3 = 0, ch3 = 0, pre_ch4 = 0, ch4 = 0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void calculateMotorPowers(double PrRoll, double PrPitch, double PrYaw);
double getControlSignal(double error, double kp, double ki, double kd, double pid_d, double pid_i, double last_error, float delta_time);
void calibrationofmot();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
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
}
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // TIMX->DIER, CCXIE bitini aktif eder.
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* USER CODE BEGIN 2 */
  data = 0x00;
    HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
    //kullanılan birimin pointer olarak döndürülmesi, cihaz slave adresi, register adresi, register boyutu, verinin pointer
    //olarak döndürülmesi, verinin boyutu , zaman aşımıı değeri
    // GYRO CNFG --> +-500 derece/saniye --> 08
    data = 0x08;
    HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
    // ACC CNFG --> +-8g --> 10
    data = 0x10;
    HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x03;
    HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, LPF_REG, 1, &data, 1, HAL_MAX_DELAY);

    for(i=0; i<2000; i++)
    {
  	  prevtime2 = time2;
  	  time2 = HAL_GetTick();
  	  elapsedtime2=(time2-prevtime2)*1000;

  	  cuffer[0]=0x43;
  	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,cuffer,1,HAL_MAX_DELAY);
  	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,cuffer,6,HAL_MAX_DELAY);

  	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
  	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
  	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

  	  gyro_cal[0] += gyro_raw[0];
  	  gyro_cal[1] += gyro_raw[1];
  	  gyro_cal[2] += gyro_raw[2];

  	  HAL_Delay(3);

    }

    gyro_cal[0] /= 2000;
    gyro_cal[1] /= 2000;
    gyro_cal[2] /= 2000;


    HAL_Delay(500);
  /* USER CODE END 2 */
    calibrationofmot();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  int32_t rec[]= {ch1,ch2,ch4};
	  prevtime1 = time1;
	  time1 = HAL_GetTick();
	  elapsedtime1=(time1-prevtime1)*1000;

	  tuffer[0]=0x3B;
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,tuffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,tuffer,6,HAL_MAX_DELAY);

	  // Acc Raw Values
	  acc_raw[0] = (tuffer[0] << 8 | tuffer[1]);
	  acc_raw[1] = (tuffer[2] << 8 | tuffer[3]);
	  acc_raw[2] = (tuffer[4] << 8 | tuffer[5]);


	  buffer[0]=0x41;
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,buffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,buffer,2,HAL_MAX_DELAY);

	  // Temperature Values
	  raw_temp = (buffer[0] << 8 | buffer[1]);
	  temp = (raw_temp / 340.0) + 36.53;


	  cuffer[0]=0x43;
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,cuffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,cuffer,6,HAL_MAX_DELAY);

	  // Gyro Raw Values
	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

	  gyro_raw[0] -= gyro_cal[0];
	  gyro_raw[1] -= gyro_cal[1];
	  gyro_raw[2] -= gyro_cal[2];


	  angle_pitch_gyro += gyro_raw[0] * 0.0000611;
	  angle_roll_gyro += gyro_raw[1] * 0.0000611;
	  angle_yaw_gyro += gyro_raw[2] * 0.0000611;

	  angle_pitch_gyro += angle_roll_gyro * sin(gyro_raw[2] * 0.000001066);
	  angle_roll_gyro -= angle_pitch_gyro * sin(gyro_raw[2] * 0.000001066);
	  angle_yaw_gyro -= angle_yaw_gyro * sin(gyro_raw[1] * 0.000001066);

	  acc_total_vector = sqrt((acc_raw[0]*acc_raw[0])+(acc_raw[1]*acc_raw[1])+(acc_raw[2]*acc_raw[2]));

	  //57.296 = 1 / (3.142 / 180)
	  angle_pitch_acc = asin((float)acc_raw[1]/acc_total_vector)* 57.296;
	  angle_roll_acc = asin((float)acc_raw[0]/acc_total_vector)* -57.296;
	  angle_yaw_acc = asin((float)acc_raw[2]/acc_total_vector)* 57.296;

	  angle_pitch_acc -= 1;//0.05;
	  angle_roll_acc -= -1.32;//-1.32;
	  angle_yaw_acc -= 0;
	  if(set_gyro){
		  angle_pitch = angle_pitch_gyro * 0.9996 + angle_pitch_acc * 0.0004;
		  angle_roll = angle_roll_gyro * 0.9996 + angle_roll_acc * 0.0004;
		  angle_yaw = angle_yaw_gyro * 0.9996 + angle_yaw_acc * 0.0004;
	  }
	  else
	  {
		  angle_pitch = angle_pitch_acc;
		  set_gyro = true;
	  }

	  while((HAL_GetTick() - prevtime)*1000 < 4000);
	  prevtime = HAL_GetTick();
	  throttle = (ch3/4.60);
	  for(int i=0; i<3; i++) {
		  rec[i]= (rec[i]/12.5)-116.8;
		  if(i==1)
			  rec[i]-=4;
	  }
	  prewRoll=-1*rec[0]; prewPitch= -1*rec[1]; prewYaw= rec[2];
	  if(throttle>420)
		  throttle=420;
	  calculateMotorPowers(prewRoll,prewPitch,prewYaw);

	  TIM2->CCR1 = motpower1;
	  TIM2->CCR2 = motpower2;
	  TIM2->CCR3 = motpower3;
	  TIM2->CCR4 = motpower4;

    /*	 USER CODE BEGIN 3 */
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 410-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4096-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void calibrationofmot() {
	 TIM2->CCR1 = 420;
	 TIM2->CCR2 = 420;
	 TIM2->CCR3 = 420;
	 TIM2->CCR4 = 420;
	 HAL_Delay(3000);

	 TIM2->CCR1 = 220;
	 TIM2->CCR2 = 220;
	 TIM2->CCR3 = 220;
	 TIM2->CCR4 = 220;
	 HAL_Delay(3000);


}
void calculateMotorPowers(double PrRoll, double PrPitch, double PrYaw) {
  double rollError = PrRoll - angle_roll;

  double yawError = PrYaw - angle_yaw;

  double pitchError = PrPitch - angle_pitch;


  pitch_control_signal =  getControlSignal(pitchError, KP_pitch, KI_pitch, KD_pitch, pitch_pid_d, pitch_pid_i, pitch_last_error, elapsedtime1);
  pitch_last_error= pitchError;
  yaw_control_signal =  getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, yaw_pid_d, yaw_pid_i, yaw_last_error, elapsedtime1);
  yaw_last_error= yawError;
  roll_control_signal = getControlSignal(rollError, KP_roll, KI_roll, KD_roll, roll_pid_d, roll_pid_i, roll_last_error, elapsedtime1);
  roll_last_error= rollError;
  motpower1 = throttle + (int32_t)(-1*roll_control_signal - yaw_control_signal + pitch_control_signal);
  if(motpower1<229) motpower1=229;
  if(motpower1>420) motpower1=420;
  motpower2 = throttle + (int32_t)(roll_control_signal + yaw_control_signal + pitch_control_signal);
  if(motpower2<210) motpower2=210;
  if(motpower2>409) motpower2=409;
  motpower3 = throttle + (int32_t)(roll_control_signal - yaw_control_signal - pitch_control_signal);
  if(motpower3<220) motpower3=220;
  if(motpower3>420) motpower3=420;
  motpower4 = throttle + (int32_t)(-1*roll_control_signal + yaw_control_signal - pitch_control_signal);
  if(motpower4<220) motpower4=220;
  if(motpower4>420) motpower4=420;
}

double getControlSignal(double error, double kp, double ki, double kd, double pid_d, double pid_i, double last_error, float delta_time) {
  double pid_p = error;
  pid_d = (error - last_error) / delta_time;
  pid_i += error * delta_time;
  if(error<2 && error>-2)
	  pid_i=0;
  double control_signal = (kp*pid_p) + (ki*pid_i) + (kd*pid_d);
  //last_error = error;  not:bu ifadeyi fonksiyondan hemen sonra yazacağız.
  return control_signal;
}
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
