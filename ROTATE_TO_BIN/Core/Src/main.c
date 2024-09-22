/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM TIM4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define POSITION_PLASTIC 0
#define POSITION_METAL 80
#define POSITION_TRASH 180

#define POSITION_OPEN 100
#define POSITION_CLOSE 177


#define SERVO_BIN 0
#define SERVO_LID 1

#define FULL_LENGTH_0 10
#define FULL_LENGTH_1 11
#define FULL_LENGTH_2 12



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */
void usDelay(uint32_t uSec);
int ROTATE_BIN(uint8_t position);
int ROTATE_MOTOR(uint8_t degree, uint8_t servo);
void UPDATE_FULLNESS(uint8_t class);
float GET_DISTANCE();
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const float speedofsound = 0.0343/2;
float distance = 0;
uint8_t icFlag = 0;
uint8_t captureIdx=0;
uint32_t edge1Time=0, edge2Time=0;
uint8_t newDataIn = 0;
uint8_t percent_class[3]= {0, 0, 0};

char uartBuf[100];
char uartBuf_test[100];
char rx_data[2]; //buffer that contains the number (sent by pc) corresponding to the bin
char error_0[50] = "error: wrong input for motor control\r\n";
char flag_0[30] = "you told me to rotate\r\n";
char flag_1[30];
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
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1 ); //start the timer
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_data, 1); //whenever it receives 1 bytes at time, will interupt and jump to a callback

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // if a new data is sent through UART2
	  if (newDataIn == 1){

		  // treating an item
		  newDataIn = 0;

		  // put the right bin under the lid
		  ROTATE_BIN((uint8_t)rx_data[0] - 48);
		  // wait till it gets to the position
		  HAL_Delay(1000);
		  // update the status of the bins' fullnes
		  UPDATE_FULLNESS(rx_data[0] - 48);
		  sprintf(uartBuf, "{\"plastic\": %d, \"metal\":%d,\"other\":%d}\r\n", percent_class[0], percent_class[1], percent_class[2]);
		  HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
		  // wait till the sensor gets and sends the measurement
		  HAL_Delay(100);
		  // open the lid
		  ROTATE_MOTOR(POSITION_OPEN, SERVO_LID);
		  // wait till it opens
		  HAL_Delay(1500);

		  //close the lid
		  ROTATE_MOTOR(POSITION_CLOSE, SERVO_LID);

	  }
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
  htim3.Init.Period = 65535;
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1680-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 1680-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 1000-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 trig_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
		if(captureIdx == 0) //Fisrt edge
		{
			edge1Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //__HAL_TIM_GetCounter(&htim3);//

			captureIdx = 1;
		}
		else if(captureIdx == 1) //Second edge
		{
			edge2Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			captureIdx = 0;
			icFlag = 1;
		}
}
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  //debugging message
  //sends what it just received
  //HAL_UART_Transmit(&huart2, (uint8_t *)rx_data, strlen(rx_data), 10);

  newDataIn = 1;


}
void UPDAT_FULLNESS(uint8_t class){
		  distance = GET_DISTANCE();
		  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
		  usDelay(10);
		  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);
		  //Start IC timer
		  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
		  //Wait for IC flag
		  uint32_t startTick = HAL_GetTick();
		  do
		  {
		  	  if(icFlag) break;
		  }while((HAL_GetTick() - startTick) < 500);  //500ms
		  icFlag = 0;
		  HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

		  //Calculate distance in cm
		  if(edge2Time > edge1Time)
		  {
			  distance = ((edge2Time - edge1Time) + 0.0f)*speedofsound;
		  }
		  else
		  {
			  distance = 0.0f;
		  }
		  sprintf(uartBuf, "Distance (cm) = %.1f\r\n", distance);

		  //HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
		  HAL_Delay(1000);
}
int ROTATE_BIN(uint8_t position)
{

  //HAL_UART_Transmit(&huart2, (uint8_t *) flag_0, strlen(flag_0), 10);
  switch (position)
  {
  case 0:
    ROTATE_MOTOR(POSITION_PLASTIC, SERVO_BIN);
    break;
  case 1:
    ROTATE_MOTOR(POSITION_METAL, SERVO_BIN);
    break;
  case 2:
    ROTATE_MOTOR(POSITION_TRASH, SERVO_BIN);
    break;
  default:
	  //HAL_UART_Transmit(&huart2, (uint8_t *) error_0, strlen(error_0), 10);
    break;
  }
  return 0;
}
int ROTATE_MOTOR(uint8_t degree, uint8_t servo)
{
  uint8_t duty_cycle = degree * 0.64444 + 14;
  //sprintf(flag_1,"duty_cycle = %u \r\n",duty_cycle);
  //HAL_UART_Transmit(&huart2, (uint8_t *) flag_1, strlen(flag_1), 10);

  switch(servo)
  {
    case 0:
    	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, duty_cycle);
      break;
    case 1:
    	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, duty_cycle);
      break;
    default:
  	  //HAL_UART_Transmit(&huart2, (uint8_t *) error_0, strlen(error_0), 10);
      break;
  }


  return 0;
}


void UPDATE_FULLNESS(uint8_t class)
{
	// initialise internal variables
	uint8_t fullness = 0;
	float distance = 0;

	//  measure distance
	distance = GET_DISTANCE();

	// convert distance to percentage
	switch (class) {
		case 0:
			fullness = (FULL_LENGTH_0 - distance) / FULL_LENGTH_0 * 100;
			break;
		case 1:
			fullness = (FULL_LENGTH_1 - distance) / FULL_LENGTH_1 * 100;
			break;
		case 2:
			fullness = (FULL_LENGTH_0 - distance) / FULL_LENGTH_2 * 100;
			break;
		default:
			if (fullness > 100)
			{
				fullness = 100;
			}else
				if(fullness < 0)
				{
					fullness  = 0;
				}
			break;
	}


	//update fullness array
	percent_class[class] = fullness;

}
float GET_DISTANCE()
{
	float distance = 0;
	// set trig pin to HIGH for 10 us
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
	usDelay(10);

	// reset it to 0
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

	//Start IC timer
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

	//Wait for IC flag
	uint32_t startTick = HAL_GetTick();
	do
	{
		if(icFlag) break;
	}while((HAL_GetTick() - startTick) < 500);  //500ms

	icFlag = 0;
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

	//Calculate distance in cm
	if(edge2Time > edge1Time)
	{
		distance = ((edge2Time - edge1Time) + 0.0f)*speedofsound;
	}
	else
	{
		distance = 0.0f;
	}

	sprintf(uartBuf, "Distance (cm) = %.1f\r\n", distance);

	//HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	//HAL_Delay(1000);

	return distance;

}
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
