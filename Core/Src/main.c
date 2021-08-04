/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*############################################################################*/
/*gto include*/
#include "stdio.h"
#include "string.h"
#include "dwt_stm32_delay.h"	//gto

/*############################################################################*/

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*############################################################################*/
/*gto define*/
//set 4Mbps
#define 	USART_CR1_OVER8_Pos           		(15U)
#define 	USART_CR1_OVER8_Msk           		(0x1UL << USART_CR1_OVER8_Pos)     /*!< 0x00008000 */
#define 	USART_CR1_OVER8               		USART_CR1_OVER8_Msk                /*!<USART Oversampling by 8 enable */
#define		UART_OVERSAMPLING_8                 ((uint32_t)USART_CR1_OVER8)

#define 	STM_TX_EN			GPIO_PIN_12
#define 	SYNC_EN				GPIO_PIN_14

/*############################################################################*/

/*############################################################################*/
/*gto variables*/

uint8_t 	data[1] = "S";				//uart1 test send data
uint8_t 	spi_data[] = "1";			//spi1 test send data
uint8_t 	uart1_4M = 0x11;	//uart1 4Mbps test send data
uint8_t 	rx_data;

/*mslee state machine variables*/
#define   	START_CODE	 		'<'
#define   	END_CODE		 	'>'
#define   	START				1
#define   	PAYLOAD 			2
#define   	END    				3
#define   	LENGTH   			16
uint8_t   	rxd[40];
uint8_t   	status = START;
uint8_t   	rx_cnt;
uint8_t		rxdata;
uint8_t		data_receive_flag = 0;
uint8_t		uart2_key_Flag = 0;

/*############################################################################*/

/*############################################################################*/
/*uart interrupt test*/ //gto

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
			//debugPrintln(&huart1, "uart1 interrupt! ");
	        // When one data is received, an interrupt is generated.
			HAL_UART_Receive_IT(&huart1, &rx_data, 1);

	        // Send the received data.
			HAL_UART_Transmit(&huart1, &rx_data, 1, 10);
	}

	if (huart->Instance == USART2) {
			//debugPrintln(&huart2, "uart1 interrupt! ");
			// When one data is received, an interrupt is generated.
			HAL_UART_Receive_IT(&huart2, &rxdata, 1);

			// Send the received data.
			//HAL_UART_Transmit(&huart2, &rx_data, 1, 10);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1); // LED YELLOW ON
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1); // GPIO OUTPUT HIGH
			/*
			if (rx_data == 0x33){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1); // GPIO OUTPUT HIGH
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1); // GPIO OUTPUT HIGH
			}

			if (rx_data == 0x11){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0); // GPIO OUTPUT LOW
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0); // GPIO OUTPUT LOW
			}
			*/

			switch(status){

				case START:
					if( rxdata == START_CODE ) {
						rxd[0] = START_CODE ;
						rx_cnt = 1 ;
						status = PAYLOAD ;
						uart2_key_Flag = 0 ;
					}
					break ;

				case PAYLOAD :
					if( rxdata == START_CODE ) {
						rxd[0] = START_CODE ;
						rx_cnt = 1 ;
						status = PAYLOAD ;
					}
					else if( rxdata == END_CODE ) {
						if( rx_cnt == (LENGTH-1) )  {
							rxd[rx_cnt++] = rxdata ;
							uart2_key_Flag = 1 ;
						}
						status = START ;
					}
					else{
						if( rx_cnt < (LENGTH+2) )  rxd[rx_cnt++] = rxdata ;
						else {
							status = START ;
							rx_cnt = 0 ;
							uart2_key_Flag = 0 ;
						}
					}
					break ;
			}

	}
}

/*############################################################################*/

/*############################################################################*/
/*Send information (strings) to the console (PC) */

/*printf for uart1*/	//gto
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, 500);
	return len;
}

/*Function to write directly to UART*/	//gto
void debugPrint(UART_HandleTypeDef *huart1, char _out[]){
 HAL_UART_Transmit(huart1, (uint8_t *) _out, strlen(_out), 10);
}


/*Function to write to UART and new line termination*/ //gto
void debugPrintln(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
 char newline[2] = "\r\n";
 HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}

/*############################################################################*/

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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, SYNC_EN, 1); // GPIO PB14 OUTPUT HIGH -> SYNC_EN
  HAL_GPIO_WritePin(GPIOB, STM_TX_EN, 0); // GPIO PB14 OUTPUT HIGH -> SYNC_EN
  printf("\r\n ### START STM32F105 Slave Board ### \r\n");
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1); // LED RED ON

  HAL_UART_Receive_IT(&huart2, (uint8_t *) &rxdata, 1); // interrupt uart 2

  //HAL_UART_Receive_IT(&huart1, &rx_data, 1);

  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1); // GPIO OUTPUT HIGH
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1); // GPIO OUTPUT HIGH
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	HAL_UART_Transmit(&huart1, (uint8_t *)&uart1_4M, 2, 10);
	//DWT_Delay_us(1000000);

	if(uart2_key_Flag)
	{
		  uart2_key_Flag = 0;
		  printf("uart2 flag on \r\n");

		  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1); // GPIO OUTPUT HIGH

		  /*
		  if (LENGTH == 16){
			  HAL_UART_Transmit(&huart2, (uint8_t *) &rxd[i], 1, 10);
		  }
		  */

		  for (int i = 0; i < LENGTH; i++) {
			  HAL_UART_Transmit(&huart2, (uint8_t *) &rxd[i], 1, 10);
		  }

	}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

#if 0
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;	//7000000; 4Mbps
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;	// UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
#endif
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 7000000; //115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END USART1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
