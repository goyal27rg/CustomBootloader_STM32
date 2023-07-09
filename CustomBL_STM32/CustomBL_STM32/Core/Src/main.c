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
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
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
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

#define COM_UART huart1
#define DEBUG_UART huart2

#define FLASH_SECTOR_2_BASE_ADDRESS 0x08008000U

uint8_t rx_buff[50];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void bootloader_sendMessage(UART_HandleTypeDef UARTx, char* str)
{
	HAL_UART_Transmit(&UARTx, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void bootloader_printDebugMsg(char* message, ...)
{
	char str[50];
	va_list args;
	va_start(args, message);
	vsprintf(str, message, args);
	bootloader_sendMessage(DEBUG_UART, str);
}

void bootloader_sendCommMsg(char* message, ...)
{
  char str[50];
	va_list args;
	va_start(args, message);
	vsprintf(str, message, args);
	bootloader_sendMessage(COM_UART, str);
}

void BL_uart_read_data(void)
{
	uint8_t rcv_len;
	
	/* Command format
	 * ---------------------------------------------------------------------------
	 * | 1 byte length | 1 byte command | <command specific fields> | 4 byte CRC |
	 * ---------------------------------------------------------------------------
   *
   * read length -> read rest of the frame -> decode cmd -> call relevant function
	*/
	
	while(1)
	{
    rcv_len = 0;
		bootloader_printDebugMsg("Enter Command\r\n");
		
		// Get the number of bytes to receive
		HAL_UART_Receive(&COM_UART, rx_buff, 1, HAL_MAX_DELAY);
    rcv_len = rx_buff[0];
		
    // Get the remaining frame
		HAL_UART_Receive(&COM_UART, &rx_buff[1], rcv_len, HAL_MAX_DELAY);
  	bootloader_printDebugMsg("RXed Cmd Code: 0x%x\r\n", rx_buff[1]);

		// decode the received command
		uint8_t cmd_code = rx_buff[1];
		switch(cmd_code)
		{
			case BL_GET_VER:
				bootloader_handle_getver_cmd(&rx_buff[1]);
				break;
			default:
				bootloader_printDebugMsg("INVALID COMMAND: 0x%x\r\n", cmd_code);
		}
	}
}

void BL_uart_jump_to_user_code(void)
{
	// User code shoould be placed at sector 2 of the Flash
	// First set the MSP for User App
	uint32_t MSP = *(volatile uint32_t *)FLASH_SECTOR_2_BASE_ADDRESS;
	bootloader_printDebugMsg("Setting MSP to 0x%X\r\n", MSP);
	__set_MSP(MSP);
	
	// Jump to User App reset handler
	uint32_t reset_handler_address = *(volatile uint32_t *) (FLASH_SECTOR_2_BASE_ADDRESS + 4);
	reset_handler_address |= 0x1;  // need the LSB to be '1' for Cortex-M
	bootloader_printDebugMsg("Reset Handler Address: 0x%x\r\n", reset_handler_address);
	void (*reset_handler) (void);
	reset_handler = (void*) reset_handler_address;
	bootloader_printDebugMsg("Jumping to USER APP...\r\n");
	reset_handler();
}

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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))  // check if User button PA0 pressed
		{	
			bootloader_printDebugMsg("Button pressed\r\n");
			BL_uart_read_data();
		}
		else
		{
			bootloader_printDebugMsg("Button released\r\n");
			BL_uart_jump_to_user_code();
		}
		
		uint32_t current_tick = HAL_GetTick();
		while(current_tick + 500 > HAL_GetTick());
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

void bootloader_handle_getver_cmd(uint8_t* cmd)
{
	bootloader_printDebugMsg("reached: BL_handle_get_version\r\n");
  bootloader_send_ack(1);
  uint8_t bl_version= BL_VERSION;
  bootloader_uart_write_data(&bl_version, 1);
}

void bootloader_send_ack(uint8_t follow_len)
{
  // follow_len is the length of the data to be sent after the
  // ACK from BL to the HOST
  uint8_t buff[2];
  buff[0] = BL_ACK;
  buff[1] = follow_len;
  HAL_UART_Transmit(&COM_UART, buff, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
  const uint8_t nack = BL_NACK;
  HAL_UART_Transmit(&COM_UART, &nack, 1, HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
  // computer the CRC of pData for length len and match with crc_host
  uint32_t crcVal = 0xff;

  for (uint32_t i=0; i<len; i++)
  {
    uint32_t data = pData[i];
    crcVal = HAL_CRC_Accumulate(&hcrc, &data, 1);
  }

  if (crcVal == crc_host)
  {
    return VERIFY_CRC_SUCCESS;
  }
  else
  {
    return VERIFY_CRC_FAIL;
  }
}

void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len)
{
  HAL_UART_Transmit(&COM_UART, pBuffer, len, HAL_MAX_DELAY);
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
