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
#define FLASH_SECTOR_3_BASE_ADDRESS 0x0800C000U
#define FLASH_SECTOR_4_BASE_ADDRESS 0x08010000U

#define FLASH_SECTOR_2_SIZE 0x04000U
#define FLASH_SECTOR_3_SIZE 0x04000U
#define FLASH_SECTOR_4_SIZE 0x10000U

#define IMAGE_A_BASER_ADDRESS       FLASH_SECTOR_2_BASE_ADDRESS
#define IMAGE_B_BASER_ADDRESS       FLASH_SECTOR_3_BASE_ADDRESS

#define IMAGE_A_MAX_SIZE            FLASH_SECTOR_2_SIZE
#define IMAGE_B_MAX_SIZE            FLASH_SECTOR_3_SIZE

uint8_t rx_buff[200];

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
    memset(rx_buff, 0, 100);
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
				bootloader_handle_getver_cmd(rx_buff);
				break;
      case BL_GET_HELP:
        bootloader_handle_gethelp_cmd(rx_buff);
				break;
      case BL_GET_CID:
        bootloader_handle_getcid_cmd(rx_buff);
				break;
      case BL_GET_RDP_STATUS:
        bootloader_handle_getrdp_cmd(rx_buff);
				break;
      case BL_GO_TO_ADDR:
        bootloader_handle_go_cmd(rx_buff);
				break;
      case BL_FLASH_ERASE:
        bootloader_handle_flash_erase_cmd(rx_buff);
				break;
      case BL_MEM_WRITE:
        bootloader_handle_mem_write_cmd(rx_buff);
				break;
      case BL_EN_RW_PROTECT:
        bootloader_handle_en_rw_protect(rx_buff);
				break;
      case BL_MEM_READ:
        bootloader_handle_mem_read(rx_buff);
				break;
      case BL_READ_SECTOR_P_STATUS:
        bootloader_handle_read_sector_protection_status(rx_buff);
				break;
      case BL_OTP_READ:
        bootloader_handle_read_otp(rx_buff);
				break;
      case BL_DIS_R_W_PROTECT:
        bootloader_handle_dis_rw_protect(rx_buff);
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
	reset_handler_address |= 0x1;  // Make T bit 1
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

uint8_t get_bootloader_version(void)
{
  return BL_VERSION;
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

void bootloader_handle_getver_cmd(uint8_t* cmd)
{
	bootloader_printDebugMsg("reached: bootloader_handle_getver_cmd\r\n");
  bootloader_send_ack(1);
  uint8_t bl_version= get_bootloader_version();
  bootloader_uart_write_data(&bl_version, 1);
}

void bootloader_handle_gethelp_cmd(uint8_t* pBuffer)
{
  bootloader_printDebugMsg("reached: bootloader_handle_gethelp_cmd\r\n");
  //char str[] = "gethelp not implemented\r\n";
  char str [] = "";  //TODO(implement, later)
  bootloader_send_ack(strlen(str));
  bootloader_uart_write_data((uint8_t*)str, strlen(str));
}

void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	bootloader_printDebugMsg("reached: bootloader_handle_getcid_cmd\r\n");
  
	uint32_t mcu_idcode = DBGMCU->IDCODE;
	uint16_t dev_id = mcu_idcode & ((1 << 12) - 1);  // device id is [11:0]
	
	bootloader_send_ack(sizeof(dev_id));
	bootloader_uart_write_data((uint8_t*)&dev_id, sizeof(dev_id));
	
}

void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
	
	bootloader_printDebugMsg("reached: bootloader_handle_getrdp_cmd\r\n");
  
	// RDP is a part of Option Bytes in Flash memory interface
	// this can be read from FLASH_OPTCR[15:8]
	uint32_t optcr = FLASH->OPTCR;
	uint8_t rdp = (optcr & FLASH_OPTCR_RDP_Msk) >> FLASH_OPTCR_RDP_Pos;
	
	bootloader_send_ack(sizeof(rdp));
	bootloader_uart_write_data(&rdp, sizeof(rdp));

}

void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
  /* Frame format
	 * ---------------------------------------------------------------------------
	 * | 1 byte length | 1 byte command |  4 byte Jump-to Address   | 4 byte CRC |
	 * ---------------------------------------------------------------------------
   */
	bootloader_send_ack(1);
  uint8_t dummy_ack = 0x0;  //TODO(Implement a function to verify address and send correct ACK/NACK)
	bootloader_uart_write_data(&dummy_ack, sizeof(dummy_ack));

  bootloader_printDebugMsg("reached: bootloader_handle_go_cmd\r\n");
  uint32_t addr = *(uint32_t*) &pBuffer[2];

  /* the following code is to get the reset handler address from the incoming address
	 * and jump to it. Uncomment if needed
	
    uint32_t app_reset_handler_addr = *((uint32_t*)(addr + 4));
    app_reset_handler_addr |= 0x1;  // Make T bit 1
    bootloader_printDebugMsg("Jumping to address: 0x%x\r\n", app_reset_handler_addr);
    void (*app_reset_handler) (void) = (void*) app_reset_handler_addr;
    app_reset_handler();
	
	*/

	// Host has to make sure that correct address is being passed.
	// We just jump to the passed address
	addr |= 0x1;  // Make T bit 1
  bootloader_printDebugMsg("Jumping to address: 0x%x\r\n", addr);
  void (*jump_to) (void) = (void*) addr;
  jump_to();
}

void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
  /* Frame format
	 * -----------------------------------------------------------------------------------------------------------
	 * | 1 byte length | 1 byte command |  1 byte sector index | 1 byte total number of sectors | 4 byte CRC |
	 * -----------------------------------------------------------------------------------------------------------
   */

	bootloader_printDebugMsg("reached: bootloader_handle_flash_erase_cmd\r\n");
	bootloader_send_ack(1);

	uint8_t sector_idx = pBuffer[2];
	uint8_t nb_sectors = pBuffer[3];
	uint8_t status = execute_flash_erase(sector_idx, nb_sectors);
	bootloader_uart_write_data(&status, sizeof(status));
}

void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
  /* Frame format
	 * -----------------------------------------------------------------------------------------------------------
	 * | 1 byte length | 1 byte command |  4 byte Base Address | 1 byte length of payload | PAYLOAD | 4 byte CRC |
	 * -----------------------------------------------------------------------------------------------------------
   */
	
	// Flash needs to be erased before writing otherwise written data won't be persistent
	//TODO(identiy sector from Address and erase it)
	
	bootloader_printDebugMsg("reached: bootloader_handle_mem_write_cmd\r\n");
	bootloader_send_ack(1);

  uint8_t status = HAL_OK;

  uint32_t mem_base_addr = *((uint32_t*) &pBuffer[2]);
  uint8_t payload_len = pBuffer[6];

  execute_mem_write(&pBuffer[7], mem_base_addr, payload_len);

	bootloader_uart_write_data(&status, sizeof(status));
}


void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
}

void bootloader_handle_mem_read (uint8_t *pBuffer)
{
}

void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
}

void bootloader_handle_read_otp(uint8_t *pBuffer)
{
}

void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
}

uint8_t execute_mem_write(uint8_t* pBuffer, uint32_t dest_base_addr, uint32_t number_of_bytes)
{
  uint8_t *pMem = (uint8_t *)dest_base_addr;
	
	bootloader_printDebugMsg("reached: execute_mem_write");
	bootloader_printDebugMsg("Dest Mem base addr: 0x%x\r\n", pMem);
	bootloader_printDebugMsg("Length of Write:    %d\r\n", number_of_bytes);
  	
	HAL_FLASH_Unlock();
	
	FLASH->CR &= ~(FLASH_CR_PSIZE); // set program size to byte since we are writing byte-by-byte later
	FLASH->CR |= FLASH_CR_PG; // set PG in FLASH_CR

	while (number_of_bytes > 0)
  {
    // wait till flash is busy
		FLASH_WaitForLastOperation(HAL_MAX_DELAY);
		//while(FLASH->SR & (1 << 16));
		
		// copy data
		*pMem = *pBuffer;
		//bootloader_printDebugMsg("pMem: 0x%x, pPayload: 0x%x\r\n", *pMem, *pBuffer);
    pMem++;
    pBuffer++;
    number_of_bytes--;
		
		// wait for FLASH_SR.BSY to get cleared
		//while(FLASH->SR & FLASH_SR_BSY_Msk);
  }
	
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	FLASH->CR &= ~(FLASH_CR_PG);
	
	HAL_FLASH_Lock();
	return HAL_OK;
}

uint8_t execute_flash_erase(uint8_t sector_number , uint8_t total_number_of_sector)
{
	
	bootloader_printDebugMsg("reached: execute_flash_erase\r\n");

  FLASH_EraseInitTypeDef flashErase;
 	uint32_t sector_error = 0;
  uint8_t status;


	if (sector_number == 0xFF)  // 0xFF indicates mass erase
	{
		bootloader_printDebugMsg("Performing MASS ERASE\r\n");
		flashErase.TypeErase = FLASH_TYPEERASE_MASSERASE;
		flashErase.Banks = FLASH_BANK_BOTH;
	}
	else if (sector_number + total_number_of_sector <= FLASH_SECTOR_TOTAL)
	{
		bootloader_printDebugMsg("Erasing sectors..\r\n");
		bootloader_printDebugMsg("First sector: %d, number of sectors: %d\r\n", sector_number, total_number_of_sector);
		flashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
		flashErase.Sector =  sector_number;
		flashErase.NbSectors = total_number_of_sector;
		flashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	}
	else
	{
		bootloader_printDebugMsg("INVALID SECTOR CHOICE!!!\r\n");
		bootloader_uart_write_data(&status, sizeof(status));
		return HAL_ERROR;
	}
	
	HAL_FLASH_Unlock();
	bootloader_printDebugMsg("Begin Erase...\r\n");
	status = HAL_FLASHEx_Erase(&flashErase, &sector_error);
	bootloader_printDebugMsg("Erase Done. Status: 0x%x\r\n", status);
	HAL_FLASH_Lock();

  return status;
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
