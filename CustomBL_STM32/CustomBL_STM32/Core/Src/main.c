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
#include <math.h>
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
      case BL_UPDATE_IMAGE:
        bootloader_handle_image_update(rx_buff);
				break;
			default:
				bootloader_printDebugMsg("INVALID COMMAND: 0x%x\r\n", cmd_code);
		}
	}
}

void bootloader_set_msp_and_jump_to_reset_handler(uint32_t image_base_address)
{
	// First set the MSP
	uint32_t MSP = *((volatile uint32_t *) image_base_address);
	bootloader_printDebugMsg("Setting MSP to 0x%X\r\n", MSP);
	__set_MSP(MSP);
	
	// Calculate and Jump to reset handler
	uint32_t reset_handler_address = *((volatile uint32_t *) (image_base_address + 4));
	reset_handler_address |= 0x1;  // Make T bit 1
	
	bootloader_printDebugMsg("Reset Handler Address: 0x%x\r\n", reset_handler_address);
	
	void (*reset_handler) (void);
	reset_handler = (void*) reset_handler_address;
	bootloader_printDebugMsg("Jumping to image...\r\n");
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
			/*
			Image selection policy:
			1/ Load image A and verify CRC
			2/ Load image B and verify CRC
			3/ if both CRC are correct, load the one with higher version number
			3.1/ if version is same, default to A
			3.2/ TODO(if version is different, fisrt update the one with lower version, later)
			4/ if only one CRC is correct, first update the other one
			5/ if both CRC are incorrect, print an error message and wait for reset
			*/
			IMAGE_data_TypeDef *pImageAData = IMAGE_A_DATA;
			IMAGE_data_TypeDef *pImageBData = IMAGE_B_DATA;
			
			pImageAData->image_base_address = IMAGE_A_BASE_ADDRESS;
			pImageBData->image_base_address = IMAGE_B_BASE_ADDRESS;
			
			uint8_t imageACrcValid = verify_image_crc(pImageAData) == VERIFY_CRC_SUCCESS;
			bootloader_printDebugMsg("Validating CRC of IMAGE A. CRC Valid: %d\r\n", imageACrcValid);
			
			uint8_t imageBCrcValid = verify_image_crc(pImageBData) == VERIFY_CRC_SUCCESS;
			bootloader_printDebugMsg("Validating CRC of IMAGE B. CRC Valid: %d\r\n", imageBCrcValid);
			
			bootloader_printDebugMsg("CRC validation complete\r\n");
			
			if (imageACrcValid && imageBCrcValid)
			{
				// case when both images have correct CRC
				bootloader_printDebugMsg("\r\nBoth images VALID.\r\n");
				if (pImageAData->version >= pImageBData->version)
				{
					// TODO(also update the other image, later)
					bootloader_boot_from_image(pImageAData);
				}
				else
				{
  				// TODO(also update the other image, later)
					bootloader_boot_from_image(pImageBData);
				}
			}
			else if (imageACrcValid)
			{
				// only image A CRC was correct
  			    // TODO(also update the other image instead of just erasing, later)
				IMAGE_data_TypeDef resetImageB = bootloader_get_reset_image_settings(pImageBData);
				execute_flash_erase(resetImageB.sector, 1);
				bootloader_write_image_settings(pImageBData, resetImageB);

				bootloader_boot_from_image(pImageAData);
			}
			else if (imageBCrcValid)
			{
				// only image B CRC was correct
				// TODO(also update the other image instead of just erasing, later)
				IMAGE_data_TypeDef resetImageA = bootloader_get_reset_image_settings(pImageAData);
				execute_flash_erase(resetImageA.sector, 1);
				bootloader_write_image_settings(pImageAData, resetImageA);
				
				bootloader_boot_from_image(pImageBData);
			}
			else
			{
				// both images are invalid
				bootloader_printDebugMsg("ERROR !!!\r\nBOTH IMAGES ARE INVALID.\r\n");

				IMAGE_data_TypeDef resetImageA = bootloader_get_reset_image_settings(pImageAData);
				IMAGE_data_TypeDef resetImageB = bootloader_get_reset_image_settings(pImageBData);
				
				// erase the sectors
				execute_flash_erase(resetImageA.sector, 1);
				execute_flash_erase(resetImageB.sector, 1);
				
				bootloader_write_image_settings(pImageAData, resetImageA);
				bootloader_write_image_settings(pImageBData, resetImageB);

				bootloader_printDebugMsg("\r\nRESET AND REPROGRAM.\r\n");
				//bootloader_printDebugMsg("BEWARE THAT IMAGES MUST BE COMPILED TO BE PLACED AT SECTOR: %d\r\n", IMAGE_PLACEMENT_SECTOR);

				while(1);
			}
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

uint32_t get_CRC32(uint8_t *pData, uint32_t len)
{
	// compute the CRC of pData for length len and match with expected_crc
  uint32_t crcVal = 0xff;

	/* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);
	
  for (uint32_t i=0; i<len; i++)
  {
    uint32_t data = pData[i];
    crcVal = HAL_CRC_Accumulate(&hcrc, &data, 1);
  }
	 /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);
	
	return crcVal;
}

uint8_t verify_crc (uint8_t *pData, uint32_t len, uint32_t expected_crc)
{
  if (get_CRC32(pData, len) == expected_crc)
  {
    return VERIFY_CRC_SUCCESS;
  }
  else
  {
    /*bootloader_printDebugMsg("BL CRC: 0x%x\r\nReceived:\r\n", crcVal);
		for (uint32_t i=0; i<len; i++)
		{
			bootloader_printDebugMsg(" 0x%x ", pData[i]);
		}*/
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

  flash_execute_mem_write(&pBuffer[7], mem_base_addr, payload_len);

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

void bootloader_handle_image_update(uint8_t *pBuffer)
{
  /* Frame format
	 * -------------------------------------------------------------
	 * | 1 byte length | 1 byte command | 4 bytes total image size |
	 * -------------------------------------------------------------
   */

   /* After receiving the command, image would be received in subsequent frames of 128 byte each
    * (except the last frame which can have any number of bytes from [1 - 128])
    *
    * Image Frame format
    * ---------------------------------------------------------
	  * | 1 byte length of image data | image data | 4 byte CRC |
	  * ---------------------------------------------------------
    */
  
 	bootloader_printDebugMsg("reached: bootloader_handle_image_update\r\n");
	 	
  /* All the frame reception related stuff */
	uint32_t rx_image_size = *((uint32_t *) &pBuffer[2]);	
  if (rx_image_size > (uint32_t) IMAGE_MAX_SIZE)
  {
    bootloader_send_nack();
		return;
  }

  bootloader_send_ack(0);

  uint8_t status = HAL_OK;
	
	// calculate number of frames to receive
	uint8_t total_image_frames = rx_image_size / 128;
	if (rx_image_size % 128 != 0)
	{
		total_image_frames++;
	}
	
  uint8_t image_rx_buff[150];
	uint8_t image_frame_data_len;
	uint8_t num_frames_received = 0;
	uint8_t ready = 0x1;
	
	bootloader_printDebugMsg("image size: %d\r\n", rx_image_size);
	bootloader_printDebugMsg("Frames to be received: %d\r\n", total_image_frames);

	// Get Image-to-update data	
	IMAGE_data_TypeDef* pImageData = get_image_to_update();
	// copy on temp struct as pImage* data wil get erased
	IMAGE_data_TypeDef temp_pImageData = *pImageData;
	
	// We don't know if *pImageData is valid or not, so use another object with default settings
	IMAGE_data_TypeDef resetImage = bootloader_get_reset_image_settings(pImageData);
	uint32_t image_to_update_address = resetImage.image_base_address;
	
	// perform a sector erase before writing
	if (execute_flash_erase(resetImage.sector , 1) != HAL_OK)
	{
		bootloader_printDebugMsg("!!! ERROR while erasing flash sector %d.\r\nReturning..\r\n", resetImage.sector);
		return;
	}

  // get all image frames from the HOST and write to mem
	while(num_frames_received < total_image_frames)
	{
		memset(image_rx_buff, 0, 150);
		
		bootloader_uart_write_data(&ready, 1);
		// receive the image length from HOST
		while(HAL_UART_Receive(&COM_UART, image_rx_buff, 1, HAL_MAX_DELAY) != HAL_OK);
		image_frame_data_len = image_rx_buff[0];

		// receive the remaining image frame from HOST
		HAL_UART_Receive(&COM_UART, &image_rx_buff[1], image_frame_data_len + 4, HAL_MAX_DELAY);  // +4 for CRC bytes
			
		// verify CRC
		uint32_t crc_host = *((uint32_t *) (image_rx_buff + image_frame_data_len + 1));  // last 4 bytes of the frame
		uint8_t crc_status = verify_crc(image_rx_buff, image_frame_data_len + 1, crc_host);
		
		if (crc_status == VERIFY_CRC_FAIL)
		{
			bootloader_send_nack();
			bootloader_printDebugMsg("\nCRC CHECK FAILED !!! -- host CRC: 0x%x\r\n", crc_host);
			return;
		}
		bootloader_send_ack(1);  // CRC check passed
		
		// write to mem and update address
		flash_execute_mem_write(&image_rx_buff[1], image_to_update_address, (uint32_t) image_frame_data_len);
		image_to_update_address += image_frame_data_len;
		num_frames_received++;
		
		bootloader_printDebugMsg("Frames Received: %d/%d\r", num_frames_received, total_image_frames);
		bootloader_uart_write_data(&status, 1);	
	}
	
	// Image has been written
	// Perform post processing	
	uint8_t temp_buff[pImageData->size];
	
	//TODO(version may also be incorrect if *pImageData was corrupt, figure out a  way, later)
	resetImage.version = temp_pImageData.version + 1;
	resetImage.size = rx_image_size;
	resetImage.crc = get_CRC32((uint8_t*)resetImage.image_base_address, rx_image_size);
	
	// remember, we can write only once after erasing.
	bootloader_write_image_settings(pImageData, resetImage);
	
	bootloader_printDebugMsg("IMAGE %c written successfully.\r\n", bootloader_get_image_name(pImageData));
	bootloader_printDebugMsg("Version: %d, Size: %d, CRC: 0x%x\r\n", pImageData->version, pImageData->size, pImageData->crc);
	bootloader_printDebugMsg("Version: %d, Size: %d, CRC: 0x%x\r\n", resetImage.version, resetImage.size, resetImage.crc);
}

IMAGE_data_TypeDef bootloader_get_reset_image_settings(IMAGE_data_TypeDef* pImageData)
{
	/*
	Reset:
	1/ CRC to 0xFFFFFFFF
	2/ size to 0
	3/ image_base_address to macro defined address
	4/ sector to macro defined sector
	5/ version to 0
	DONT FORGET! -> Flash can't be written directly, so create a temp obj and copy it
	*/
	
	IMAGE_data_TypeDef temp;
	temp.crc = 0xFFFFFFFF;
	temp.size = 0;
	temp.version = 0;
	
	if (pImageData == IMAGE_A_DATA)
	{
		temp.image_base_address = IMAGE_A_BASE_ADDRESS;
		temp.sector = IMAGE_A_SECTOR;
	}
	else
	{
		temp.image_base_address = IMAGE_B_BASE_ADDRESS;
		temp.sector = IMAGE_B_SECTOR;
	}
	
	// Don't write here as flash needs to be erased for that
	//bootloader_write_image_settings(pImageData, temp);
	
	return temp;
}


void bootloader_write_image_settings(IMAGE_data_TypeDef* pImageData, IMAGE_data_TypeDef ImageDataToWrite)
{
	bootloader_printDebugMsg("Writing %c Image settings\r\n", bootloader_get_image_name(pImageData));
	flash_execute_mem_write((uint8_t*) &ImageDataToWrite, (uint32_t) pImageData, sizeof(IMAGE_data_TypeDef));
}

uint8_t verify_image_crc(IMAGE_data_TypeDef* pImageData)
{
		//TODO(check if this is needed, later)
	if (pImageData->crc == 0xFFFFFFFF) {
		return VERIFY_CRC_FAIL;
	}
	return verify_crc((uint8_t *) pImageData->image_base_address, pImageData->size, pImageData->crc);
}

IMAGE_data_TypeDef* get_image_to_update(void)
{
	/*
	Find which image is to be updated
	Erase the sector of that image
	
	Procedure:
	1/ verify crc of both images
	2/ if crc verification of an image fails, select it
	3/ if crc verification of both images passes, pick the one with lower version number
	4/ if versions are same, pick image A
	*/
	
	bootloader_printDebugMsg("reached: get_image_to_update\r\n");
	
	IMAGE_data_TypeDef* pImageAData = IMAGE_A_DATA;
	IMAGE_data_TypeDef* pImageBData = IMAGE_B_DATA;

	if (verify_image_crc(pImageAData) == VERIFY_CRC_FAIL)
	{
		bootloader_printDebugMsg("IMAGE A CRC Failed\r\n");
		return pImageAData;
	}
	if (verify_image_crc(pImageBData) == VERIFY_CRC_FAIL)
	{
		bootloader_printDebugMsg("IMAGE B CRC Failed\r\n");
		return pImageBData;
	}
	bootloader_printDebugMsg("versions A, B: %d, %d\r\n", pImageAData->version, pImageAData->version);
	if (pImageAData->version <= pImageBData->version)
	{
		return pImageAData;
	}
	else
	{
		return pImageBData;
	}
}

uint8_t flash_execute_mem_write(uint8_t* pBuffer, uint32_t dest_base_addr, uint32_t number_of_bytes)
{
  uint8_t *pMem = (uint8_t *)dest_base_addr;
	
	//bootloader_printDebugMsg("reached: flash_execute_mem_write\r\n");
	//bootloader_printDebugMsg("Dest Mem base addr: 0x%x\r\n", pMem);
	//bootloader_printDebugMsg("Length of Write:    %d\r\n", number_of_bytes);
  	
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
		return HAL_ERROR;
	}
	
	HAL_FLASH_Unlock();
	bootloader_printDebugMsg("Begin Erase...\r\n");
	while(HAL_FLASHEx_Erase(&flashErase, &sector_error) != HAL_OK);
	bootloader_printDebugMsg("Erase Done.\r\n");
	HAL_FLASH_Lock();

  return HAL_OK;
}

void bootloader_boot_from_image(IMAGE_data_TypeDef* pImageData)
{
	/*
	to boot from an image,
	1/ first Erase the placement sector (IMAGE_PLACEMENT_SECTOR)
	2/ Then load the image to placement address (IMAGE_PLACEMENT_ADDRESS)
	3/ Get the reset handler and jump to it
	*/

	char image_name = bootloader_get_image_name(pImageData);

	execute_flash_erase(IMAGE_PLACEMENT_SECTOR, 1);
	bootloader_printDebugMsg("\r\nLoading IMAGE %c to SECTOR %d\r\n", image_name, IMAGE_PLACEMENT_SECTOR);
	flash_execute_mem_write((uint8_t *) pImageData->image_base_address, (uint32_t) IMAGE_PLACEMENT_ADDRESS, pImageData->size);
	
	bootloader_printDebugMsg("Booting from IMAGE: %c\r\n", image_name);	
	
	bootloader_set_msp_and_jump_to_reset_handler((uint32_t) IMAGE_PLACEMENT_ADDRESS);
}

char bootloader_get_image_name(IMAGE_data_TypeDef* pImageData)
{
	if (pImageData == IMAGE_A_DATA)
	{
		return 'A';
	}
	else
	{
		return 'B';
	}
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
