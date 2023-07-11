/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* Bootloader Prototypes */

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);
void bootloader_handle_go_cmd(uint8_t *pBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);
void bootloader_handle_en_rw_protect(uint8_t *pBuffer);
void bootloader_handle_mem_read (uint8_t *pBuffer);
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer);
void bootloader_handle_read_otp(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer);
void bootloader_handle_image_update(uint8_t *pBuffer);

void bootloader_send_ack(uint8_t follow_len);
void bootloader_send_nack(void);

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host);
uint8_t get_bootloader_version(void);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);

uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t total_number_of_sector);
uint8_t execute_mem_write(uint8_t* pBuffer, uint32_t dest_base_addr, uint32_t num_of_bytes);

uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);

uint16_t read_OB_rw_protection_status(void);

/* Image update helper functions */
void reset_image_settings(void);
uint32_t get_image_to_update(void);


//version 1.0
#define BL_VERSION 0x10

// our bootloader commands

//#define  <command name >	<command_code>

//This command is used to read the bootloader version from the MCU
#define BL_GET_VER				0x51

//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP				0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID				0x53

//This command is used to read the FLASH Read Protection level.
#define BL_GET_RDP_STATUS		0x54

//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR			0x55

//This command is used to mass erase or sector erase of the user flash .
#define BL_FLASH_ERASE          0x56

//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE			0x57

//This command is used to enable or disable read/write protect on different sectors of the user flash .
#define BL_EN_RW_PROTECT		0x58

//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ				0x59

//This command is used to read all the sector protection status.
#define BL_READ_SECTOR_P_STATUS	0x5A

//This command is used to read the OTP contents.
#define BL_OTP_READ				0x5B

//This command is used disable all sector read/write protection 
#define BL_DIS_R_W_PROTECT				0x5C

//This command is used to push image update
#define BL_UPDATE_IMAGE				    0x5D

/* ACK and NACK bytes*/
#define BL_ACK   0XA5
#define BL_NACK  0X7F

/*CRC*/
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0

#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01

#define INVALID_SECTOR 0x04

/*Some Start and End addresses of different memories of STM32F446xx MCU */
/*Change this according to your MCU */
#define SRAM1_SIZE            112*1024     // STM32F446RE has 112KB of SRAM1
#define SRAM1_END             (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE            16*1024     // STM32F446RE has 16KB of SRAM2
#define SRAM2_END             (SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE             512*1024     // STM32F446RE has 512KB of SRAM2
#define BKPSRAM_SIZE           4*1024     // STM32F446RE has 4KB of SRAM2
#define BKPSRAM_END            (BKPSRAM_BASE + BKPSRAM_SIZE)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* A/B Swap mechanism related macro definitions */
typedef struct {
  uint32_t crc;
  uint32_t size;
} IMAGE_data_TypeDef;

#define FLASH_SECTOR_1_BASE_ADDRESS 0x08004000U
#define FLASH_SECTOR_2_BASE_ADDRESS 0x08008000U
#define FLASH_SECTOR_3_BASE_ADDRESS 0x0800C000U
#define FLASH_SECTOR_4_BASE_ADDRESS 0x08010000U

#define FLASH_SECTOR_1_SIZE 0x04000U
#define FLASH_SECTOR_2_SIZE 0x04000U
#define FLASH_SECTOR_3_SIZE 0x04000U
#define FLASH_SECTOR_4_SIZE 0x10000U

#define IMAGE_A_BASE_ADDRESS        FLASH_SECTOR_2_BASE_ADDRESS
#define IMAGE_B_BASE_ADDRESS        FLASH_SECTOR_3_BASE_ADDRESS

#define IMAGE_MAX_SIZE              FLASH_SECTOR_2_SIZE

// Store Image Data in sector 1
#define IMAGE_A_DATA_BASE_ADDRESS   FLASH_SECTOR_1_BASE_ADDRESS
#define IMAGE_B_DATA_BASE_ADDRESS   IMAGE_A_DATA_BASE_ADDRESS + sizeof(IMAGE_data_TypeDef);

#define IMAGE_A_DATA                ((IMAGE_data_TypeDef *) IMAGE_A_DATA_BASE_ADDRESS)
#define IMAGE_B_DATA                ((IMAGE_data_TypeDef *) IMAGE_B_DATA_BASE_ADDRESS)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
