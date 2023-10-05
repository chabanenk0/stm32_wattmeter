/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v2.0_Cube
  * @brief          : Memory management layer.
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
#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

// changes taken from https://community.st.com/t5/stm32-mcus-embedded-software/stm32f4-mass-storage-class-with-internal-flash/td-p/447929
#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  200  //256 blocks * 512 = 128k
#define STORAGE_BLK_SIZ                  512 //doesn't seem to work well with values other than 512

#define FLASH_STORAGE_START 0x08020000
#define FLASH_STORAGE_START_PAGE ((FLASH_STORAGE_START - FLASH_BASE)/FLASH_PAGE_SIZE)

#define STORAGE_BLK_PER_PAGE (FLASH_PAGE_SIZE/STORAGE_BLK_SIZ) //for the F433 would be 4
/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes over USB FS IP
  * @param  lun:
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  .
  * @param  lun: .
  * @param  block_num: .
  * @param  block_size: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */
  memcpy(buf, (const void *)(FLASH_STORAGE_START + blk_addr * STORAGE_BLK_SIZ), blk_len * STORAGE_BLK_SIZ);

  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */

FLASH_EraseInitTypeDef eraseStruct;
eraseStruct.TypeErase=FLASH_TYPEERASE_PAGES;
eraseStruct.PageAddress = FLASH_STORAGE_START_PAGE + blk_addr;
eraseStruct.NbPages = 1;
eraseStruct.Banks = 0;
uint32_t PageError;
uint64_t *pBuf64;
pBuf64 = (uint64_t *) buf;

//buffer to hold one full page
static uint8_t pageShadow[FLASH_PAGE_SIZE];
uint32_t targetFlashPage, lastTargetPage;
uint32_t targetPageOffset;
lastTargetPage = 0xFFFFFFFF; //some invalid page

HAL_FLASH_Unlock();

for (uint32_t blk_index = blk_addr; blk_index < (blk_addr + blk_len); blk_index++) {

  //copy the contents of the whole page before erasing
  targetFlashPage = FLASH_STORAGE_START_PAGE + (blk_index / STORAGE_BLK_PER_PAGE);
  targetPageOffset = (blk_index % STORAGE_BLK_PER_PAGE) * STORAGE_BLK_SIZ;

  //are we still writing to the page we wrote last?
  if (lastTargetPage != targetFlashPage) {
     //No, this is a different page
     //copy the contents of the page to a buffer
     memcpy(pageShadow, (const void *) (FLASH_BASE + targetFlashPage * FLASH_PAGE_SIZE), FLASH_PAGE_SIZE);

    //erase the page
    eraseStruct.PageAddress = targetFlashPage;
    eraseStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&eraseStruct, &PageError);

    //no need to call FLASH_WaitForLastOperation() here
    //remember the page we're on

    lastTargetPage = targetFlashPage;

  }

  //copy one block to the buffer
  memcpy(&pageShadow[targetPageOffset], &buf[(blk_index - blk_addr) * STORAGE_BLK_SIZ], STORAGE_BLK_SIZ);

  //if we're about to change pages or this is the end, commit to Flash
  //Use DWORD (64-bit) access when programming (I expect it to be faster)

  if (((blk_index % STORAGE_BLK_PER_PAGE) == (STORAGE_BLK_PER_PAGE - 1))
    || blk_index == (blk_addr + blk_len - 1)) {
    pBuf64 = (uint64_t*) pageShadow;
    for (uint32_t dword_index = 0; dword_index < FLASH_PAGE_SIZE; dword_index += 4) {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, targetFlashPage * FLASH_PAGE_SIZE + dword_index, *pBuf64);
      FLASH_WaitForLastOperation(1000); //not sure if this is required
      pBuf64++; //next 64 bits
    }
  }
}

//lock the flash to prevent accidental changes

HAL_FLASH_Lock();

  return (USBD_OK);
  /* USER CODE END 7 */
}

/**
  * @brief  .
  * @param  None
  * @retval .
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

