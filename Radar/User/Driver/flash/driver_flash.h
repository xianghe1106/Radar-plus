/*
 * driver_flash.h
 *
 *  Created on: Dec 21, 2018
 *      Author: xianghe
 */

#ifndef USER_DRIVER_FLASH_DRIVER_FLASH_H_
#define USER_DRIVER_FLASH_DRIVER_FLASH_H_



/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "cpu.h"
#include "xmc1_flash.h"
#include "driver.h"

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/

typedef enum
{
	flash_page_factory 	= 0,
	flash_page_user 	= 1
}FLASH_PAGE_Type;

typedef enum
{
	FLASH_NO_ERROR			= 0,
	FLASH_ADDRESS_NOT_MATCH = 1,
	FLASH_PAGE_UNDEFINED 	= 2
}FLASH_ERROR_CODE_Type;

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

//extern INT8U  flash_buffer[256];

/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/

#define FLASH_BUFFER_SIZE						XMC_FLASH_BYTES_PER_PAGE

/** Physical flash Block size and Page size defined*/
#define E_EEPROM_XMC1_FLASH_BLOCK_BYTE_SIZE  (16U)
#define E_EEPROM_XMC1_FLASH_PAGE_BYTE_SIZE   (256U)

/*
 *  Flash address and Size informations as per user configuration
 */
#define E_EEPROM_XMC1_FLASH_TOTAL_SIZE     (1024U)
#define E_EEPROM_XMC1_FLASH_BANK_SIZE      (512U)

/*
 *  EMULATED_EEPROM Bank, start and end addresses
 */
#define E_EEPROM_XMC1_FLASH_BANK0_BASE     (0x10008c00U)
#define E_EEPROM_XMC1_FLASH_BANK0_END      (0x10008dffU)
#define E_EEPROM_XMC1_FLASH_BANK1_BASE     (0x10008e00U)
#define E_EEPROM_XMC1_FLASH_BANK1_END      (0x10008fffU)

/* Total number of configured Data blocks */
#define E_EEPROM_XMC1_MAX_BLOCK_COUNT      (2U)

/*
 *  Total number of pages per bank, resulting after division of banks
 *  i.e. E_EEPROM_XMC1_BANK_PAGES = (E_EEPROM_XMC1_FLASH_TOTAL_SIZE in Bytes / ((256 Bytes * 2 Banks))
 */
#define E_EEPROM_XMC1_BANK_PAGES           (2U)

/*
 *  Block Names generated as per user configured in GUI
 */

/**  Block 1 */
#define USER_BLOCK1  (1U)
#define USER_BLOCK1_SIZE					128

/**  Block 2 */
#define USER_BLOCK2  (2U)
#define USER_BLOCK2_SIZE					128

#define DEFAULT_DATA_OFFSET_0				0
#define DEFAULT_DATA_OFFSET_1				1


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/


FLASH_ERROR_CODE_Type FLASH_WriteSpecificPage(FLASH_PAGE_Type page_type, INT8U *pBuffer, INT16U size);
FLASH_ERROR_CODE_Type FLASH_ReadSpecificPage(FLASH_PAGE_Type page_type, INT8U *buffer);
FLASH_ERROR_CODE_Type Flash_EraseMemory(FLASH_PAGE_Type page_type);


/*
*********************************************************************************************************
*                                               END
*********************************************************************************************************
*/

#endif /* USER_DRIVER_FLASH_DRIVER_FLASH_H_ */
