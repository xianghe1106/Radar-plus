/*
 * driver_flash.c
 *
 *  Created on: Dec 21, 2018
 *      Author: xianghe
 */




/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "driver_flash.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

INT8U  flash_buffer[FLASH_BUFFER_SIZE];

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

FLASH_ERROR_CODE_Type FLASH_ConfirmFlashPageType(FLASH_PAGE_Type page_type);
FLASH_ERROR_CODE_Type FLASH_ConfirmFlashPage(FLASH_PAGE_Type page_type, INT32U *page_start_address);
FLASH_ERROR_CODE_Type Flash_EraseMemory(FLASH_PAGE_Type page_type);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


FLASH_ERROR_CODE_Type FLASH_WriteSpecificPage(FLASH_PAGE_Type page_type, INT8U *pBuffer, INT16U size)
{
	FLASH_ERROR_CODE_Type error_code = FLASH_NO_ERROR;
//	INT16U block_size;
	INT32U start_address;

	if(FLASH_ConfirmFlashPageType(page_type) != FLASH_NO_ERROR)
	{
		error_code = FLASH_PAGE_UNDEFINED;
		return error_code;
	}

	if(FLASH_ConfirmFlashPage(page_type, &start_address) != FLASH_NO_ERROR)
	{
		error_code = FLASH_PAGE_UNDEFINED;
		return error_code;
	}

//	MEM_Clr(flash_buffer, sizeof(flash_buffer));
	MEM_Set(flash_buffer, 0xFF, sizeof(flash_buffer));
	MEM_Copy(flash_buffer, pBuffer, size);

	XMC_FLASH_ProgramVerifyPage((INT32U *)start_address, (INT32U *)flash_buffer);

	return error_code;
}

FLASH_ERROR_CODE_Type FLASH_ReadSpecificPage(FLASH_PAGE_Type page_type, INT8U *buffer)
{
	FLASH_ERROR_CODE_Type error_code = FLASH_NO_ERROR;
	INT16U block_size;
	INT32U start_address;

	if(FLASH_ConfirmFlashPageType(page_type) != FLASH_NO_ERROR)
	{
		error_code = FLASH_PAGE_UNDEFINED;
		return error_code;
	}

	if(FLASH_ConfirmFlashPage(page_type, &start_address) != FLASH_NO_ERROR)
	{
		error_code = FLASH_PAGE_UNDEFINED;
		return error_code;
	}

	block_size = XMC_FLASH_BYTES_PER_PAGE / XMC_FLASH_BYTES_PER_BLOCK;

	XMC_FLASH_ReadBlocks((INT32U *)start_address, (INT32U *)buffer, block_size);

	return error_code;
}

FLASH_ERROR_CODE_Type Flash_EraseMemory(FLASH_PAGE_Type page_type)
{
	FLASH_ERROR_CODE_Type error_code = FLASH_NO_ERROR;
	INT32U start_address;

	if(FLASH_ConfirmFlashPageType(page_type) != FLASH_NO_ERROR)
	{
		error_code = FLASH_PAGE_UNDEFINED;
		return error_code;
	}

	if(FLASH_ConfirmFlashPage(page_type, &start_address) != FLASH_NO_ERROR)
	{
		error_code = FLASH_PAGE_UNDEFINED;
		return error_code;
	}

	XMC_FLASH_ErasePage((INT32U *)start_address);

	return error_code;
}

FLASH_ERROR_CODE_Type FLASH_ConfirmFlashPageType(FLASH_PAGE_Type page_type)
{
	FLASH_ERROR_CODE_Type error_code = FLASH_NO_ERROR;

	if((page_type == flash_page_factory)
	|| (page_type == flash_page_user))
	{
		error_code = FLASH_NO_ERROR;
	}
	else
	{
		error_code = FLASH_PAGE_UNDEFINED;
	}

	return error_code;
}

FLASH_ERROR_CODE_Type FLASH_ConfirmFlashPage(FLASH_PAGE_Type page_type, INT32U *page_start_address)
{
	FLASH_ERROR_CODE_Type error_code = FLASH_NO_ERROR;
	INT32U start_address=0;

	if((page_type == flash_page_factory)
	|| (page_type == flash_page_user))
	{
		error_code = FLASH_NO_ERROR;
	}
	else
	{
		error_code = FLASH_PAGE_UNDEFINED;
	}

	switch(page_type)
	{
		case flash_page_factory:
			start_address = E_EEPROM_XMC1_FLASH_BANK0_BASE;
			break;

		case flash_page_user:
			start_address = E_EEPROM_XMC1_FLASH_BANK1_BASE;
			break;
	}


	*page_start_address = start_address;

	return error_code;
}


