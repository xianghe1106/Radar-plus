#ifndef _QUEUE_H
#define _QUEUE_H

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "cpu.h"

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

typedef struct 
{
	INT32U Size;
	INT32U Entries;
	
	INT8U  *pStart;
	INT8U  *pEnd;
	
	INT8U  *pIn;
	INT8U  *pOut;
//	INT8U  *pBuf;
}QUEUE8_Type;

typedef struct
{
	INT32U Size;
	INT32U Entries;

	INT16U  *pStart;
	INT16U  *pEnd;

	INT16U  *pIn;
	INT16U  *pOut;
//	INT8U  *pBuf;
}QUEUE16_Type;

typedef enum
{
	QUEUE_NO_ERROR			= 0,
	QUEUE_POINT_ERROR,
	
	QUEUE_FULL,
	QUEUE_EMPTY,

}QUEUE_ERROR;

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/




/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

INT8U QUEUE8_Create(QUEUE8_Type *pQ8, INT8U *pBuf, INT32U size);

INT8U QUEUE8_Push(QUEUE8_Type *pQ8, INT8U data);
INT8U QUEUE8_PushNData(QUEUE8_Type *pQ8, INT8U *pData, INT32U num);

INT8U QUEUE8_Pop(QUEUE8_Type *pQ8, INT8U *pData);
INT8U QUEUE8_PopNData(QUEUE8_Type *pQ8, INT8U *pData, INT32U num);

INT8U QUEUE8_GetSize(QUEUE8_Type *pQ8, INT32U *pSize);
INT8U QUEUE8_GetEntries(QUEUE8_Type *pQ8, INT32U *pEntries);

INT8U QUEUE8_Clear(QUEUE8_Type *pQ8);

INT8U QUEUE8_PushEx(QUEUE8_Type *pQ8, INT8U data);
INT8U QUEUE8_PushNDataEx(QUEUE8_Type *pQ8, INT8U *pData, INT32U num);


/*----------------------queue16-----------------*/
INT8U QUEUE16_Create(QUEUE16_Type *pQ16, INT16U *pBuf, INT32U size);
INT8U QUEUE16_Push(QUEUE16_Type *pQ16, INT16U data);
INT8U QUEUE16_PushEx(QUEUE16_Type *pQ16, INT16U data);
INT8U QUEUE16_PushNData(QUEUE16_Type *pQ16, INT16U *pData, INT32U num);
INT8U QUEUE16_PushNDataEx(QUEUE16_Type *pQ16, INT16U *pData, INT32U num);
INT8U QUEUE16_Pop(QUEUE16_Type *pQ16, INT16U *pData);
INT8U QUEUE16_PopNData(QUEUE16_Type *pQ16, INT16U *pData, INT32U num);
INT8U QUEUE16_GetSize(QUEUE16_Type *pQ16, INT32U *pSize);
INT8U QUEUE16_GetEntries(QUEUE16_Type *pQ16, INT32U *pEntries);
INT8U QUEUE16_Clear(QUEUE16_Type *pQ16);

/*
*********************************************************************************************************
*                                               END
*********************************************************************************************************
*/

#endif
