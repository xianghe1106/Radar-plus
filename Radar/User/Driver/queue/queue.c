/*
*********************************************************************************************************
*                                                queue
*
* File    : queue.c
* By      : XH
* Version : V1.1
*
* Comments:
* ---------------
* V1.1		2018-12-19		xiang he			- Added:
*													QUEUE8_PushEx, QUEUE8_PushNDataEx
*													INT8U QUEUE16_Create(QUEUE16_Type *pQ16, INT16U *pBuf, INT32U size);
													INT8U QUEUE16_Push(QUEUE16_Type *pQ16, INT16U data);
													INT8U QUEUE16_PushEx(QUEUE16_Type *pQ16, INT16U data);
													INT8U QUEUE16_PushNData(QUEUE16_Type *pQ16, INT16U *pData, INT32U num);
													INT8U QUEUE16_PushNDataEx(QUEUE16_Type *pQ16, INT16U *pData, INT32U num);
													INT8U QUEUE16_Pop(QUEUE16_Type *pQ16, INT16U *pData);
													INT8U QUEUE16_PopNData(QUEUE16_Type *pQ16, INT16U *pData, INT32U num);
													INT8U QUEUE16_GetSize(QUEUE16_Type *pQ16, INT32U *pSize);
													INT8U QUEUE16_GetEntries(QUEUE16_Type *pQ16, INT32U *pEntries);
													INT8U QUEUE16_Clear(QUEUE16_Type *pQ16);
*********************************************************************************************************
*/ 



/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "queue.h"

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



/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/




//QUEUE8_Type *QUEUE8_Create(INT8U *pBuf, INT32U size)
INT8U QUEUE8_Create(QUEUE8_Type *pQ8, INT8U *pBuf, INT32U size)
{
	if((pQ8 == (void *)0)
	|| (pBuf == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	pQ8->Size    = size;
	pQ8->Entries	= 0;
	
	pQ8->pIn		= pBuf;
	pQ8->pOut	= pBuf;
	
	pQ8->pStart  = pBuf;
	pQ8->pEnd    = &pBuf[size];
 
	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_Push(QUEUE8_Type *pQ8, INT8U data)
{
	if(pQ8 == (void *)0)
	{
		return QUEUE_POINT_ERROR;
	}
	
	if(pQ8->Entries >= pQ8->Size)
	{
		return QUEUE_FULL;
	}
	
	*pQ8->pIn = data;
	
	pQ8->Entries++;	
	
	pQ8->pIn++;
	if(pQ8->pIn == pQ8->pEnd)
	{
		pQ8->pIn = pQ8->pStart;
	}
	
	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_PushEx(QUEUE8_Type *pQ8, INT8U data)
{
	if(pQ8 == (void *)0)
	{
		return QUEUE_POINT_ERROR;
	}

//	if(pQ8->Entries >= pQ8->Size)
//	{
//		return QUEUE_FULL;
//	}

	*pQ8->pIn = data;

	pQ8->Entries++;

	pQ8->pIn++;
	if(pQ8->pIn == pQ8->pEnd)
	{
		pQ8->pIn = pQ8->pStart;
	}

	if(pQ8->pIn == pQ8->pOut)	//buffer full
	{
		pQ8->pOut++;			//delete the oldest data
		if(pQ8->pOut == pQ8->pEnd)
		{
			pQ8->pOut = pQ8->pStart;
		}

		pQ8->Entries--;
	}

	return QUEUE_NO_ERROR;
}


INT8U QUEUE8_PushNData(QUEUE8_Type *pQ8, INT8U *pData, INT32U num)
{
	INT32U count = num;
	
	if((pQ8 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}
	
	while(count--)
	{
		if(pQ8->Entries >= pQ8->Size)
		{
			return QUEUE_FULL;
		}
		
		*pQ8->pIn = *pData;
		
		pData++;
		pQ8->Entries++;	
		
		pQ8->pIn++;
		if(pQ8->pIn == pQ8->pEnd)
		{
			pQ8->pIn = pQ8->pStart;
		}
	}
	
	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_PushNDataEx(QUEUE8_Type *pQ8, INT8U *pData, INT32U num)
{
	INT32U count = num;

	if((pQ8 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	while(count--)
	{
//		if(pQ8->Entries >= pQ8->Size)
//		{
//			return QUEUE_FULL;
//		}

		*pQ8->pIn = *pData;

		pData++;
		pQ8->Entries++;

		pQ8->pIn++;
		if(pQ8->pIn == pQ8->pEnd)
		{
			pQ8->pIn = pQ8->pStart;
		}

		if(pQ8->pIn == pQ8->pOut)	//buffer full
		{
			pQ8->pOut++;			//delete the oldest data
			if(pQ8->pOut == pQ8->pEnd)
			{
				pQ8->pOut = pQ8->pStart;
			}

			pQ8->Entries--;
		}
	}

	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_Pop(QUEUE8_Type *pQ8, INT8U *pData)
{
	if((pQ8 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}
	
	if(pQ8->Entries == 0)
	{
		return QUEUE_EMPTY;
	}
	
	*pData = *pQ8->pOut;
	
	pQ8->Entries--;		
	pQ8->pOut++;
	if(pQ8->pOut == pQ8->pEnd)
	{
		pQ8->pOut = pQ8->pStart;
	}


	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_PopNData(QUEUE8_Type *pQ8, INT8U *pData, INT32U num)
{
	INT32U count = num;
	
	if((pQ8 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}
	
	while(count--)
	{
		if(pQ8->Entries == 0)
		{
			return QUEUE_EMPTY;
		}
		
		*pData = *pQ8->pOut;
		
		pData++;
		pQ8->Entries--;		
		pQ8->pOut++;
		if(pQ8->pOut == pQ8->pEnd)
		{
			pQ8->pOut = pQ8->pStart;
		}
	}

	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_GetSize(QUEUE8_Type *pQ8, INT32U *pSize)
{
	if((pQ8 == (void *)0)
	|| (pSize == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	*pSize = pQ8->Size;
	
	return QUEUE_NO_ERROR;
}

/*
*********************************************************************************************************
*				INT8U QUEUE8_GetEntries(QUEUE8_Type *pQ8, INT32U *pEntries)
*
* Description: This function is used to get the message entries in the created queue.
*
* Arguments  : pQ8    is the queue point which has been created.
*
*
* Returns    : Error code.
*
* Note(s)     : (1) .
*
*               (2) .
*********************************************************************************************************
*/
INT8U QUEUE8_GetEntries(QUEUE8_Type *pQ8, INT32U *pEntries)
{
	if((pQ8 == (void *)0)
	|| (pEntries == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}
	
	*pEntries = pQ8->Entries;
	
	return QUEUE_NO_ERROR;
}

/*
*********************************************************************************************************
*				INT8U QUEUE8_Clear(QUEUE8_Type *pQ8)
*
* Description: This function is used to clear the queue which has been created.
*
* Arguments  : pQ8    is the queue point which has been created.
*
*
* Returns    : Error code.
*
* Note(s)     : (1) .
*
*               (2) .
*********************************************************************************************************
*/
INT8U QUEUE8_Clear(QUEUE8_Type *pQ8)
{
	if(pQ8 == (void *)0)
	{
		return QUEUE_POINT_ERROR;
	}
	
	pQ8->Entries = 0;
	
	pQ8->pIn	 = pQ8->pStart;
	pQ8->pOut	 = pQ8->pStart;
	
	return QUEUE_NO_ERROR;
}


/*---------------------queue16---------------------------*/


INT8U QUEUE16_Create(QUEUE16_Type *pQ16, INT16U *pBuf, INT32U size)
{
	if((pQ16 == (void *)0)
	|| (pBuf == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	pQ16->Size   	= size;
	pQ16->Entries	= 0;

	pQ16->pIn		= pBuf;
	pQ16->pOut		= pBuf;

	pQ16->pStart  	= pBuf;
	pQ16->pEnd    	= &pBuf[size];

	return QUEUE_NO_ERROR;
}

INT8U QUEUE16_Push(QUEUE16_Type *pQ16, INT16U data)
{
	if(pQ16 == (void *)0)
	{
		return QUEUE_POINT_ERROR;
	}

	if(pQ16->Entries >= pQ16->Size)
	{
		return QUEUE_FULL;
	}

	*pQ16->pIn = data;

	pQ16->Entries++;

	pQ16->pIn++;
	if(pQ16->pIn == pQ16->pEnd)
	{
		pQ16->pIn = pQ16->pStart;
	}

	return QUEUE_NO_ERROR;
}

INT8U QUEUE16_PushEx(QUEUE16_Type *pQ16, INT16U data)
{
	if(pQ16 == (void *)0)
	{
		return QUEUE_POINT_ERROR;
	}

//	if(pQ8->Entries >= pQ8->Size)
//	{
//		return QUEUE_FULL;
//	}

	*pQ16->pIn = data;

	pQ16->Entries++;

	pQ16->pIn++;
	if(pQ16->pIn == pQ16->pEnd)
	{
		pQ16->pIn = pQ16->pStart;
	}

	if(pQ16->pIn == pQ16->pOut)	//buffer full
	{
		pQ16->pOut++;			//delete the oldest data
		if(pQ16->pOut == pQ16->pEnd)
		{
			pQ16->pOut = pQ16->pStart;
		}

		pQ16->Entries--;
	}

	return QUEUE_NO_ERROR;
}


INT8U QUEUE16_PushNData(QUEUE16_Type *pQ16, INT16U *pData, INT32U num)
{
	INT32U count = num;

	if((pQ16 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	while(count--)
	{
		if(pQ16->Entries >= pQ16->Size)
		{
			return QUEUE_FULL;
		}

		*pQ16->pIn = *pData;

		pData++;
		pQ16->Entries++;

		pQ16->pIn++;
		if(pQ16->pIn == pQ16->pEnd)
		{
			pQ16->pIn = pQ16->pStart;
		}
	}

	return QUEUE_NO_ERROR;
}

INT8U QUEUE16_PushNDataEx(QUEUE16_Type *pQ16, INT16U *pData, INT32U num)
{
	INT32U count = num;

	if((pQ16 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	while(count--)
	{
//		if(pQ8->Entries >= pQ8->Size)
//		{
//			return QUEUE_FULL;
//		}

		*pQ16->pIn = *pData;

		pData++;
		pQ16->Entries++;

		pQ16->pIn++;
		if(pQ16->pIn == pQ16->pEnd)
		{
			pQ16->pIn = pQ16->pStart;
		}

		if(pQ16->pIn == pQ16->pOut)	//buffer full
		{
			pQ16->pOut++;			//delete the oldest data
			if(pQ16->pOut == pQ16->pEnd)
			{
				pQ16->pOut = pQ16->pStart;
			}

			pQ16->Entries--;
		}
	}

	return QUEUE_NO_ERROR;
}

INT8U QUEUE16_Pop(QUEUE16_Type *pQ16, INT16U *pData)
{
	if((pQ16 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	if(pQ16->Entries == 0)
	{
		return QUEUE_EMPTY;
	}

	*pData = *pQ16->pOut;

	pQ16->Entries--;
	pQ16->pOut++;
	if(pQ16->pOut == pQ16->pEnd)
	{
		pQ16->pOut = pQ16->pStart;
	}


	return QUEUE_NO_ERROR;
}

INT8U QUEUE16_PopNData(QUEUE16_Type *pQ16, INT16U *pData, INT32U num)
{
	INT32U count = num;

	if((pQ16 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	while(count--)
	{
		if(pQ16->Entries == 0)
		{
			return QUEUE_EMPTY;
		}

		*pData = *pQ16->pOut;

		pData++;
		pQ16->Entries--;
		pQ16->pOut++;
		if(pQ16->pOut == pQ16->pEnd)
		{
			pQ16->pOut = pQ16->pStart;
		}
	}

	return QUEUE_NO_ERROR;
}

INT8U QUEUE16_GetSize(QUEUE16_Type *pQ16, INT32U *pSize)
{
	if((pQ16 == (void *)0)
	|| (pSize == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	*pSize = pQ16->Size;

	return QUEUE_NO_ERROR;
}

/*
*********************************************************************************************************
*				INT8U QUEUE8_GetEntries(QUEUE8_Type *pQ8, INT32U *pEntries)
*
* Description: This function is used to get the message entries in the created queue.
*
* Arguments  : pQ8    is the queue point which has been created.
*
*
* Returns    : Error code.
*
* Note(s)     : (1) .
*
*               (2) .
*********************************************************************************************************
*/
INT8U QUEUE16_GetEntries(QUEUE16_Type *pQ16, INT32U *pEntries)
{
	if((pQ16 == (void *)0)
	|| (pEntries == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	*pEntries = pQ16->Entries;

	return QUEUE_NO_ERROR;
}

/*
*********************************************************************************************************
*				INT8U QUEUE8_Clear(QUEUE8_Type *pQ8)
*
* Description: This function is used to clear the queue which has been created.
*
* Arguments  : pQ8    is the queue point which has been created.
*
*
* Returns    : Error code.
*
* Note(s)     : (1) .
*
*               (2) .
*********************************************************************************************************
*/
INT8U QUEUE16_Clear(QUEUE16_Type *pQ16)
{
	if(pQ16 == (void *)0)
	{
		return QUEUE_POINT_ERROR;
	}

	pQ16->Entries = 0;

	pQ16->pIn	 = pQ16->pStart;
	pQ16->pOut	 = pQ16->pStart;

	return QUEUE_NO_ERROR;
}



