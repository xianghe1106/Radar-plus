/*
*********************************************************************************************************
*                                                SCH_Queue
*
* File    : SCH_Queue.c
* By      : XH
* Version : V1.4
*
* Comments:
* ---------------
*     
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "SCH_Core.h"

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

/*------------------------------------------------------------------*-

  SCHQ_Single_Typedef SCHQ_SingleCreat(INT8U *start, INT16U size)

  Queue creat function.  

-*------------------------------------------------------------------*/
SCHQ_Single_Type SCHQ_SingleCreate(INT8U *start, INT16U size)
{
	SCHQ_Single_Type schq;
	
    if(start == (void *)0) 
	{
		schq.SCHQIn = 0;
		schq.SCHQOut = 0;
		
		schq.SCHQSize  = 0;
		schq.SCHQEntries = 0;
		
        return schq;
    }
	
	schq.SCHQStart = start;
	schq.SCHQEnd = &start[size];
	
	schq.SCHQIn = start;
	schq.SCHQOut = start;
	
	schq.SCHQSize  = size;
	schq.SCHQEntries = 0;
	
	return schq;
}

/*------------------------------------------------------------------*-

  INT8U SCHQ_SinglePost(SCHQ_Single_Typedef *qPtr, INT8U data)

  Queue post function.  

-*------------------------------------------------------------------*/
INT8U SCHQ_SinglePost(SCHQ_Single_Type *qPtr, INT8U data)
{
//	CPU_SR_ALLOC();
	
    if(qPtr == (void *)0) 
	{

        return SCHQ_ERR_POINT_NULL;
    }
	
	if(qPtr->SCHQEntries >= qPtr->SCHQSize)
	{
		return SCHQ_ERR_FULL;
	}
	
//	SCH_CRITICAL_ENTER();
	
	*qPtr->SCHQIn = data;
	
	qPtr->SCHQEntries++;
	
	qPtr->SCHQIn++;
	
	if(qPtr->SCHQIn == qPtr->SCHQEnd)
	{
		qPtr->SCHQIn = qPtr->SCHQStart;
	}
	
//	SCH_CRITICAL_EXIT();
	
	return SCHQ_NO_ERROR;
}

/*------------------------------------------------------------------*-

  INT16U SCHQ_SinglePend(SCHQ_Single_Typedef *qPtr)

  Queue pend function.  

-*------------------------------------------------------------------*/
INT16U SCHQ_SinglePend(SCHQ_Single_Type *qPtr)
{
	INT8U data;
//	CPU_SR_ALLOC();
	
    if(qPtr == (void *)0) 
	{
        return SCHQ_ERR_POINT_NULL;
    }

	if(qPtr->SCHQEntries == 0)
	{
		return SCHQ_EMPTY;
	}

//	SCH_CRITICAL_ENTER();
	
	data = *qPtr->SCHQOut;
	
	qPtr->SCHQEntries--;
	
	qPtr->SCHQOut++;
	
	if(qPtr->SCHQOut == qPtr->SCHQEnd)
	{
		qPtr->SCHQOut = qPtr->SCHQStart;
	}

//	SCH_CRITICAL_EXIT();
	
	return data;
}


/*------------------------------------------------------------------*-

  SCHQ_Double_Typedef SCHQ_DoubleCreat(MSG_Cache_Typedef *start, INT16U size)

  Queue creat function.  

-*------------------------------------------------------------------*/
SCHQ_Double_Type SCHQ_DoubleCreate(MSG_Cache_Type *start, INT16U size)
{
	SCHQ_Double_Type schq;
	
    if(start == (void *)0) 
	{
		schq.SCHQIn = 0;
		schq.SCHQOut = 0;
		
		schq.SCHQSize  = 0;
		schq.SCHQEntries = 0;
		
        return schq;
    }
	
	schq.SCHQStart = start;
	schq.SCHQEnd = &start[size];
	
	schq.SCHQIn = start;
	schq.SCHQOut = start;
	
	schq.SCHQSize  = size;
	schq.SCHQEntries = 0;
	
	return schq;
}

/*------------------------------------------------------------------*-

  INT8U SCHQ_DoublePost(SCHQ_Double_Typedef *qPtr, MSG_Cache_Typedef msg)

  Queue post function.  

-*------------------------------------------------------------------*/
INT8U SCHQ_DoublePost(SCHQ_Double_Type *qPtr, MSG_Cache_Type msg)
{
//	CPU_SR_ALLOC();
	
    if(qPtr == (void *)0) 
	{
        return SCHQ_ERR_POINT_NULL;
    }
	
	if(qPtr->SCHQEntries >= qPtr->SCHQSize)
	{
		return SCHQ_ERR_FULL;
	}
	
//	SCH_CRITICAL_ENTER();
	
	*qPtr->SCHQIn = msg;
	
	qPtr->SCHQEntries++;
	
	qPtr->SCHQIn++;
	
	if(qPtr->SCHQIn == qPtr->SCHQEnd)
	{
		qPtr->SCHQIn = qPtr->SCHQStart;
	}
	
//	SCH_CRITICAL_EXIT();
	
	return SCHQ_NO_ERROR;
}

/*------------------------------------------------------------------*-

  MSG_Cache_Typedef SCHQ_DoublePend(SCHQ_Double_Typedef *qPtr)

  Queue pend function.  

-*------------------------------------------------------------------*/
MSG_Cache_Type SCHQ_DoublePend(SCHQ_Double_Type *qPtr)
{
	MSG_Cache_Type msg;
//	CPU_SR_ALLOC();
	
    if(qPtr == (void *)0) 
	{
		msg.Type  = 0;
		msg.Value = 0;
		
		return msg;
    }

	if(qPtr->SCHQEntries == 0)
	{
		msg.Type  = 0;
		msg.Value = 0;
		
		return msg;
	}

//	SCH_CRITICAL_ENTER();
	
	msg = *qPtr->SCHQOut;
	
	qPtr->SCHQEntries--;
	
	qPtr->SCHQOut++;
	
	if(qPtr->SCHQOut == qPtr->SCHQEnd)
	{
		qPtr->SCHQOut = qPtr->SCHQStart;
	}

//	SCH_CRITICAL_EXIT();
	
	return msg;
}





