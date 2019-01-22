#ifndef _SCH_QUEUE_H
#define _SCH_QUEUE_H

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "cpu.h"
#include "SCH_cfg.h"

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

/* Determine the size of SCH_QUEUES (8, 16 or 32 bits) */
#if SCH_Q_NBITS == 8
typedef  INT8U    SCH_QUEUES;
#endif

#if SCH_Q_NBITS == 16
typedef  INT16U   SCH_QUEUES;
#endif

#if SCH_Q_NBITS == 32
typedef  INT32U   SCH_QUEUES;
#endif

typedef struct
{
	INT8U *SCHQStart;
	INT8U *SCHQEnd;
	
	INT8U *SCHQIn;
	INT8U *SCHQOut;
	
	INT16U SCHQSize;
	INT16U SCHQEntries;
}SCHQ_Single_Type;

typedef struct 
{
	SCH_QUEUES Type;
	SCH_QUEUES Value;
}MSG_Cache_Type;


typedef struct
{
	MSG_Cache_Type *SCHQStart;
	MSG_Cache_Type *SCHQEnd;
	
	MSG_Cache_Type *SCHQIn;
	MSG_Cache_Type *SCHQOut;
	
	INT16U SCHQSize;
	INT16U SCHQEntries;
}SCHQ_Double_Type;

typedef enum
{
	SCHQ_NO_ERROR			= 0,					
	SCHQ_ERR_FULL			= 1,
	SCHQ_ERR_PEVENT_NULL   	= 2,
	SCHQ_ERR_POST_NULL_PTR	= 3,
	SCHQ_ERR_POINT_NULL		= 4,
	
	SCHQ_EMPTY				= 0xFFFF
}SCH_ERROR;


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

SCHQ_Single_Type SCHQ_SingleCreate(INT8U *start, INT16U size);

INT8U SCHQ_SinglePost(SCHQ_Single_Type *qPtr, INT8U data);

INT16U SCHQ_SinglePend(SCHQ_Single_Type *qPtr);


SCHQ_Double_Type SCHQ_DoubleCreate(MSG_Cache_Type *start, INT16U size);

INT8U SCHQ_DoublePost(SCHQ_Double_Type *queuePoint, MSG_Cache_Type msg);

MSG_Cache_Type SCHQ_DoublePend(SCHQ_Double_Type *queuePoint);



#endif
