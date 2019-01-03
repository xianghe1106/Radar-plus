/*
*********************************************************************************************************
*                                                SCH_Flag
*
* File    : SCH_Flag.c
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

SCH_Flag_Type SCH_Flag_G[SCH_MAX_FLAG];

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


void SCH_Flag_Init(void)
{
	INT8U Index; 

	for (Index = 0; Index < SCH_MAX_FLAG; Index++)
	{
		SCH_Flag_G[Index].Type   = 0x00;
		SCH_Flag_G[Index].Period = 0x00;
	}
}

/*------------------------------------------------------------------*-

  SCH_Add_Flag()

  Causes a "delay" to be executed at regular intervals 
  or after a user-defined delay

  Type   - The name of the "delay" which is to be scheduled.
           NOTE: All scheduled functions must be 'void, void' -
           that is, they must take no parameters, and have 
           a void return type. 

  PERIOD - the "delay" is called repeatedly at an interval
           determined by the value of PERIOD .


  RETURN VALUE:  

  Returns the position in the "delay" array at which the task has been 
  added.  If the return value is SCH_MAX_DLYS then the "delay" could 
  not be added to the array (there was insufficient space).  If the
  return value is < SCH_MAX_DLYS, then the "delay" was added 
  successfully.  

  EXAMPLES:

-*------------------------------------------------------------------*/
INT8U SCH_Add_Flag(const INT8U TYPE,const INT16U PERIOD)
{
	INT8U Index = 0;
	INT8U errorCode = 0;

//	CPU_SR_ALLOC();

//	SCH_CRITICAL_ENTER();
	
	// First find a gap in the array (if there is one)
	while ((SCH_Flag_G[Index].Type != 0) && (Index < SCH_MAX_FLAG))
	{
		if(SCH_Flag_G[Index].Type == TYPE)			
		{
			if(SCH_Flag_G[Index].Period < PERIOD)
			{
				SCH_Flag_G[Index].Period = PERIOD;
			}

			errorCode = SCH_FLAG_REGISTERED;
			break; 
		}

		Index++;
	} 
	
	// Have we reached the end of the list?   
	if (Index == SCH_MAX_FLAG)
	{
		// Return an error code
		errorCode = SCH_FLAG_FULL; 
	}

	if(errorCode)
	{
//		SCH_CRITICAL_EXIT();
		return errorCode;
	}
	
	// If we're here, there is a space in the Delay array
	SCH_Flag_G[Index].Type   = TYPE;
	
	SCH_Flag_G[Index].Period = PERIOD;
	if(SCH_Flag_G[Index].Period > 1)
	{
		SCH_Flag_G[Index].Period = SCH_Flag_G[Index].Period - 1;
	}

//	SCH_CRITICAL_EXIT();
	
	return Index; // return position of task (to allow later deletion)
}

/*------------------------------------------------------------------*-

  SCH_FLG_Update()

  This is the scheduler ISR.  

-*------------------------------------------------------------------*/
void SCH_Flag_Update(void)
{
	INT8U Index;

	for (Index = 0; Index < SCH_MAX_FLAG; Index++)
	{
		if (SCH_Flag_G[Index].Type)
		{
			if (SCH_Flag_G[Index].Period)
			{
				SCH_Flag_G[Index].Period--;

				if (SCH_Flag_G[Index].Period == 0)
				{
					SCH_Flag_G[Index].Type   = 0x00;	//delete current "flag"
					SCH_Flag_G[Index].Period = 0x00;
				}
			}
		}         
	}  
}

/*------------------------------------------------------------------*-

  INT8U SCH_Get_Flag(const INT8U Type)

  Type   - The name of the "flag" which is to be delayed.

  RETURN VALUE: 
	  If return value is 0,it means that the parameter has been completed delay,
	  or the "flag" is not registered
  
-*------------------------------------------------------------------*/
INT16U SCH_Get_Flag(const INT8U Type)
{
	INT8U Index;

	for (Index = 0; Index < SCH_MAX_FLAG; Index++)
	{
		if (SCH_Flag_G[Index].Type == Type)
		{
			return SCH_Flag_G[Index].Period;
		}         
	} 

	return 0;
}














