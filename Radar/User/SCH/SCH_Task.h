#ifndef _SCH_TASK_H
#define _SCH_TASK_H

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
   // Pointer to the task (must be a 'void (void)' function)
   void (*pTask)(void);  

   // Delay (ticks) until the function will (next) be run
   // - see SCH_Add_Task() for further details
   INT16U Delay;       

   // Interval (ticks) between subsequent runs.
   // - see SCH_Add_Task() for further details
   INT16U Period;       

   // Incremented (by scheduler) when task is due to execute
   INT8U RunMe;       
}SCH_Task_Type; 

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

#define SCH_MAX_TASKS   						(15)  

//extern schTask SCH_tasks_G[SCH_MAX_TASKS];

// The maximum number of tasks required at any one time
// during the execution of the program
//
// MUST BE ADJUSTED FOR EACH NEW PROJECT

#define ERROR_SCH_TOO_MANY_TASKS 				(1)

#define ERROR_SCH_CANNOT_DELETE_TASK 			(2)

#define RETURN_NORMAL 							0

#define RETURN_ERROR 							1

#if SCH_MAX_TASKS < 10
#error Add task is beyond the max tasks...
#endif

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void  SCH_Dispatch_Tasks(void);

INT8U SCH_Add_Task(void (*) (void), const INT16U, const INT16U);  

INT8U SCH_Delete_Task(const INT8U);

void  SCH_Report_Status(void);

void SCH_Task_Init(void);

void SCH_Task_Update(void);




#endif
