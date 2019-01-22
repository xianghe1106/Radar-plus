#ifndef _SCH_CORE_H
#define _SCH_CORE_H 

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "XMC1300.h"
#include "cpu.h"
//#include "bsp.h"
#include "stddef.h"

#include "SCH_Task.h"
#include "SCH_Flag.h"
#include "SCH_RTC.h"
#include "SCH_Queue.h"

#include "RTAPP.h"


//#include "RTAPP.h"

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

void (*p_APP_Task_Update)(void);

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

#define SCH_CRITICAL_ENTER()			CPU_CRITICAL_ENTER() 
#define SCH_CRITICAL_EXIT()				CPU_CRITICAL_EXIT()

#define SCH_CFG_TICK_RATE_HZ       		100u               // Tick rate in Hertz (10 to 1000 Hz) 
#define SCH_TICK_TIME_MS				(1000 / SCH_CFG_TICK_RATE_HZ)


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void SCH_Init(void);

void SCH_Start(void);

void SysTick_Update(void);


#endif 

