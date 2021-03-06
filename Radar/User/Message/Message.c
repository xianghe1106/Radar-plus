/*
*********************************************************************************************************
*                                                Message
*
* File    : Message.c
* By      : XH
* Version : V1.0
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

//#include "main.h"
//#include "app.h"
#include "Message.h"
#include "SCH_Core.h"
#include "Bsp.h"
#include "driver.h"

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

MSG_Cache_Type 		AppMsgCache[MSG_SIZE];
SCHQ_Double_Type 	AppMsg;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void APP_MessageInit(void)
{
	AppMsg = SCHQ_DoubleCreate(&AppMsgCache[0], MSG_SIZE);
}

void APP_MessagePost(MSG_Cache_Type msg)
{
	SCHQ_DoublePost(&AppMsg, msg);
}

void APP_MessageProcess(void)
{
	MSG_Cache_Type msg;
//	CPU_SR_ALLOC();
	
	msg = SCHQ_DoublePend(&AppMsg);

	switch(msg.Type)
	{
		case MsgType_NULL:	
		break;

		case MsgType_WDG:
			BSP_FeedDog();	//watch dog SCH			
		break;
		
		case MsgType_Reset:	
			Bsp_SystemReset();
		break;

		case MsgType_BaudRate:
			Driver_uart_init();
			break;

		default:	//No msgs
		break;
	}
}



