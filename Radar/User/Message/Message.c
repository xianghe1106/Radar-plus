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

//MSG_Cache_Type 		AppMsgCache[MSG_SIZE];
//SCHQ_Double_Type 	AppMsg;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

#if 0

void APP_MessageInit(void)
{
//	AppMsg = SCHQ_DoubleCreate(&AppMsgCache[0], MSG_SIZE);
}

void APP_MessagePost(MSG_Cache_Type msg)
{
	SCHQ_DoublePost(&AppMsg, msg);
}

void APP_MessageProcess(void)
{
	MSG_Cache_Type msg;
//	CPU_SR_ALLOC();
	
//	INT8U  TxData[15];
//	SCH_RTC_Type rtc;
	
	msg = SCHQ_DoublePend(&AppMsg);

	switch(msg.Type)
	{
		case MsgType_NULL:	
		break;

		case MsgType_WDG:
			BSP_FeedDog();	//watch dog SCH			
		break;
		
		case MsgType_BandChange:
			SetLOFrequency();	//configure PLL		
		break;
		
		case MsgType_Reset:	
			Bsp_SystemReset();
		break;
		
		case MsgType_UpdateAllTable:	
//			CPU_CRITICAL_ENTER();
			LoadCalibrationTable();
//			CPU_CRITICAL_EXIT();
		break;

		case MsgType_PowerControl:
			BUC_PowerControl();
			break;
		
#if NJ_PROJECT_ENB == TRUE
		case MsgType_ResetUdp:
			resetUdpPort();
			break;
#endif
		
		case MsgType_LogBackup:
			LOG_Backup();
			break;

		default:	//No msgs
		break;
	}
}


#endif


