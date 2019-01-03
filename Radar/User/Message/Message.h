#ifndef _MESSAGE_H
#define _MESSAGE_H  

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "cpu.h"
//#include "stm32f10x.h"

#include "SCH_Core.h"

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



/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

//extern SCHQ_Double_Type AppMsg;

/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/

#define MSG_SIZE								50

typedef enum
{
	MsgType_NULL				= 0,					/*have no message*/
	MsgType_BandChange			= 1,
	MsgType_Reset				= 2,
	MsgType_LogBackup			= 3,
//	MsgType_StaticResponse		= 4,
	MsgType_PowerControl		= 5,
	MsgType_ResetUdp			= 6,
	

	MsgType_WDG					= 10,	//send by SCH	SCH_MsgType_WDG = MsgType_WDG
	MsgType_Task				= 11,
	MsgType_Heartbeat			= 12,
	
	MsgType_UpdateAllTable		= 20,
}MsgType;

typedef enum
{
	MsgValue_NULL				= 0x00,					/*have no message*/
	MsgValue_Online				,
	MsgValue_Offline			,
	MsgValue_CriticalAlarmOn	,
	MsgValue_CriticalAlarmOff	,
	MsgValue_SSPAON				,
	MsgValue_SSPAOFF			,

}MsgValue;

typedef enum
{
	Flag_Null 					= 0x00,
	Flag_UnmuteDelay 			= 0x01,
	Flag_CriticalAlarmDelay 	= 0x02,
	Flag_ProtocolProcessDelay 	= 0x03,
//	Flag_FaultDelay 			= 0x04,
//	Flag_WorkModeProcessDelay 	= 0x05,
//	Flag_ResetFaultDelay 		= 0x06,
//	Flag_PowerOffDelay			= 0x07,	//True: BUC power off, False: recover SSPA state
//	Flag_TaskTimeManagerDelay 	= 0x08,
//	Flag_GeneralDelay 			= 0x09,
//	Flag_PowerLossDelay			= 0x0A,
//	Flag_StartupDelay			= 0x0B,
//	Flag_UnmuteDelay			= 0x0C,
//	Flag_OnlineDelay			= 0x0B,
//	Flag_SwitchOnlineDelay		= 0x0C,
	
	Flag_PLLConfigDelay 		= 0x20,	//system
}SCH_FLAG_FLAG;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
#if 0
void APP_MessageInit(void);

void APP_MessagePost(MSG_Cache_Type msg);

void APP_MessageProcess(void);

#endif
#endif


