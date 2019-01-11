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

}MsgValue;

typedef enum
{
	Flag_Null 					= 0x00,
	Flag_GestureDelay 			= 0x01,
	Flag_CoverADelay			= 0x02,
	Flag_CoverBDelay			= 0x03,
	Flag_SampleFilter			= 0x04,

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


