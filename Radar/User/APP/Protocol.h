/*
 * Protocol.h
 *
 *  Created on: Dec 11, 2018
 *      Author: xianghe
 */

#ifndef USER_APP_PROTOCOL_H_
#define USER_APP_PROTOCOL_H_



/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "cpu.h"
#include "XMC1300.h"

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

enum
{
	CMD_SUCCEED = 0, DATA_OVERFLOW=1, UNSUPPORT_PARAMETER = 2, CRC_ERROR = 3,
	SYSTEM_BUSY = 4, UNSUPPORT_CMD = 5, REV_SHORT=6, NO_CALDATA=7, FLIEPAG_ERROR=8
};

typedef struct
{
//	INT8U CommandType;		//1:set	0:get
	INT8U RxParaLen;
	INT8U *ErrorCode;
	INT8U *Para;
	INT8U *Output;
	INT8U *DynamicLen;
}ProtocolMsg_TypeDef;

struct NEURON
{
	INT16U Command;
	void (*Cell)(ProtocolMsg_TypeDef ProtocolMsg);
	INT16U DataLen;
};

typedef struct
{
	//input para offset
	INT8U start_byte_offset;
	INT8U rx_len_offset;
	INT8U device_address_offset;
	INT8U command_offset;
	INT8U para_start_offset;
	INT8U stop_byte_offset;

	//output pata offset
/*	INT8U start_byte_offset;
	INT8U rx_len_offset;
	INT8U device_address_offset;
	INT8U command_offset;
	INT8U para_start_offset;
	INT8U stop_byte_offset;*/
}ProtocolOffset_TypeDef;

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

#define MAJOR_VERSION							1	// from 0 to 9
#define MINOR_VERSION							0	// from 0 to 9

#define START_BYTE								0x7E
#define STOP_BYTE								0x7E



/*--- address ---*/
#define DEVICE_ADD_EE							0x0001
//#define DEVICE_ADD_EE							0x0002

#define DISTANCE_OFFSET_1_EE					0x0003
#define DISTANCE_OFFSET_2_EE					0x0004
//...
#define DISTANCE_OFFSET_10_EE					0x000C

#define AMPLITUDE_OFFSET_1_EE					0x0010
#define AMPLITUDE_OFFSET_10_EE					0x001A
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void Protocol_init(void);
void Protocol_preprocessing(void);
void Protocol_process(void);
void Protocol_heart_beat(void);
void Protocol_RaderSamplingData(void);


/*
*********************************************************************************************************
*                                               END
*********************************************************************************************************
*/

#endif /* USER_APP_PROTOCOL_H_ */
