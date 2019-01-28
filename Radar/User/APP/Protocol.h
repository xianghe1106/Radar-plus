/*
 * Protocol.h
 *
 *  Created on: Dec 11, 2018
 *      Author: xianghe
 *
 *      V1.0.3		2019-01-07		XH				- fix the toilet cover A detection.
 *      											- fix the distance detection, add buffer to filter the abnormal distance data
 *      											caused by abnormal signal.
 *      											- toilet cover B will closed while distance beyond 1.5m and the time beyond 10s,
 *      											and then toilet cover A will closed.
 *      											- fix DistanceLinear().
 *
 *      V1.0.4		2019-01-14		XH				- add distance offset.
 *      												1. protocol.c
 *      												  a. SetDistanceOffset updated.
 *      												  b. GetDistanceOffset updated.
 *      												2. radar.c
 *														  a. RADAR_GetDistance updated.
 *														3. pwm.c
 *														  a. add pwm.c and pwm.h files.
 *														4. RTAPP
 *														  a. add RTAPP.c and RTAPP.h files.
 *														  b. SCH_Core.c and SCH_Core.h updated, add p_APP_Task_Update.
 *
 *      V1.0.5		2019-01-17		XH
 *      											- ARM-GCC optimization level changed from Os to O1。
 *													- GetFirmwareVersion, FW version info changed from 2 bytes to 3 bytes.
 *													- add Radar_DistanceUpdate function to calculate a new distance parameter,
 *													calculate the new distance parameter by formula.
 *
 *      V1.0.6		2019-01-19		XH
 *      											- Radar_PWMStateUpdate updated.
 *      											- Radar_MotionUpdate updated.
 *      											- Radar_PWMStateUpdate updated.
 *      											- RADAR_GetAmplitude updated.
 *      											- Radar_ToiletCoverUpdate updated.
 *      											- Protocol_heart_beat updated.
 *      											- protocol: add command 0x85,0x87,0x88,0xA0,0xA8,0xA9.
 *      											- add Message to this project.
 *      											- UART IRQ priority changed from 3 to 2.
 *      											- Protocol_heart_beat called by radarsense2gol_result, not SCH_TASK.
 *      											- protocol: add command 0x89, 0xAA.
 *      											- driver_flash.c updated.
 *      											- ARM-GCC optimization level changed from Os to O1, and do not select Function Section(-fdata-sections)
 *													- Driver, add timer to generate PWM (GLOBAL_CC8 PWM_CC8), pwm.c and pwm.h updated.
 *
 *      V1.0.7		2019-01-27		XH
 *      											- Protocol.c and Protocol.h updated,
 *      											a. using customer protocol, not the protocol defined by ourself.
 *      											b. add some internal commands.
 *      											- Radar.c and Radar.h updated.
 *      											- driver_uart.c updated.
 *      											- Message.c updated to support baud rate changes.
 *      											- Bsp.c updated to support baud rate changes.
 *
 *      V1.0.8		2019-01-28		XH
 *      											- GetSummaryInfoEx updated.
 *      											- RADAR_GetSignalStrength updated.
 *
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

typedef enum
{
	PACKET_STATUS_START_BYTE_1 	= 0,
	PACKET_STATUS_START_BYTE_2 	= 1,
	PACKET_STATUS_DEVICE_ADD 	= 2,
	PACKET_STATUS_LENGTH 		= 3,
	PACKET_STATUS_COMMAND 		= 4,
	PACKET_STATUS_PARA_DATA 	= 5,
	PACKET_STATUS_CHECK_SUM 	= 6
}PACKET_STATUS_Type;

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
#define BUILD_VERSION							8

#define START_BYTE_1							0x41
#define START_BYTE_2							0x52



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
