/*
 * Protocol.c
 *
 *  Created on: Dec 11, 2018
 *      Author: xianghe
 */




/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include <driver_DAVE.h>
#include "Protocol.h"
//#include "queue.h"
//#include "mem.h"

#include "xmc_uart.h"
#include "radar.h"
#include "driver.h"
#include "Message.h"

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

#define COMMAND_COUNT 						50
#define PACKET_MAX_LEN						52
#define PACKET_MAX_ENTRIES					5

/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/

/*typedef enum
{
	notify_enable = 0,
	notify_disable = 1
}NOTIFY_STATE_Type;*/

typedef enum
{
	notify_disable 	= 0,
	notify_internal = 1,
	notify_external = 2,
}NOTIFY_STATE_Type;

typedef enum
{
	internal_cmd_enable = 0,
	internal_cmd_disable = 1
}INTERNAL_COMMAND_STATE_Type;

typedef enum
{
	flag_null 		= 0,
	flag_start 		= 1,
	flag_address 	= 2,
	flag_length 	= 3,
	flag_data 		= 4,
	flag_crc 		= 5
}PACKET_FLAG_Type;

typedef struct
{
	INT8U count;
	INT8U buffer[51];
}SCAN_PACKET_Type;

typedef struct
{
	INT8U store_index;
	INT8U process_index;
	INT8U buffer[PACKET_MAX_ENTRIES][PACKET_MAX_LEN];
}PACKET_BUFFER_Type;

ProtocolOffset_TypeDef protocol_offset;



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

INT8U  rx_buffer[100];
INT8U  tx_buffer[300];


QUEUE8_Type rx_queue, *p_rx_queue = &rx_queue;

SCAN_PACKET_Type preprocess_buffer/*, process_buffer*/;
PACKET_BUFFER_Type packet_buffer;

NOTIFY_STATE_Type  notify_state = notify_external;
INTERNAL_COMMAND_STATE_Type internal_cmd_state = internal_cmd_disable;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void SetBaudRate(ProtocolMsg_TypeDef ProtocolMsg);
static void GetSummaryInfoEx(ProtocolMsg_TypeDef ProtocolMsg);


static void SetDeviceAddress(ProtocolMsg_TypeDef ProtocolMsg);
static void SetNotifyState(ProtocolMsg_TypeDef ProtocolMsg);
static void SetDistanceOffset(ProtocolMsg_TypeDef ProtocolMsg);
static void SetAmplitudeOffset(ProtocolMsg_TypeDef ProtocolMsg);
static void SetGestureTripPoint(ProtocolMsg_TypeDef ProtocolMsg);
static void SetSoftReset(ProtocolMsg_TypeDef ProtocolMsg);
static void SetDistanceTripPoint(ProtocolMsg_TypeDef ProtocolMsg);
static void SetDistanceSpeedOffsetState(ProtocolMsg_TypeDef ProtocolMsg);
static void SetPWMTripPoint(ProtocolMsg_TypeDef ProtocolMsg);
static void SetCoverOpenedTripPoint(ProtocolMsg_TypeDef ProtocolMsg);
static void SetInternalCommandState(ProtocolMsg_TypeDef ProtocolMsg);


static void GetFirmwareVersion(ProtocolMsg_TypeDef ProtocolMsg);
static void GetDeviceAddress(ProtocolMsg_TypeDef ProtocolMsg);
static void GetNotifyState(ProtocolMsg_TypeDef ProtocolMsg);
static void GetDistanceOffset(ProtocolMsg_TypeDef ProtocolMsg);
static void GetSummaryInfo(ProtocolMsg_TypeDef ProtocolMsg);
static void GetAmplitudeOffset(ProtocolMsg_TypeDef ProtocolMsg);
static void GetGestureTripPoint(ProtocolMsg_TypeDef ProtocolMsg);
static void GetDistanceTripPoint(ProtocolMsg_TypeDef ProtocolMsg);
static void GetDistanceSpeedOffsetState(ProtocolMsg_TypeDef ProtocolMsg);
static void GetPWMTripPoint(ProtocolMsg_TypeDef ProtocolMsg);
static void GetCoverOpenedTripPoint(ProtocolMsg_TypeDef ProtocolMsg);
static void GetInternalCommandState(ProtocolMsg_TypeDef ProtocolMsg);


static void LoadCalibrationData(ProtocolMsg_TypeDef ProtocolMsg);
static void SetAmplitudeCalibrationMode(ProtocolMsg_TypeDef ProtocolMsg);

static void GetCalibrationData(ProtocolMsg_TypeDef ProtocolMsg);
static void GetAmplitude(ProtocolMsg_TypeDef ProtocolMsg);
static void GetAmplitudeCalibrationMode(ProtocolMsg_TypeDef ProtocolMsg);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

const struct NEURON neuron[COMMAND_COUNT]=
{
	/*-------------------------------------------
	--
	-- EX_Protocol Set command :
	--
	-------------------------------------------*/
	{0x02 	, SetBaudRate							,1 },
	{0x03 	, SetPWMTripPoint						,2 },

	/*------------------------------------------
	--
	-- EX_Protocol Get command :
	--
	-------------------------------------------*/
	{0x01 	, GetSummaryInfoEx						,0 },
	{0x04 	, GetPWMTripPoint						,2 },


	/*---------------------------------------------------
	--
	-- Internal set command : 0x80 to 0xAF
	--
	----------------------------------------------------*/
	{0x80 	, SetDeviceAddress						,1 },
	{0x81 	, SetNotifyState						,1 },
	{0x82 	, SetDistanceOffset						,8 },
	{0x83 	, SetAmplitudeOffset					,2 },
	{0x84 	, SetGestureTripPoint					,3 },
	{0x85 	, SetSoftReset							,1 },
	{0x86 	, SetDistanceTripPoint					,2 },
	{0x87 	, SetDistanceSpeedOffsetState			,1 },
//	{0x88 	, SetPWMTripPoint						,2 },
	{0x89 	, SetCoverOpenedTripPoint				,2 },
	{0x8A 	, SetInternalCommandState				,1 },

	/*---------------------------------------------------
	--
	-- Internal get command: 0xA0 to 0xDF
	--
	----------------------------------------------------*/
	{0xA0 	, GetFirmwareVersion					,3 },
	{0xA1	, GetDeviceAddress						,1 },
	{0xA2	, GetNotifyState						,1 },
	{0xA3	, GetDistanceOffset						,8 },
	{0xA4	, GetSummaryInfo						,0 },
	{0xA5	, GetAmplitudeOffset					,2 },
	{0xA6	, GetGestureTripPoint					,3 },

	{0xA7 	, GetDistanceTripPoint					,2 },
	{0xA8 	, GetDistanceSpeedOffsetState			,1 },
	{0xA9 	, GetPWMTripPoint						,2 },
	{0xAA 	, GetCoverOpenedTripPoint				,2 },
	{0xAB 	, GetInternalCommandState				,1 },


	/*---------------------------------------------------
	--
	-- Internal calibration command: 0xD0 to 0xFE
	--
	----------------------------------------------------*/
	{0xD0 	, LoadCalibrationData					,0 },
	{0xD1 	, SetAmplitudeCalibrationMode			,1 },


	{0xE0 	, GetCalibrationData					,0 },
	{0xE1 	, GetAmplitude							,2 },
	{0xE2 	, GetAmplitudeCalibrationMode			,1 },


	{0xFFFF , NULL									,0 }
};


void Protocol_init(void)
{
	QUEUE8_Create(p_rx_queue, rx_buffer, sizeof(rx_buffer));
	MEM_Clr(&preprocess_buffer.count, 	sizeof(preprocess_buffer));
	MEM_Clr(&packet_buffer.store_index, sizeof(packet_buffer));
}


void USIC0_1_IRQHandler(void)
{
	INT8U data;

	data = XMC_UART_CH_GetReceivedData(XMC_UART0_CH0);  // receive data
	QUEUE8_Push(p_rx_queue, data);
}

void send_uart_data(INT8U *p_buffer, INT16U count)
{
	INT16U i;

	for(i = 0; i < count; i++)
	{
		XMC_UART_CH_Transmit(XMC_UART0_CH0, p_buffer[i]);
	}
}

void Protocol_preprocessing(void)
{
	INT8U  data;
	INT32U queue_entries = 0;
	static PACKET_STATUS_Type packet_state = PACKET_STATUS_START_BYTE_1;
	static INT8U  para_count = 0;

	if(QUEUE8_GetEntries(p_rx_queue, &queue_entries) != QUEUE_NO_ERROR)
	{
		return;
	}

	if(queue_entries > 0)
	{
		while(queue_entries)
		{
			queue_entries--;
			QUEUE8_Pop(p_rx_queue, &data);

			switch(packet_state)
			{
				case PACKET_STATUS_START_BYTE_1:
					if(data == START_BYTE_1)
					{
						packet_state = PACKET_STATUS_START_BYTE_2;
					}
					break;

				case PACKET_STATUS_START_BYTE_2:
					if(data == START_BYTE_2)
					{
						packet_state = PACKET_STATUS_DEVICE_ADD;
					}
					else if(data == START_BYTE_1)
					{
						packet_state = PACKET_STATUS_START_BYTE_2;
					}
					else
					{
						packet_state = PACKET_STATUS_START_BYTE_1;
					}
					break;

				case PACKET_STATUS_DEVICE_ADD:
					packet_state = PACKET_STATUS_LENGTH;

					preprocess_buffer.buffer[0] = START_BYTE_1;
					preprocess_buffer.buffer[1] = START_BYTE_2;
					preprocess_buffer.buffer[2] = data;	//add
					break;

				case PACKET_STATUS_LENGTH:
					packet_state = PACKET_STATUS_COMMAND;

//					preprocess_buffer.buffer[0] = START_BYTE_1;
//					preprocess_buffer.buffer[1] = START_BYTE_2;
//					preprocess_buffer.buffer[2] = add;	//add
					preprocess_buffer.buffer[3] = data;	//LENGTH

					if(data > 40)
					{
						packet_state = PACKET_STATUS_START_BYTE_1;
					}
					else if(data > 4)
					{
						para_count = data - 4;//not include device add, length, command and check sum
					}
					else
					{
						packet_state = PACKET_STATUS_START_BYTE_1;
					}
					break;

				case PACKET_STATUS_COMMAND:
					packet_state = PACKET_STATUS_PARA_DATA;

//					preprocess_buffer.buffer[0] = START_BYTE_1;
//					preprocess_buffer.buffer[1] = START_BYTE_2;
//					preprocess_buffer.buffer[2] = add;	//add
//					preprocess_buffer.buffer[3] = data;	//LENGTH
					preprocess_buffer.buffer[4] = data;	//COMMAND

					preprocess_buffer.count = 5;
					break;

				case PACKET_STATUS_PARA_DATA:
					preprocess_buffer.buffer[preprocess_buffer.count++] = data;
					para_count--;
					if(para_count == 0)
					{
						packet_state = PACKET_STATUS_CHECK_SUM;
					}
					break;

				case PACKET_STATUS_CHECK_SUM:
					packet_state = PACKET_STATUS_START_BYTE_1;
					preprocess_buffer.buffer[preprocess_buffer.buffer[3] + 1] = data;

//					for(i = 0; i < (preprocess_buffer.buffer[3] + 2); i++)
//					{
//						packet_buffer.buffer[packet_buffer.store_index][i] = preprocess_buffer.buffer[i];
//					}

					MEM_Copy(&packet_buffer.buffer[packet_buffer.store_index][0], preprocess_buffer.buffer, preprocess_buffer.buffer[3] + 2);

					packet_buffer.store_index = (packet_buffer.store_index + 1) % PACKET_MAX_ENTRIES;
					break;

				default:
					packet_state = PACKET_STATUS_START_BYTE_1;
					break;
			}
		}
	}
}

INT16U Calculate_CheckSum(INT8U *Packet, INT8U length)
{
	INT8U  CHECK_SUM = 0;
	INT8U  i;

	for(i = 0; i < length; i++)
	{
		CHECK_SUM = CHECK_SUM + Packet[i];
	}

    return CHECK_SUM;
}

void Protocol_heart_beat(void)
{
	INT8U  tx_buffer[30];
	ProtocolMsg_TypeDef ProtocolMsg;
	INT8U  dynamicLen = 0;

	if(notify_state != notify_internal)
	{
		return;
	}

	ProtocolMsg.Output      = &tx_buffer[5];
	ProtocolMsg.DynamicLen  = &dynamicLen;

	GetSummaryInfo(ProtocolMsg);

	tx_buffer[0] = START_BYTE_1;
	tx_buffer[1] = START_BYTE_2;
	tx_buffer[2] = radar_user_data.device_address;	//device_adress
	tx_buffer[3] = dynamicLen + 4;	//len
	tx_buffer[4] = 0x70;	//command

	tx_buffer[tx_buffer[3] + 1] = Calculate_CheckSum(&tx_buffer[2], tx_buffer[3] - 1);

	send_uart_data(tx_buffer, tx_buffer[3] + 2);
}

void Protocol_heart_beat_ex(void)
{
	INT8U  tx_buffer[30];
	ProtocolMsg_TypeDef ProtocolMsg;
	INT8U  dynamicLen = 0;

	if(notify_state != notify_external)
	{
		return;
	}

	ProtocolMsg.Output      = &tx_buffer[5];
	ProtocolMsg.DynamicLen  = &dynamicLen;

	GetSummaryInfoEx(ProtocolMsg);

	tx_buffer[0] = START_BYTE_1;
	tx_buffer[1] = START_BYTE_2;
	tx_buffer[2] = radar_user_data.device_address;	//device_adress
	tx_buffer[3] = dynamicLen + 4;	//len
	tx_buffer[4] = 0x01;	//command

	tx_buffer[tx_buffer[3] + 1] = Calculate_CheckSum(&tx_buffer[2], tx_buffer[3] - 1);

	send_uart_data(tx_buffer, tx_buffer[3] + 2);
}



void Protocol_process(void)
{
	INT8U  rxLen, errorCode=CMD_SUCCEED, dynamicLen=0, staticLen=0;
	INT16U cmdCode, index;
	INT16U crcValue;
//	INT8U  txLen = 0, paraCache[40];
	INT8U  packet_address;
	INT8U  *ptr_buffer;

	ProtocolMsg_TypeDef ProtocolMsg;

	if(packet_buffer.process_index == packet_buffer.store_index)
	{
		return;
	}

	ptr_buffer = &packet_buffer.buffer[packet_buffer.process_index][0];

	rxLen = ptr_buffer[3];

	if((ptr_buffer[0] != START_BYTE_1)
	|| (ptr_buffer[1] != START_BYTE_2))
	{
		packet_buffer.process_index = (packet_buffer.process_index + 1) % PACKET_MAX_ENTRIES;
		return;
	}

	packet_address = ptr_buffer[2];
	if(packet_address != radar_user_data.device_address)
	{
		packet_buffer.process_index = (packet_buffer.process_index + 1) % PACKET_MAX_ENTRIES;
		return;
	}

	crcValue = Calculate_CheckSum(&ptr_buffer[2], rxLen - 1);
	if(crcValue != ptr_buffer[rxLen + 1])
	{
		errorCode = CRC_ERROR;

		packet_buffer.process_index = (packet_buffer.process_index + 1) % PACKET_MAX_ENTRIES;
		return;
	}

	cmdCode = ptr_buffer[4];
	if(errorCode == CMD_SUCCEED)
	{
		//process packet
		for(index = 0; index < COMMAND_COUNT; index++)
		{
			if(0xFFFF == neuron[index].Command)
			{
				staticLen = 0;
				errorCode = UNSUPPORT_CMD;
				break;
			}

			if(cmdCode == neuron[index].Command)
			{
				errorCode				= CMD_SUCCEED;	//reset
				dynamicLen				= 0x00; 		//reset

				staticLen 			    = neuron[index].DataLen;

				//input data
	//			ProtocolMsg.CommandType = cmdCode >> 15;
				ProtocolMsg.RxParaLen  	= ptr_buffer[3] - 4;	//number of parameters
				ProtocolMsg.Para        = &ptr_buffer[5];

				//output data
				ProtocolMsg.ErrorCode   = &errorCode;
	//			ProtocolMsg.Output      = &paraCache[0];   	//paraCache
				ProtocolMsg.Output      = &tx_buffer[5];   	//paraCache
				ProtocolMsg.DynamicLen  = &dynamicLen;
				(neuron[index].Cell)(ProtocolMsg);
				break;
			}

/*			if(index == (COMMAND_COUNT - 1))
			{
				staticLen = 0;
				errorCode = UNSUPPORT_CMD;
				break;
			}*/
		}
	}

	//preparing the response packet

	tx_buffer[0] = START_BYTE_1;
	tx_buffer[1] = START_BYTE_2;
	tx_buffer[2] = radar_user_data.device_address;
	tx_buffer[3] = staticLen + dynamicLen + 4;
	tx_buffer[4] = cmdCode;

//	tx_buffer[3] = errorCode;

	crcValue = Calculate_CheckSum(&tx_buffer[2], tx_buffer[3] - 1);

	tx_buffer[tx_buffer[3] + 1] = crcValue;

	if(errorCode != UNSUPPORT_CMD)
	{
		send_uart_data(tx_buffer, tx_buffer[3] + 2);
	}

	packet_buffer.process_index = (packet_buffer.process_index + 1) % PACKET_MAX_ENTRIES;
}


/*static INT8U zxUTILS_Crc8(INT8U *ucBuf, INT8U ucLen)
{
	INT8U  crc = 0x00;

    while (ucLen--)
    {
        crc = s_aucCrc8LoopupTbl[crc ^ *ucBuf++];
    }
    return crc ^ 0x55;
}*/

INTERNAL_COMMAND_STATE_Type GetInternalCommandStatus(void)
{
	return internal_cmd_state;
}

static void SetBaudRate(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_USER_DATA_Type 	radar_user_data_bak;
	MSG_Cache_Type msg;

	MEM_Copy(&radar_user_data_bak, &radar_user_data, sizeof(radar_user_data));

	radar_user_data_bak.baud_rate_option = ProtocolMsg.Para[0];

	if((radar_user_data_bak.baud_rate_option >= 1)
	&& (radar_user_data_bak.baud_rate_option <= 10))
	{
		if(radar_user_data_bak.baud_rate_option != radar_user_data.baud_rate_option)
		{
			if(FLASH_WriteSpecificPage(flash_page_user, (INT8U *)&radar_user_data_bak, sizeof(radar_user_data_bak)) == FLASH_NO_ERROR)
			{
				radar_user_data.baud_rate_option = radar_user_data_bak.baud_rate_option;

				msg.Type = MsgType_BaudRate;
				msg.Value = MsgValue_NULL;
				APP_MessagePost(msg);
			}
			else
			{
				*ProtocolMsg.ErrorCode = SYSTEM_BUSY;
		//		return;
			}
		}
	}


	//output
	ProtocolMsg.Output[0] = radar_user_data.baud_rate_option;
}


static void GetSummaryInfoEx(ProtocolMsg_TypeDef ProtocolMsg)
{
	INT8U buffer[5];
	XMC_RADARSENSE2GOL_MOTION_t motion;

	*ProtocolMsg.DynamicLen = 10;

	RADAR_GetDistance(buffer);
	ProtocolMsg.Output[0] = buffer[0];	//distance
	ProtocolMsg.Output[1] = buffer[1];

	RADAR_GetSpeedInCM(buffer);
	RADAR_GetMotion(&motion);
	if(motion == XMC_MOTION_DETECT_APPROACHING)
	{
		buffer[0] = buffer[0] & 0x0F;
	}
	else if(motion == XMC_MOTION_DETECT_DEPARTING)
	{
		buffer[0] = buffer[0] | 0xF0;
	}
	ProtocolMsg.Output[2] = buffer[0];	//speed
	ProtocolMsg.Output[3] = buffer[1];

	RADAR_GetSignalStrength(buffer);
	ProtocolMsg.Output[4] = buffer[0];

	RADAR_GetToiletCoverBStatus(buffer);
	ProtocolMsg.Output[5] = buffer[0];

	ProtocolMsg.Output[6] = 0;
	ProtocolMsg.Output[7] = 0;
	ProtocolMsg.Output[8] = 0;
	ProtocolMsg.Output[9] = 0;
}

/*static void SetPWMTripPointEx(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;
//	DISTANCE_TRIP_POINT_Type dis_point;

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

	radar_factory_data_bak.pwm_point = get16(ProtocolMsg.Para);

	if(radar_factory_data_bak.pwm_point != radar_factory_data.pwm_point)
	{
		if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
		{
			radar_factory_data.pwm_point = radar_factory_data_bak.pwm_point;
		}
		else
		{
			*ProtocolMsg.ErrorCode = SYSTEM_BUSY;
		}
	}

	RADAR_GetPWMTripPoint(ProtocolMsg.Output);
}*/



static void GetFirmwareVersion(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	ProtocolMsg.Output[0] = MAJOR_VERSION;
	ProtocolMsg.Output[1] = MINOR_VERSION;
	ProtocolMsg.Output[2] = BUILD_VERSION;
}

static void GetDeviceAddress(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	ProtocolMsg.Output[0] = radar_user_data.device_address;
}

static void GetNotifyState(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	ProtocolMsg.Output[0] = notify_state;
}

static void GetInternalCommandState(ProtocolMsg_TypeDef ProtocolMsg)
{
	ProtocolMsg.Output[0] = internal_cmd_state;
}

static void GetDistanceOffset(ProtocolMsg_TypeDef ProtocolMsg)
{
	INT8U i;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	for(i = 0; i < 8; i++)
	{
		ProtocolMsg.Output[i] = radar_factory_data.distance_offset[i];
	}
}

static void GetAmplitudeOffset(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	ProtocolMsg.Output[0] = WORD_HIGH(radar_factory_data.amplitude_offset[0]);
	ProtocolMsg.Output[1] = WORD_LOW(radar_factory_data.amplitude_offset[0]);
}

static void GetGestureTripPoint(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	ProtocolMsg.Output[0] = WORD_HIGH(radar_factory_data.gesture_dis_point);
	ProtocolMsg.Output[1] = WORD_LOW(radar_factory_data.gesture_dis_point);

	ProtocolMsg.Output[2] = radar_factory_data.gesture_spd_point;
}

static void GetDistanceTripPoint(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	ProtocolMsg.Output[0] = radar_factory_data.distance_point.step;
	ProtocolMsg.Output[1] = radar_factory_data.distance_point.speed;
}

static void GetDistanceSpeedOffsetState(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	RADAR_GetDistanceSpeedOffsetStatus(ProtocolMsg.Output);
}

static void GetPWMTripPoint(ProtocolMsg_TypeDef ProtocolMsg)
{
//	if(GetInternalCommandStatus() != internal_cmd_enable)
//	{
//		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
//		return;
//	}

	RADAR_GetPWMTripPoint(ProtocolMsg.Output);
}

static void GetCoverOpenedTripPoint(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	RADAR_GetCoverOpenedTripPoint(ProtocolMsg.Output);
}

static void GetSummaryInfo(ProtocolMsg_TypeDef ProtocolMsg)
{
	INT8U buffer[5];

	*ProtocolMsg.DynamicLen = 13;

	RADAR_GetDistance(buffer);
	ProtocolMsg.Output[0] = buffer[0];	//distance
	ProtocolMsg.Output[1] = buffer[1];

	RADAR_GetSpeedInCM(buffer);
	ProtocolMsg.Output[2] = buffer[0];	//speed
	ProtocolMsg.Output[3] = buffer[1];

	RADAR_GetMotion(buffer);
	ProtocolMsg.Output[4] = buffer[0];

	RADAR_GetSignal(buffer);
	ProtocolMsg.Output[5] = buffer[0];	//Signal
	ProtocolMsg.Output[6] = buffer[1];

	RADAR_GetAmplitude(buffer);
	ProtocolMsg.Output[7] = buffer[0];	//Amplitude
	ProtocolMsg.Output[8] = buffer[1];

	RADAR_GetToiletCover(buffer);
	ProtocolMsg.Output[9] = buffer[0];

	RADAR_GetToiletCoverBStatus(buffer);
	ProtocolMsg.Output[10] = buffer[0];

	RADAR_GetAmplitudeCalValue(buffer);
	ProtocolMsg.Output[11] = buffer[0];
	ProtocolMsg.Output[12] = buffer[1];
}


static void SetDeviceAddress(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_USER_DATA_Type radar_user_data_bak;
	INT8U  cur_add;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	RADAR_GetDeviceAddress(&cur_add);

	if(cur_add != ProtocolMsg.Para[0])
	{
		MEM_Copy(&radar_user_data_bak, &radar_user_data ,sizeof(radar_user_data));
		radar_user_data_bak.device_address = ProtocolMsg.Para[0];


		if(FLASH_WriteSpecificPage(flash_page_user, (INT8U *)&radar_user_data_bak, sizeof(radar_user_data)) == FLASH_NO_ERROR)
		{
//			radar_user_data.device_address = ProtocolMsg.Para[0];
			RADAR_SetDeviceAddress(ProtocolMsg.Para[0]);
		}
		else
		{
			*ProtocolMsg.ErrorCode = SYSTEM_BUSY;
	//		return;
		}
	}

	RADAR_GetDeviceAddress(ProtocolMsg.Output);
}

static void SetNotifyState(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	if((ProtocolMsg.Para[0] == notify_disable)
	 ||(ProtocolMsg.Para[0] == notify_internal)
	 ||(ProtocolMsg.Para[0] == notify_external))
	{
		notify_state = ProtocolMsg.Para[0];
	}

	ProtocolMsg.Output[0] = notify_state;
}

static void SetInternalCommandState(ProtocolMsg_TypeDef ProtocolMsg)
{
	if((ProtocolMsg.Para[0] == internal_cmd_enable)
	 ||(ProtocolMsg.Para[0] == internal_cmd_disable))
	{
		internal_cmd_state = ProtocolMsg.Para[0];
	}

	ProtocolMsg.Output[0] = internal_cmd_state;
}

static void SetDistanceOffset(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;
	INT8U i;
	INT8U new_set = 0;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

	for(i = 0; i < 8; i++)
	{
		radar_factory_data_bak.distance_offset[i] = (INT8S)ProtocolMsg.Para[i];
	}

	for(i = 0; i < 8; i++)
	{
		if(radar_factory_data_bak.distance_offset[i] != radar_factory_data.distance_offset[i])
		{
			new_set = 1;
			break;
		}
	}

	if(new_set)
	{
		if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
		{
			for(i = 0; i < 8; i++)
			{
				radar_factory_data.distance_offset[i] = (INT8S)ProtocolMsg.Para[i];
			}
		}
	}

	//output
	for(i = 0; i < 8; i++)
	{
		ProtocolMsg.Output[i] = radar_factory_data.distance_offset[i];
	}
}

static void SetAmplitudeOffset(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;
	INT16S para_data = 0;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

	radar_factory_data_bak.amplitude_offset[0] = (INT16S)get16(ProtocolMsg.Para);

	if(radar_factory_data_bak.amplitude_offset[0] != radar_factory_data.amplitude_offset[0])
	{
		if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
		{
			radar_factory_data.amplitude_offset[0] = para_data;
		}
	}

	//output
	ProtocolMsg.Output[0] = WORD_HIGH(radar_factory_data.amplitude_offset[0]);
	ProtocolMsg.Output[1] = WORD_LOW(radar_factory_data.amplitude_offset[1]);
}

static void SetGestureTripPoint(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;
	INT16U para_amplitude, para_speed;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

	para_amplitude = get16(ProtocolMsg.Para);
	para_speed = ProtocolMsg.Para[2];

	radar_factory_data_bak.gesture_dis_point = para_amplitude;
	radar_factory_data_bak.gesture_spd_point = para_speed;

	if((radar_factory_data_bak.gesture_dis_point != radar_factory_data.gesture_dis_point)
	|| (radar_factory_data_bak.gesture_spd_point != radar_factory_data.gesture_spd_point))
	{
		if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
		{
			radar_factory_data.gesture_dis_point = para_amplitude;
			radar_factory_data.gesture_spd_point = para_speed;
		}
	}

	ProtocolMsg.Output[0] = WORD_HIGH(radar_factory_data.gesture_dis_point);
	ProtocolMsg.Output[1] = WORD_LOW(radar_factory_data.gesture_dis_point);
	ProtocolMsg.Output[2] = radar_factory_data.gesture_spd_point;
}

static void SetSoftReset(ProtocolMsg_TypeDef ProtocolMsg)
{
	MSG_Cache_Type msg;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	if(ProtocolMsg.Para[0] == 0x30)
	{
		msg.Type = MsgType_Reset;
		msg.Value = MsgValue_NULL;
		APP_MessagePost(msg);
	}

	ProtocolMsg.Output[0] = ProtocolMsg.Para[0];
}


static void SetDistanceTripPoint(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;
//	DISTANCE_TRIP_POINT_Type dis_point;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

	radar_factory_data_bak.distance_point.step = ProtocolMsg.Para[0];
	radar_factory_data_bak.distance_point.speed = ProtocolMsg.Para[1];

	if((radar_factory_data_bak.distance_point.step != radar_factory_data.distance_point.step)
	|| (radar_factory_data_bak.distance_point.speed != radar_factory_data.distance_point.speed))
	{
		if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
		{
			radar_factory_data.distance_point.step = ProtocolMsg.Para[0];
			radar_factory_data.distance_point.speed = ProtocolMsg.Para[1];
		}
		else
		{
			*ProtocolMsg.ErrorCode = SYSTEM_BUSY;
		}
	}

	//output
	ProtocolMsg.Output[0] = radar_factory_data.distance_point.step;
	ProtocolMsg.Output[1] = radar_factory_data.distance_point.speed;
}

static void SetDistanceSpeedOffsetState(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;
//	DISTANCE_TRIP_POINT_Type dis_point;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

	radar_factory_data_bak.dis_spd_offset_status = ProtocolMsg.Para[0];

	if(radar_factory_data_bak.dis_spd_offset_status != radar_factory_data.dis_spd_offset_status)
	{
		if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
		{
			radar_factory_data.dis_spd_offset_status = ProtocolMsg.Para[0];
		}
		else
		{
			*ProtocolMsg.ErrorCode = SYSTEM_BUSY;
		}
	}


	ProtocolMsg.Output[0] = radar_factory_data.dis_spd_offset_status;
}


static void SetPWMTripPoint(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;
//	DISTANCE_TRIP_POINT_Type dis_point;
	INT16U cur_para;

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

	cur_para = get16(ProtocolMsg.Para);
	radar_factory_data_bak.pwm_point = cur_para;

	if(radar_factory_data_bak.pwm_point != radar_factory_data.pwm_point)
	{
		if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
		{
			radar_factory_data.pwm_point = cur_para;
		}
		else
		{
			*ProtocolMsg.ErrorCode = SYSTEM_BUSY;
	//		return;
		}
	}

	//output
	RADAR_GetPWMTripPoint(ProtocolMsg.Output);
}

static void SetCoverOpenedTripPoint(ProtocolMsg_TypeDef ProtocolMsg)
{
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;
//	DISTANCE_TRIP_POINT_Type dis_point;
	INT16U cur_para;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

	cur_para = get16(ProtocolMsg.Para);
	radar_factory_data_bak.cover_point = cur_para;

	if(radar_factory_data_bak.cover_point != radar_factory_data.cover_point)
	{
		if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
		{
			radar_factory_data.cover_point = cur_para;
		}
		else
		{
			*ProtocolMsg.ErrorCode = SYSTEM_BUSY;
	//		return;
		}
	}

	//output
	RADAR_GetCoverOpenedTripPoint(ProtocolMsg.Output);
}



static void LoadCalibrationData(ProtocolMsg_TypeDef ProtocolMsg)
{
	INT8U i, size;
	RADAR_FACTORY_DATA_Type 	radar_factory_data_bak;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	MEM_Copy(&radar_factory_data_bak, &radar_factory_data, sizeof(radar_factory_data));

//	MEM_Copy(&radar_factory_data_bak.cal_table_type, ProtocolMsg.Para, sizeof(radar_factory_data_bak.cal_table_type));

	radar_factory_data_bak.cal_table_type.planType = ProtocolMsg.Para[0];
	radar_factory_data_bak.cal_table_type.orderType = ProtocolMsg.Para[1];
	radar_factory_data_bak.cal_table_type.minDistance = ProtocolMsg.Para[2];
	radar_factory_data_bak.cal_table_type.maxDistance = ProtocolMsg.Para[3];

	size = sizeof(radar_factory_data_bak.cal_table_type.table) / 2;
	for(i = 0; i < size; i++)
	{
		radar_factory_data_bak.cal_table_type.table[i] = get16(&ProtocolMsg.Para[4 + 2*i]);
	}


	if(FLASH_WriteSpecificPage(flash_page_factory, (INT8U *)&radar_factory_data_bak, sizeof(radar_factory_data_bak)) == FLASH_NO_ERROR)
	{
		MEM_Copy(&radar_factory_data, &radar_factory_data_bak, sizeof(radar_factory_data));
	}
	else
	{
		*ProtocolMsg.ErrorCode = SYSTEM_BUSY;
//		return;
	}

	//output
	*ProtocolMsg.DynamicLen = sizeof(radar_factory_data_bak.cal_table_type);
	MEM_Copy(ProtocolMsg.Output, &radar_factory_data_bak.cal_table_type.planType, sizeof(radar_factory_data_bak.cal_table_type));
}

static void SetAmplitudeCalibrationMode(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	if((ProtocolMsg.Para[0] == CALIBRATION_MODE)
	|| (ProtocolMsg.Para[0] == FREE_MODE))
	{
		RADAR_SetCalibrationMode(ProtocolMsg.Para[0]);

		if(ProtocolMsg.Para[0] == FREE_MODE)
		{
			RADAR_SetAmplitudeCalValue(0);
		}
	}

	//output
	RADAR_GetCalibrationMode(ProtocolMsg.Output);
}

static void GetCalibrationData(ProtocolMsg_TypeDef ProtocolMsg)
{
	INT8U size, i;

	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	*ProtocolMsg.DynamicLen = sizeof(radar_factory_data.cal_table_type);

//	MEM_Copy(ProtocolMsg.Output, &radar_factory_data.cal_table_type, sizeof(radar_factory_data.cal_table_type));

	ProtocolMsg.Output[0] = radar_factory_data.cal_table_type.planType;
	ProtocolMsg.Output[1] = radar_factory_data.cal_table_type.orderType;
	ProtocolMsg.Output[2] = radar_factory_data.cal_table_type.minDistance;
	ProtocolMsg.Output[3] = radar_factory_data.cal_table_type.maxDistance;

	size = sizeof(radar_factory_data.cal_table_type.table) / 2;
	for(i = 0; i < size; i++)
	{
		ProtocolMsg.Output[4 + 2*i] = WORD_HIGH(radar_factory_data.cal_table_type.table[i]);
		ProtocolMsg.Output[4 + 2*i + 1] = WORD_LOW(radar_factory_data.cal_table_type.table[i]);
	}
}

static void GetAmplitude(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

//	RADAR_GetAmplitude(ProtocolMsg.Output);

	RADAR_GetAmplitudeCalValue(ProtocolMsg.Output);
}

static void GetAmplitudeCalibrationMode(ProtocolMsg_TypeDef ProtocolMsg)
{
	if(GetInternalCommandStatus() != internal_cmd_enable)
	{
		*ProtocolMsg.ErrorCode = UNSUPPORT_CMD;
		return;
	}

	RADAR_GetCalibrationMode(ProtocolMsg.Output);
}



