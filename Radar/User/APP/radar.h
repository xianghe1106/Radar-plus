/*
 * radar.h
 *
 *  Created on: Dec 6, 2018
 *      Author: xianghe
 */

#ifndef USER_APP_RADAR_H_
#define USER_APP_RADAR_H_

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "driver_Dave.h"
#include "cpu.h"
#include "radarsense2gol_library.h"
#include "config.h"
#include "xmc_uart.h"
#include "driver.h"

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
	INT8U  planType;
	INT8U  orderType;	//
	INT8U  minDistance;
	INT8U  maxDistance;
	INT16U table[6];
}DISTANCE_TABLE_Type;

typedef struct
{
	INT8S distance_offset[8];
	INT8S amplitude_offset[8];
	union
	{
		INT8U cal_table[16];
		DISTANCE_TABLE_Type cal_table_type;
	}distance;
}RADAR_FACTORY_DATA_Type;

typedef struct
{
	INT8U device_address;
}RADAR_USER_DATA_Type;

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

#define BUFF_SIZE (FFT_SIZE)		/**< I and Q raw samples buffer size */
#define FFT_BIN_SIZE ((float)SAMPLING_FREQ_HZ / FFT_SIZE) /**< size of each FFT bin. DO NOT CHANGE!!! */

extern uint16_t g_sampling_data_I[BUFF_SIZE];				 /**< raw data i channel */
extern uint16_t g_sampling_data_Q[BUFF_SIZE];

extern RADAR_FACTORY_DATA_Type 	radar_factory_data;
extern RADAR_USER_DATA_Type		radar_user_data;

//extern INT8U radar_flash_buffer[FLASH_BUFFER_SIZE];

/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/

#define SAMPLING_SIZE							2


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void RADAR_Test(void);

void RADAR_Init(void);

void RADAR_Process(void);


/*API*/
void RADAR_GetDistance(INT8U *output);
void RADAR_GetSpeed(INT8U *output);
void RADAR_GetSignal(INT8U *output);
void RADAR_GetAmplitude(INT8U *output);
void RADAR_GetMotion(INT8U *output);


#endif /* USER_APP_RADAR_H_ */
