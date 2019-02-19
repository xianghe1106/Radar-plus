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
	INT16U table[10];
}DISTANCE_TABLE_Type;

typedef struct
{
	INT8U step;
	INT8U speed;
}DISTANCE_TRIP_POINT_Type;

typedef enum
{
	speed_offset_enable = 0,
	speed_offset_disable = 1
}DISTANCE_SPEED_OFFSET_Type;

typedef struct
{
	INT8S  distance_offset[8];
	INT16S amplitude_offset[8];

//	union
//	{
//		INT8U cal_table[24];
		DISTANCE_TABLE_Type cal_table_type;
//	}distance;

	INT16U gesture_dis_point;	//distance
	INT8U  gesture_spd_point;

	DISTANCE_TRIP_POINT_Type distance_point;

	DISTANCE_SPEED_OFFSET_Type dis_spd_offset_status;
	INT16U  pwm_point;
	INT16U  cover_point;

}RADAR_FACTORY_DATA_Type;

typedef struct
{
	INT8U device_address;
	INT8U baud_rate_option;
}RADAR_USER_DATA_Type;

typedef enum
{
	NO_GESTURE_DETECTED = 0,
	GESTURE_DETECTED = 1,
	GESTURE_UNDEFINED = 2
} RADAR_GESTURE_Type;

typedef enum
{
	TOILET_COVER_CLOSED = 0,
	TOILET_COVER_OPENED = 1
} RADAR_TOILET_COVER_Type;

typedef enum
{
	CALIBRATION_MODE = 0x30,
	FREE_MODE = 0x31
} RADAR_CALIBRATION_MODE_Type;


typedef struct
{
	INT8U  filter_cnt;
	INT16U buffer[2];
}Diatance_Type;


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

#define DISTANCE_STEP							50

#define DISTANCE_BUFFER_SIZE					3

#define MOTION_FILTER_CNT						2


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
//void RADAR_GetSpeed(INT8U *output);//in km/h
void RADAR_GetSpeedInCM(INT8U *output);//in cm/s
void RADAR_GetSignal(INT8U *output);
void RADAR_GetAmplitude(INT8U *output);
void RADAR_GetMotion(INT8U *output);
void RADAR_GetToiletCover(INT8U *output);
void RADAR_GetToiletCoverBStatus(RADAR_TOILET_COVER_Type *status);

void RADAR_SetCalibrationMode(RADAR_CALIBRATION_MODE_Type mode);
void RADAR_GetCalibrationMode(RADAR_CALIBRATION_MODE_Type *mode);
void RADAR_SetAmplitudeCalValue(INT16U amp);
void RADAR_GetAmplitudeCalValue(INT8U *output);

void RADAR_GetPWMTripPoint(INT8U *output);
void RADAR_GetDistanceSpeedOffsetStatus(INT8U *output);
void RADAR_GetCoverOpenedTripPoint(INT8U *output);
void RADAR_GetSignalStrength(INT8U *output);
void RADAR_GetDeviceAddress(INT8U *output);
void RADAR_SetDeviceAddress(INT8U para);
bool Radar_GetGestureState(void);

INT32U RADAR_GetBaudRate(void);


#endif /* USER_APP_RADAR_H_ */
