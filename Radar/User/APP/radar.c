/*
 * radar.c
 *
 *  Created on: Dec 6, 2018
 *      Author: xianghe
 */


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "radar.h"
#include <math.h>
#include <stdlib.h>
//#include "HostCommUART.h"
#include "xmc_uart.h"
#include <DAVE.h>

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



/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/





RADAR_FACTORY_DATA_Type radar_factory_data;
RADAR_USER_DATA_Type	radar_user_data;
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

/*!
 * \brief Setup the timings related parameters
 */

XMC_RADARSENSE2GOL_TIMING_t radarsense2gol_timing =
{
		.t_sample_us = (1.0 / SAMPLING_FREQ_HZ) * 1000.0 * 1000.0, /**< sample time in us = (1/sample_frequency) * 1000 * 1000 */
		.t_cycle_ms = 100,           /**< 300 ms */
		.N_exponent_samples = log2(BUFF_SIZE)
};


/*!
 * \brief Setup the detection triggers related parameters
 */
XMC_RADARSENSE2GOL_ALG_t radarsense2gol_algorithm =
{
		.hold_on_cycles = 1,      /**< hold-on cycles to trigger detection */
		.trigger_det_level = DETECTION_THRESHOLD,  /**< trigger detection level */
		.rootcalc_enable = XMC_RADARSENSE2GOL_DISABLED /**< root calculation for magnitude disabled */
};

/*!
 * \brief Setup the power management related parameters
 */
XMC_RADARSENSE2GOL_POWERDOWN_t radarsense2gol_powerdown =
{
		.sleep_deepsleep_enable   = XMC_RADARSENSE2GOL_DISABLED, /**< sleep / deepsleep enabled */
		.mainexec_enable          = XMC_RADARSENSE2GOL_ENABLED, /**< main exec enabled */
		.vadc_clock_gating_enable = XMC_RADARSENSE2GOL_ENABLED  /**< vadc clock gating enabled */
};

/**********************************************************************************************************************
 * Micirum uC Probe variables
 **********************************************************************************************************************/

uint16_t g_sampling_data_I[BUFF_SIZE] = {0};				 /**< raw data i channel */
uint16_t g_sampling_data_Q[BUFF_SIZE] = {0};				 /**< raw data q channel */
uint32_t g_fft_data[FFT_SIZE/2] = {0};						 /**< fft data of i channel */
XMC_RADARSENSE2GOL_MOTION_t g_motion = XMC_NO_MOTION_DETECT; /**< motion indicator */
float g_doppler_frequency = 0.0;							 /**< doppler frequency */
float g_doppler_velocity = 0.0;								 /**< doppler speed */
float g_min_velocity = MOTION_MIN_VELOCITY; 				 /**< min velocity to detect motion */
float g_max_velocity = MOTION_MAX_VELOCITY; 				 /**< max velocity to detect motion */
bool g_start = true;										 /**< control for execution of doppler algorithm */
bool g_uart_start = UART_RAW_DATA;							 /**< control for execution of UART feature to transmit raw IQ from ADC */

/*QUEUE16_Type crest_I_queue, *p_crest_I_queue = &crest_I_queue;
QUEUE16_Type trough_I_queue, *p_trough_I_queue = &trough_I_queue;

INT16U crest_I_buffer[SAMPLING_SIZE];
INT16U trough_I_buffer[SAMPLING_SIZE];*/

//TABLE_Type DistanceTable;
//INT8U para_buffer[100];

//INT8U radar_flash_buffer[512];

RADAR_TOILET_COVER_Type toilet_cover_a = TOILET_COVER_CLOSED, toilet_cover_b = TOILET_COVER_CLOSED;
//INT16U sample_bak_buffer[BUFF_SIZE];

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void RADAR_SetToiletCoverAStatus(RADAR_TOILET_COVER_Type status);
void RADAR_GetToiletCoverAStatus(RADAR_TOILET_COVER_Type *status);
void RADAR_SetToiletCoverBStatus(RADAR_TOILET_COVER_Type status);
void RADAR_GetToiletCoverBStatus(RADAR_TOILET_COVER_Type *status);

void RADAR_TestTime(void);
void radarsense2gol_result( uint32_t *fft_magnitude_array,
		uint16_t size_of_array_mag,
		int16_t *adc_aqc_array_I,
		int16_t *adc_aqc_array_Q,
		uint16_t size_of_array_acq,
		XMC_RADARSENSE2GOL_MOTION_t motion,
		uint32_t max_frq_mag,
		uint32_t max_frq_index);

void Radar_SearchMaxMinSamplingData(INT16U *input, INT16U size, INT16U *output);


void RADAR_Test(void)
{
	DIGITAL_GPIO_ToggleOutput(&LED_BLUE);
//	DIGITAL_GPIO_SetOutputLow(&LED_BLUE);
//	DIGITAL_GPIO_ToggleOutput(&LED_ORANGE);
//	DIGITAL_GPIO_ToggleOutput(&LED_BLUE);
}

//INT8U flash_bu[10];
void RADAR_Init(void)
{
	INT8U radar_flash_buffer[FLASH_BUFFER_SIZE];

	radarsense2gol_init(
			radarsense2gol_timing,
			radarsense2gol_algorithm,
			radarsense2gol_powerdown,
			&TIMER_0
	);

	// register call backs
	radarsense2gol_regcb_result ( radarsense2gol_result );
	radarsense2gol_exitmain();


	FLASH_ReadSpecificPage(flash_page_factory, radar_flash_buffer);
	MEM_Clr(&radar_factory_data, sizeof(radar_factory_data));
	MEM_Copy(&radar_factory_data, radar_flash_buffer, sizeof(radar_factory_data));

	if(radar_factory_data.cal_table_type.planType != 'A')
	{
		radar_factory_data.cal_table_type.planType = 'A';
		radar_factory_data.cal_table_type.orderType = '1';

	#if 0
		radar_factory_data.distance.cal_table_type.minDistance = 5;		//0.5m
		radar_factory_data.distance.cal_table_type.maxDistance = 30;	//3m
		radar_factory_data.distance.cal_table_type.table[0] = 3544;		//0.5
		radar_factory_data.distance.cal_table_type.table[1] = 2605;		//1.0 fake
		radar_factory_data.distance.cal_table_type.table[2] = 1667;		//1.5
		radar_factory_data.distance.cal_table_type.table[3] = 963;		//2.0
		radar_factory_data.distance.cal_table_type.table[4] = 660;		//2.5 fake
		radar_factory_data.distance.cal_table_type.table[5] = 358;		//3.0
	#endif

		radar_factory_data.cal_table_type.minDistance = 5;		//0.5m
		radar_factory_data.cal_table_type.maxDistance = 30;	//3m

		radar_factory_data.cal_table_type.table[0] = 800;		//0.5
		radar_factory_data.cal_table_type.table[1] = 220;		//1.0 fake
		radar_factory_data.cal_table_type.table[2] = 190;		//1.5
		radar_factory_data.cal_table_type.table[3] = 160;		//2.0
		radar_factory_data.cal_table_type.table[4] = 150;		//2.5 fake
		radar_factory_data.cal_table_type.table[5] = 120;		//3.0

		radar_factory_data.gesture_dis_point = 1000;
		radar_factory_data.gesture_spd_point = 20;	//2KM/H
	}


	radar_user_data.device_address = 0x30;
}

void RADAR_Process(void)
{
	static bool running = false;

	if (running == false)
	{
		if (g_start == true)
		{
			running = true;
			radarsense2gol_start();
		}
	}
	else
	{
		if (g_start == false)
		{
			running = false;
			radarsense2gol_stop();
		}

		radarsense2gol_set_detection_threshold(radarsense2gol_algorithm.trigger_det_level);

		/* place your application code for main execution here */
		/* e.g. communication on peripherals */
		radarsense2gol_exitmain(); /* only need to be called if mainexec_enable is enabled during init */

	}
}



/*! max_frw_index gives the fft bin in which the maximum magnitude of the fft was found each fft bin has a size
 * and therefore corresponds to a frequency range the size of the fft bin is calculated by dividing the sampling frequency
 * by the number of fft points the doppler frequency is calculated by multiplying the fft bin size with the number
 * of the fft bin where the maximum was found.
 */
static float calcDopplerFrequency(const uint32_t max_frq_index)
{
	return (float)(max_frq_index * FFT_BIN_SIZE);
}

/************************************************************************************************************/

/*! doppler velocity is calculated based on the doppler frequency
 * for 24GHz doppler radar systems a velocity of 1km/h will correspond to a doppler frequency of 44.4Hz
 * doppler velocity can therefore be calculated by dividing the doppler frequency by 44.4Hz
 */

static float calcDopplerSpeed(const float doppler_freq)
{
	return doppler_freq / 44.4f;
}

/************************************************************************************************************/

/*!
 * \brief Callback executed after new data is available from algorithm.
 *
 * \param[in] *fft_magnitude_array  Array pointer to the FFT spectrum
 * \param[in] size_of_array_mag  Length of array for the FFT spectrum
 * \param[in] *adc_aqc_array_I   Array pointer for the raw ADC I data samples
 * \param[in] *adc_aqc_array_Q   Array pointer for the raw ADC Q data samples
 * \param[in] size_of_array_acq  Length of array for the raw ADC I&Q data samples
 * \param[out] motion  Approaching, departing or no motion information
 * \param[out] max_frq_mag  Maximum doppler frequency computed
 * \param[out] max_frq_index  Maximum frequency bin index above threshold
 *
 */
void radarsense2gol_result( uint32_t *fft_magnitude_array,
		uint16_t size_of_array_mag,
		int16_t *adc_aqc_array_I,
		int16_t *adc_aqc_array_Q,
		uint16_t size_of_array_acq,
		XMC_RADARSENSE2GOL_MOTION_t motion,
		uint32_t max_frq_mag,
		uint32_t max_frq_index)
{
	// copy raw data and fft data and motion indicator to global variables used in micrium GUI
	memcpy(g_sampling_data_I, adc_aqc_array_I, size_of_array_acq * sizeof(uint16_t));
	memcpy(g_sampling_data_Q, adc_aqc_array_Q, size_of_array_acq * sizeof(uint16_t));
	memcpy(g_fft_data, &fft_magnitude_array[1], (size_of_array_mag - 1) * sizeof(uint32_t));

	// To remove the spike from last bins in fft spectrum, force last two bins to 0
	g_fft_data[size_of_array_mag-1] = 0;
	g_fft_data[size_of_array_mag-2] = 0;

	fft_magnitude_array[size_of_array_mag-1] = 0;
	fft_magnitude_array[size_of_array_mag-2] = 0;

	g_motion = motion;

	// calc doppler frequency and velocity
	g_doppler_frequency = calcDopplerFrequency(max_frq_index);
	g_doppler_velocity = calcDopplerSpeed(g_doppler_frequency);


//	if (g_uart_start)
//		dumpRawIQ_uint16(g_sampling_data_I, g_sampling_data_Q, (uint16_t)BUFF_SIZE);

	// check results
	if (motion != XMC_NO_MOTION_DETECT &&			// motion detected
	    g_doppler_velocity > g_min_velocity &&		// doppler velocity is greater than min velocity
		g_doppler_velocity < g_max_velocity)		// doppker velocity is less than max veloctiy
	{
		if (motion == XMC_MOTION_DETECT_APPROACHING)	// target is approaching radar
		{
			// turn on red LED, turn off orange and blue LEDs
//			DIGITAL_IO_SetOutputLow(&LED_RED);
//			DIGITAL_IO_SetOutputHigh(&LED_ORANGE);
//			DIGITAL_GPIO_SetOutputLow(&LED_BLUE);
		}
		else // motion == XMC_MOTION_DETECT_DEPARTING => target is moving away from radar
		{
			// turn on orange LED, turn off red and blue LEDs
			DIGITAL_GPIO_SetOutputLow(&LED_ORANGE);
//			DIGITAL_IO_SetOutputHigh(&LED_RED);
//			DIGITAL_IO_SetOutputHigh(&LED_BLUE);
		}
	}
	else // no motion detected
	{
		// turn on blue LED, turn off red and blue LEDs
//		DIGITAL_GPIO_SetOutputHigh(&LED_BLUE);
		DIGITAL_GPIO_SetOutputHigh(&LED_ORANGE);
//		DIGITAL_IO_SetOutputHigh(&LED_RED);

		// set velocity and frequency to 0 in case of no motion
		g_doppler_frequency = 0.0;
		g_doppler_velocity = 0.0;
		g_motion = XMC_NO_MOTION_DETECT;
	}
}

void Radar_SearchMaxMinSamplingData(INT16U *input, INT16U size, INT16U *output)
{
	INT16U i;

	output[0] = input[0];
	output[1] = input[0];

	for(i = 0; i < size; i++)
	{
		if(input[i] > output[0])
		{
			output[0] = input[i];	//serch max data
		}

		if(input[i] < output[1])
		{
			output[1] = input[i];	//search min data
		}
	}
}

void RADAR_FactoryDataInitFromFlash(void)
{
	INT8U radar_flash_buffer[FLASH_BUFFER_SIZE];

	MEM_Clr(&radar_factory_data, sizeof(radar_factory_data));

	if(FLASH_ReadSpecificPage(flash_page_factory, radar_flash_buffer) == FLASH_NO_ERROR)
	{
		MEM_Copy(&radar_factory_data, radar_flash_buffer, sizeof(radar_factory_data));
	}
}



void RADAR_UserDataInitFromDefaults(void)
{
	INT8U radar_flash_buffer[FLASH_BUFFER_SIZE];

	MEM_Clr(&radar_user_data, sizeof(radar_user_data));

	if(FLASH_ReadSpecificPage(flash_page_user, radar_flash_buffer) == FLASH_NO_ERROR)
	{
		MEM_Copy(&radar_user_data, radar_flash_buffer, sizeof(radar_user_data));
	}

	if((radar_user_data.device_address < 15)
	|| (radar_user_data.device_address > 250))
	{
		radar_user_data.device_address = 0x30;
	}
}



/*
 * API
 */

INT16U DistanceLinear(INT16U base_dis, INT16U samle_value, INT16U *cal_table)
{
	INT16U output = 250;

	if(cal_table[0] == cal_table[1])
	{
		return output;
	}

	output = base_dis - (((samle_value - cal_table[1]) * 50) / (cal_table[0] - cal_table[1]));

	return output;
}

void RADAR_GetDistance(INT8U *output)
{
	INT16U value = 0;
	INT16U buffer[2], cur_speed;
	volatile INT32U new_distance_value = 0;
	volatile static INT32U cur_distance_value;
	volatile static INT8U startup = 0;
	volatile static INT16U dis_buffer[20];
	INT16U i, size;
	INT32U dis_sum;
	INT8U  byte_buffer[2];

	XMC_RADARSENSE2GOL_MOTION_t motion;

	Radar_SearchMaxMinSamplingData(g_sampling_data_I, BUFF_SIZE, buffer);

	value = buffer[0] - buffer[1];

	RADAR_GetSpeed(byte_buffer);
	cur_speed = get16(byte_buffer);

	RADAR_GetMotion(&motion);

	for(i = 0; i < (sizeof(dis_buffer) / 2 - 1); i++) //dis_buffer[sizeof(dis_buffer) - 1] Stay the same
	{
		dis_buffer[i] = dis_buffer[i + 1];
	}

	if((cur_speed < radar_factory_data.gesture_spd_point)
		&& (SCH_Get_Flag(Flag_SampleFilter) == 0)
		&& ((motion == XMC_MOTION_DETECT_APPROACHING) || (motion == XMC_MOTION_DETECT_DEPARTING)))
	{
		if(value >= radar_factory_data.cal_table_type.table[0])
		{
			new_distance_value = 50;
		}
		else if(value > radar_factory_data.cal_table_type.table[1])
		{
			new_distance_value = DistanceLinear(100, value, &radar_factory_data.cal_table_type.table[0]);
		}
		else if(value > radar_factory_data.cal_table_type.table[2])
		{
			new_distance_value = DistanceLinear(150, value, &radar_factory_data.cal_table_type.table[1]);
		}
		else if(value > radar_factory_data.cal_table_type.table[3])
		{
			new_distance_value = DistanceLinear(200, value, &radar_factory_data.cal_table_type.table[2]);
		}
		else if(value > radar_factory_data.cal_table_type.table[4])
		{
			new_distance_value = DistanceLinear(250, value, &radar_factory_data.cal_table_type.table[3]);
		}
		else if(value > radar_factory_data.cal_table_type.table[5])
		{
			new_distance_value = DistanceLinear(300, value, &radar_factory_data.cal_table_type.table[4]);
		}
		else
		{
			new_distance_value = 300;
		}

		dis_buffer[sizeof(dis_buffer)/2 - 1] = new_distance_value;
	}
	else
	{
		if(SCH_Get_Flag(Flag_SampleFilter) == 0)
		{
			SCH_Add_Flag(Flag_SampleFilter, SCH_FLAG_300MS);
		}

		size = (sizeof(dis_buffer) / 2) / 3;
		for(dis_sum = 0, i = 0; i < size; i++)
		{
			dis_sum += dis_buffer[i];
		}

		dis_sum = dis_sum / size;

		for(i = 0; i < (sizeof(dis_buffer) / 2); i++)
		{
			dis_buffer[i] = dis_sum;
		}
	}

	if(startup == 0)
	{
		startup = 1;

		cur_distance_value = new_distance_value;

		for(i = 0; i < (sizeof(dis_buffer) / 2); i++)
		{
			dis_buffer[i] = cur_distance_value;
		}
	}


	dis_sum = 0;
	for(i = 0; i < (sizeof(dis_buffer) / 2); i++)
	{
		dis_sum += dis_buffer[i];
	}

	new_distance_value = dis_sum / (sizeof(dis_buffer) / 2);

	RADAR_GetMotion(&motion);

	if(motion == XMC_MOTION_DETECT_APPROACHING)
	{
		if(new_distance_value <= cur_distance_value)
		{
			cur_distance_value = new_distance_value;
		}
	}
	else if(motion == XMC_MOTION_DETECT_DEPARTING)
	{
		if(new_distance_value >= cur_distance_value)
		{
			cur_distance_value = new_distance_value;
		}
	}

	output[0] = WORD_HIGH(cur_distance_value);
	output[1] = WORD_LOW(cur_distance_value);
}

void RADAR_GetSpeed(INT8U *output)
{
	INT16U speed = 0;

	speed = (10 * g_doppler_velocity);

	output[0] = WORD_HIGH(speed);
	output[1] = WORD_LOW(speed);
}

void RADAR_GetSignal(INT8U *output)
{
	INT16U value = 0;

	value = 10 * g_doppler_frequency;

	output[0] = WORD_HIGH(value);
	output[1] = WORD_LOW(value);
}

void RADAR_GetAmplitude(INT8U *output)
{
	INT16U value = 0;
	INT16U buffer[2];

	Radar_SearchMaxMinSamplingData(g_sampling_data_I, BUFF_SIZE, buffer);

	value = buffer[0] - buffer[1];
	output[0] = WORD_HIGH(value);
	output[1] = WORD_LOW(value);
}

void RADAR_GetMotion(INT8U *output)
{
	output[0] = g_motion;
}

void RADAR_GetGesture(INT8U *output)
{
	INT8U buffer[2];
	INT16U cur_speed;
//	static RADAR_GESTURE_Type state = GESTURE_UNDEFINED;
	RADAR_TOILET_COVER_Type cover_status;

	RADAR_GetSpeed(buffer);
	cur_speed = get16(buffer);

//	RADAR_GetDistance(buffer);
//	cur_distance = get16(buffer);

	RADAR_GetToiletCoverAStatus(&cover_status);

	if((cur_speed > radar_factory_data.gesture_spd_point)
	&& (cover_status == TOILET_COVER_OPENED))
	{
//		state = GESTURE_DETECTED;

		RADAR_SetToiletCoverBStatus(TOILET_COVER_OPENED);
	}

//	if(cur_distance >150)
//	{
//		state = NO_GESTURE_DETECTED;
//	}

	RADAR_GetToiletCoverBStatus(output);
}

void RADAR_GetToiletCover(INT8U *output)
{
	INT8U  buffer[5];
	RADAR_TOILET_COVER_Type cover_b_state;

	RADAR_GetDistance(buffer);

	if(get16(buffer) < 150)
	{
		SCH_Add_Flag(Flag_CoverADelay, SCH_FLAG_10000MS);

//		state = TOILET_COVER_OPENED;

		RADAR_SetToiletCoverAStatus(TOILET_COVER_OPENED);
	}
	else
	{
		if(SCH_Get_Flag(Flag_CoverADelay) == 0)
		{
			RADAR_GetToiletCoverBStatus(&cover_b_state);
			if(cover_b_state == TOILET_COVER_OPENED)
			{
				RADAR_SetToiletCoverBStatus(TOILET_COVER_CLOSED);

				if(SCH_Get_Flag(Flag_CoverBDelay) == 0)
				{
					SCH_Add_Flag(Flag_CoverBDelay, SCH_FLAG_1500MS);	//1.5s delay
				}
			}

			if(SCH_Get_Flag(Flag_CoverBDelay) == 0)
			{
//				state = TOILET_COVER_CLOSED;

				RADAR_SetToiletCoverAStatus(TOILET_COVER_CLOSED);
			}
		}
	}



//	output[0] = state;

	RADAR_GetToiletCoverAStatus(output);
}


//API

void RADAR_SetToiletCoverAStatus(RADAR_TOILET_COVER_Type status)
{
	toilet_cover_a = status;
}

void RADAR_GetToiletCoverAStatus(RADAR_TOILET_COVER_Type *status)
{
	*status = toilet_cover_a;
}

void RADAR_SetToiletCoverBStatus(RADAR_TOILET_COVER_Type status)
{
	toilet_cover_b = status;
}

void RADAR_GetToiletCoverBStatus(RADAR_TOILET_COVER_Type *status)
{
	*status = toilet_cover_b;
}



