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

#include "pwm.h"
#include "Protocol.h"

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

const INT32U baud_rate_buffer[] = {115200, 57600, 38400, 28800, 19200, 14400, 9600, 4800, 2400, 1200};

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


RADAR_TOILET_COVER_Type toilet_cover_a = TOILET_COVER_CLOSED, toilet_cover_b = TOILET_COVER_CLOSED;

RADAR_CALIBRATION_MODE_Type RADAR_CALIBRATION_MODE = FREE_MODE;
INT16U AMPLITUDE_CAL_VALUE = 0;

INT16U RADAR_DISTANCE_BUFFER[DISTANCE_BUFFER_SIZE] = {160};
INT16U CUR_AMPLITUDE_VALUE = 0;
INT16U CUR_DISTANCE_VALUE = 0;

Diatance_Type dis_depart, dis_approach;

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

//void Radar_SearchMaxMinSamplingData(INT16U *input, INT16U size, INT16U *output);
//INT16U Distance_CalMaxAmplitude(void);

void Radar_DistanceUpdate(void);
void Radar_MotionUpdate(XMC_RADARSENSE2GOL_MOTION_t motion);
void Radar_PWMStateUpdate(void);
void Radar_ToiletCoverUpdate(void);



void RADAR_Test(void)
{
	DIGITAL_GPIO_ToggleOutput(&LED_BLUE);
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

		radar_factory_data.dis_spd_offset_status = speed_offset_disable;
		radar_factory_data.pwm_point = 150;

		radar_factory_data.cover_point = 150;
	}


	FLASH_ReadSpecificPage(flash_page_user, radar_flash_buffer);
	MEM_Clr(&radar_user_data, sizeof(radar_user_data));
	MEM_Copy(&radar_user_data, radar_flash_buffer, sizeof(radar_user_data));

	radar_user_data.device_address = 0x30;
	if((radar_user_data.baud_rate_option == 0)
	&& (radar_user_data.baud_rate_option > 10))
	{
		radar_user_data.baud_rate_option = 0x03;
	}

	RADAR_CALIBRATION_MODE = FREE_MODE;

	MEM_Clr(&dis_depart, sizeof(dis_depart));
	MEM_Clr(&dis_approach, sizeof(dis_approach));
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
//	INT16U word_buffer[4];
	RADAR_CALIBRATION_MODE_Type cal_mode;
	XMC_RADARSENSE2GOL_MOTION_t cur_motion;

	// copy raw data and fft data and motion indicator to global variables used in micrium GUI
	memcpy(g_sampling_data_I, adc_aqc_array_I, size_of_array_acq * sizeof(uint16_t));
	memcpy(g_sampling_data_Q, adc_aqc_array_Q, size_of_array_acq * sizeof(uint16_t));
	memcpy(g_fft_data, &fft_magnitude_array[1], (size_of_array_mag - 1) * sizeof(uint32_t));

	// To remove the spike from last bins in fft spectrum, force last two bins to 0
	g_fft_data[size_of_array_mag-1] = 0;
	g_fft_data[size_of_array_mag-2] = 0;

	fft_magnitude_array[size_of_array_mag-1] = 0;
	fft_magnitude_array[size_of_array_mag-2] = 0;


	// calc doppler frequency and velocity
	g_doppler_frequency = calcDopplerFrequency(max_frq_index);
	g_doppler_velocity = calcDopplerSpeed(g_doppler_frequency);

	/*-- motion update --*/
	Radar_MotionUpdate(motion);

	/*-- distance update --*/
	Radar_DistanceUpdate();

	/*-- Toilet Cover update --*/
	Radar_ToiletCoverUpdate();

	/*-- PWM State update --*/
	Radar_PWMStateUpdate();

	/*-- Calibration --*/
	RADAR_GetCalibrationMode(&cal_mode);
	if(cal_mode == CALIBRATION_MODE)
	{
		RADAR_SetAmplitudeCalValue(CUR_AMPLITUDE_VALUE);
	}


	Protocol_heart_beat();

	RADAR_GetMotion(&cur_motion);
	if((cur_motion == XMC_MOTION_DETECT_APPROACHING)
	|| (cur_motion == XMC_MOTION_DETECT_DEPARTING))
	{
		Protocol_heart_beat_ex();
	}

}


/*void RADAR_FactoryDataInitFromFlash(void)
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

//	if((radar_user_data.device_address < 15)
//	|| (radar_user_data.device_address > 250))
//	{
		radar_user_data.device_address = 0x30;
//	}
}*/

INT16U Distance_GetBufferSum(INT16U *buffer, INT8U size)
{
	INT8U  i;
	INT16U output = 0;

	for(i = 0; i < size; i++)
	{
		output += buffer[i];
	}

	return output;
}

INT16U Distance_CalLatestDistance(INT16U cur_amplitude)
{
	INT16U output = 0;

	output = radar_factory_data.cal_table_type.table[0] / 2;
	output = (100 * output) / cur_amplitude;

	return output;
}

INT16U Distance_CalMaxAmplitude(void)
{
	INT16U i;
	INT16U output = 0;
	INT16U max_value, min_value;

	max_value = g_sampling_data_I[0];
	min_value = max_value;

	for(i = 0; i < BUFF_SIZE; i++)
	{
		if(g_sampling_data_I[i] > max_value)
		{
			max_value = g_sampling_data_I[i];	//serch max data
		}

		if(g_sampling_data_I[i] < min_value)
		{
			min_value = g_sampling_data_I[i];	//search min data
		}
	}

	output = max_value - min_value;

	return output;
}

void Distance_JitterResistant(XMC_RADARSENSE2GOL_MOTION_t motion, Diatance_Type *depart, Diatance_Type *approach)
{
	if(motion == XMC_MOTION_DETECT_APPROACHING)
	{
		depart->filter_cnt = 0;

		approach->filter_cnt++;
		if(approach->filter_cnt >= MOTION_FILTER_CNT)
		{
			approach->filter_cnt = MOTION_FILTER_CNT;
		}
	}
	else	// depart
	{
		approach->filter_cnt = 0;

		depart->filter_cnt++;
		if(depart->filter_cnt >= MOTION_FILTER_CNT)
		{
			depart->filter_cnt = MOTION_FILTER_CNT;
		}
	}
}

bool Distance_FilterInvalidData(Diatance_Type *depart, Diatance_Type *approach, INT16U *cur_distance)
{
	bool status = true;
	INT16U dis_sum;
	INT16U average_value;
	INT16U delta_distance = 0;
	INT8U  byte_buffer[2];
	INT16U cur_speed, dis_offset = 0;
	DISTANCE_SPEED_OFFSET_Type  speed_status;

	RADAR_GetSpeedInCM(byte_buffer);
	cur_speed = get16(byte_buffer);

	dis_sum = Distance_GetBufferSum(RADAR_DISTANCE_BUFFER, sizeof(RADAR_DISTANCE_BUFFER) / 2);
	average_value = dis_sum / (sizeof(RADAR_DISTANCE_BUFFER) / 2);

	dis_offset = (10 * cur_speed) / 36; // 1Km/H = 100000/3600 = 27.7 cm/s = 2.77 cm/100ms

	dis_offset = dis_offset / 4;

	if(depart->filter_cnt == MOTION_FILTER_CNT)			//depart
	{
		if(*cur_distance < average_value)
		{
			if(radar_factory_data.dis_spd_offset_status == speed_offset_enable)
			{
				status = true;
			}
			else
			{
				status = false;
			}

			dis_offset = average_value + dis_offset;
			*cur_distance = dis_offset;
			return status;
		}

		delta_distance = *cur_distance - average_value;
		dis_offset = average_value + dis_offset;
	}
	else	//approach
	{
		if(*cur_distance > average_value)
		{
			if(radar_factory_data.dis_spd_offset_status == speed_offset_enable)
			{
				status = true;
			}
			else
			{
				status = false;
			}

			dis_offset = average_value - dis_offset;
			*cur_distance = dis_offset;

			return status;
		}

		delta_distance = average_value - *cur_distance;
		dis_offset = average_value - dis_offset;
	}

	if(delta_distance > radar_factory_data.distance_point.step)
	{
		RADAR_GetDistanceSpeedOffsetStatus(&speed_status);
		if(speed_status == speed_offset_enable)
		{
			*cur_distance = dis_offset;
		}
		else
		{
			status = false;
			return status;
		}
	}

	return status;
}

void Distance_DistanceBufferUpdate(Diatance_Type *depart, Diatance_Type *approach, INT16U cur_distance)
{
	INT8U  i;
	INT16U dis_sum;
	INT16U average_value;

	dis_sum = Distance_GetBufferSum(RADAR_DISTANCE_BUFFER, sizeof(RADAR_DISTANCE_BUFFER) / 2);
	average_value = dis_sum / (sizeof(RADAR_DISTANCE_BUFFER) / 2);

	for(i = 0; i < (sizeof(RADAR_DISTANCE_BUFFER) / 2 - 1); i++) //dis_buffer[sizeof(dis_buffer) - 1] Stay the same
	{
		RADAR_DISTANCE_BUFFER[i] = RADAR_DISTANCE_BUFFER[i + 1];	//Throw away the oldest data.
	}

	if(depart->filter_cnt == MOTION_FILTER_CNT)
	{
		depart->buffer[0] = average_value;
		depart->buffer[1] = cur_distance;

		RADAR_DISTANCE_BUFFER[sizeof(RADAR_DISTANCE_BUFFER) / 2 - 1] = (depart->buffer[0] + depart->buffer[1]) / 2;
	}
	else
	{
		approach->buffer[0] = average_value;
		approach->buffer[1] = cur_distance;

		RADAR_DISTANCE_BUFFER[sizeof(RADAR_DISTANCE_BUFFER) / 2 - 1] = (approach->buffer[0] + approach->buffer[1]) / 2;
	}
}

void Radar_DistanceUpdate(void)
{
	INT8U  byte_buffer[4];
	INT16U cur_speed, new_distance_value;
	XMC_RADARSENSE2GOL_MOTION_t motion;
	static INT8U startup = 0;
	INT16U dis_sum;

	RADAR_GetSpeedInCM(byte_buffer);
	cur_speed = get16(byte_buffer);
	RADAR_GetMotion(&motion);


	CUR_AMPLITUDE_VALUE = Distance_CalMaxAmplitude();

	if(cur_speed >= radar_factory_data.gesture_spd_point)
	{
		SCH_Add_Flag(Flag_DistanceDelay, SCH_FLAG_200MS);
	}

	if((SCH_Get_Flag(Flag_DistanceDelay) > 0)
	|| (motion == XMC_NO_MOTION_DETECT)
	|| (cur_speed <= radar_factory_data.distance_point.speed))
	{
		return;
	}

	new_distance_value = Distance_CalLatestDistance(CUR_AMPLITUDE_VALUE);

	if((new_distance_value < 10 * radar_factory_data.cal_table_type.minDistance)
	|| (new_distance_value > 10 * radar_factory_data.cal_table_type.maxDistance))
	{
		return;
	}

	Distance_JitterResistant(motion, &dis_depart, &dis_approach);

	if((dis_depart.filter_cnt == MOTION_FILTER_CNT) || (dis_approach.filter_cnt == MOTION_FILTER_CNT))
	{
		startup++;
		if(startup >= MOTION_FILTER_CNT)
		{
			startup = MOTION_FILTER_CNT;
		}

		if(Distance_FilterInvalidData(&dis_depart, &dis_approach, &new_distance_value) == false)
		{
			return;
		}

		Distance_DistanceBufferUpdate(&dis_depart, &dis_approach, new_distance_value);
	}


	dis_sum = Distance_GetBufferSum(RADAR_DISTANCE_BUFFER, sizeof(RADAR_DISTANCE_BUFFER) / 2);

	CUR_DISTANCE_VALUE = dis_sum / (sizeof(RADAR_DISTANCE_BUFFER) / 2);

//	CUR_DISTANCE_VALUE = amp_buffer[sizeof(amp_buffer) / 2 - 1];
}

void Radar_MotionUpdate(XMC_RADARSENSE2GOL_MOTION_t motion)
{
	g_motion = motion;
	// check results
	if (motion != XMC_NO_MOTION_DETECT &&			// motion detected
	    g_doppler_velocity > g_min_velocity &&		// doppler velocity is greater than min velocity
		g_doppler_velocity < g_max_velocity)		// doppker velocity is less than max veloctiy
	{
		if (motion == XMC_MOTION_DETECT_APPROACHING)	// target is approaching radar
		{

		}
		else // motion == XMC_MOTION_DETECT_DEPARTING => target is moving away from radar
		{

		}
	}
	else // no motion detected
	{
		DIGITAL_GPIO_SetOutputHigh(&LED_ORANGE);

		// set velocity and frequency to 0 in case of no motion
		g_doppler_frequency = 0.0;
		g_doppler_velocity = 0.0;
		g_motion = XMC_NO_MOTION_DETECT;
	}
}

void Radar_PWMStateUpdate(void)
{
	INT8U  byte_buffer[4];
	INT8U  pwm_point;

	RADAR_GetDistance(byte_buffer);
	get16(byte_buffer);

//	if(get16(byte_buffer) <= 150)
	RADAR_GetPWMTripPoint(&pwm_point);
	if(get16(byte_buffer) <= pwm_point)
	{
//		PWM_SetState(PWM_ENABLE);
		PWM_Start();
	}
	else
	{
//		PWM_SetState(PWM_LOW);
		PWM_Stop();
	}
}

void Radar_ToiletCoverUpdate(void)
{
	INT8U  buffer[5];
	INT16U cur_speed, cur_distance;
	RADAR_TOILET_COVER_Type cover_status;
	RADAR_TOILET_COVER_Type cover_b_state;
	INT8U  cover_point;

	/* toilet main cover control */
	RADAR_GetDistance(buffer);
	cur_distance = get16(buffer);

	RADAR_GetCoverOpenedTripPoint(buffer);
	cover_point = get16(buffer);

	if(cur_distance < cover_point)
	{
		SCH_Add_Flag(Flag_CoverADelay, SCH_FLAG_3000MS);
//		SCH_Add_Flag(Flag_CoverADelay, SCH_FLAG_10000MS);

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

	/* toilet secondary cover control */

	RADAR_GetSpeedInCM(buffer);
	cur_speed = get16(buffer);

	RADAR_GetToiletCoverAStatus(&cover_status);

	if((cur_speed > radar_factory_data.gesture_spd_point)
	&& (cover_status == TOILET_COVER_OPENED)
	&& (cur_distance > radar_factory_data.gesture_dis_point))
	{
//		state = GESTURE_DETECTED;

		RADAR_SetToiletCoverBStatus(TOILET_COVER_OPENED);
	}
}

//API

void RADAR_GetDistance(INT8U *output)
{
	output[0] = WORD_HIGH(CUR_DISTANCE_VALUE);
	output[1] = WORD_LOW(CUR_DISTANCE_VALUE);
}

/*void RADAR_GetSpeed(INT8U *output)
{
	INT16U speed = 0;

	speed = (10 * g_doppler_velocity);

	output[0] = WORD_HIGH(speed);
	output[1] = WORD_LOW(speed);
}*/

void RADAR_GetSpeedInCM(INT8U *output)
{
	INT16U speed = 0;

//	speed = (10 * g_doppler_velocity);

	speed = 1000 * g_doppler_velocity / 36;

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
	output[0] = WORD_HIGH(CUR_AMPLITUDE_VALUE);
	output[1] = WORD_LOW(CUR_AMPLITUDE_VALUE);
}

void RADAR_GetMotion(INT8U *output)
{
	output[0] = g_motion;
}

void RADAR_GetToiletCover(INT8U *output)
{
	RADAR_GetToiletCoverAStatus(output);
}


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

void RADAR_SetCalibrationMode(RADAR_CALIBRATION_MODE_Type mode)
{
	RADAR_CALIBRATION_MODE = mode;
}

void RADAR_GetCalibrationMode(RADAR_CALIBRATION_MODE_Type *mode)
{
	*mode = RADAR_CALIBRATION_MODE;
}

void RADAR_SetAmplitudeCalValue(INT16U amp)
{
	RADAR_CALIBRATION_MODE_Type mode;

	RADAR_GetCalibrationMode(&mode);

	if(mode == FREE_MODE)
	{
		AMPLITUDE_CAL_VALUE = 0;
	}
	else	//CALIBRATION_MODE
	{
		if(amp > AMPLITUDE_CAL_VALUE)
		{
			AMPLITUDE_CAL_VALUE = amp;
		}
	}
}

void RADAR_GetAmplitudeCalValue(INT8U *output)
{
	output[0] = WORD_HIGH(AMPLITUDE_CAL_VALUE);
	output[1] = WORD_LOW(AMPLITUDE_CAL_VALUE);
}

void RADAR_GetPWMTripPoint(INT8U *output)
{
	output[0] = WORD_HIGH(radar_factory_data.pwm_point);
	output[1] = WORD_LOW(radar_factory_data.pwm_point);
}

void RADAR_GetDistanceSpeedOffsetStatus(INT8U *output)
{
	output[0] = radar_factory_data.dis_spd_offset_status;
}

void RADAR_GetCoverOpenedTripPoint(INT8U *output)
{
//	output[0] = radar_factory_data.cover_point;
	output[0] = WORD_HIGH(radar_factory_data.cover_point);
	output[1] = WORD_LOW(radar_factory_data.cover_point);
}

void RADAR_GetSignalStrength(INT8U *output)
{
	INT8U  buffer[4];
	INT16U cur_dis;

	RADAR_GetDistance(buffer);
	cur_dis = get16(buffer);

	if(cur_dis <= 10 * radar_factory_data.cal_table_type.minDistance)
	{
		output[0] = 100;
	}
	else if(cur_dis >= 10 * radar_factory_data.cal_table_type.maxDistance)
	{
		output[0] = 10;
	}
	else
	{
		output[0] = 118 - (9 * cur_dis) / 25;
	}
}

void RADAR_GetDeviceAddress(INT8U *output)
{
	output[0] = radar_user_data.device_address;
}

void RADAR_SetDeviceAddress(INT8U para)
{
	radar_user_data.device_address = para;
}

INT32U RADAR_GetBaudRate(void)
{
	INT32U cur_baud_rate;

	switch(radar_user_data.baud_rate_option)
	{
	case 1:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 2:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 3:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 4:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 5:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 6:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 7:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 8:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 9:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	case 10:
		cur_baud_rate = baud_rate_buffer[radar_user_data.baud_rate_option - 1];
		break;

	default:
		cur_baud_rate = 2400;
		break;
	}

	return cur_baud_rate;
}

