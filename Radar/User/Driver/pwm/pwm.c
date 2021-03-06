/*
 * pwm.c
 *
 *  Created on: Jan 14, 2019
 *      Author: xianghe
 */




/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "pwm.h"
#include "PWM_CCU8/pwm_ccu8.h"

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

//PWM_SRARUS_Type PWM_STATE;

//bool pwm_state = false;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

//static void GPIOInit(void);
//static void ParaInit(void);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

void PWM_Start(void)
{
//	if(pwm_state == false)
//	{
//		pwm_state = true;
		PWM_CCU8_Start(&PWM_CCU8_0);
//	}
}

void PWM_Stop(void)
{
//	if(pwm_state == true)
//	{
//		pwm_state = false;
		PWM_CCU8_Stop(&PWM_CCU8_0);
//	}
}

/*

const DIGITAL_IO_type PWM_PORT =
{
  .gpio_port = XMC_GPIO_PORT0,
  .gpio_pin = 8U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};


void PWM_Init(void)
{
	GPIOInit();
	ParaInit();
}

static void GPIOInit(void)
{
	DIGITAL_GPIO_Init(&PWM_PORT);

	DIGITAL_GPIO_SetOutputLow(&PWM_PORT);
}

static void ParaInit(void)
{
	PWM_STATE = PWM_LOW;
}

void PWM_Update(void)
{
	PWM_SRARUS_Type state;

	INT8U  buffer[2];
	INT16U cur_distance;

	RADAR_GetDistance(buffer);
	cur_distance = get16(buffer);

	if(cur_distance <= 150)
	{
		PWM_STATE = PWM_ENABLE;
	}
	else
	{
		PWM_STATE = PWM_LOW;
	}

	PWM_GetState(&state);

	if(state == PWM_LOW)
	{
		DIGITAL_GPIO_SetOutputLow(&PWM_PORT);
	}
	else //PWM_ENABLE
	{
		DIGITAL_GPIO_ToggleOutput(&PWM_PORT);
	}
}

//API

void PWM_SetState(PWM_SRARUS_Type state)
{
	PWM_STATE = state;
}

void PWM_GetState(PWM_SRARUS_Type *state)
{
	*state = PWM_STATE;
}
*/



