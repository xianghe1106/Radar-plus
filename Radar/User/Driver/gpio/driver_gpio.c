/*
 * led.c
 *
 *  Created on: Dec 17, 2018
 *      Author: xianghe
 */




/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include <gpio/driver_gpio.h>

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



/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void GPIOInit(void);
static void ParaInit(void);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

const DIGITAL_IO_type LED_ORANGE =
{
  .gpio_port = XMC_GPIO_PORT0,
  .gpio_pin = 9U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_type BGT24 =
{
  .gpio_port = XMC_GPIO_PORT0,
  .gpio_pin = 0U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_type LED_RED =
{
  .gpio_port = XMC_GPIO_PORT0,
  .gpio_pin = 7U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_type LED_BLUE =
{
  .gpio_port = XMC_GPIO_PORT0,
  .gpio_pin = 5U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};



void Multi_IO_Init(void)
{
	GPIOInit();
	ParaInit();
}

static void GPIOInit(void)
{
	DIGITAL_GPIO_Init(&LED_ORANGE);
	//  Initialization of DIGITAL_IO APP instance BGT24
	DIGITAL_GPIO_Init(&BGT24);

	//  Initialization of DIGITAL_IO APP instance LED_RED
	DIGITAL_GPIO_Init(&LED_RED);
	//  Initialization of DIGITAL_IO APP instance LED_BLUE
	DIGITAL_GPIO_Init(&LED_BLUE);

	// turn off all leds
	DIGITAL_GPIO_SetOutputHigh(&LED_ORANGE);
	DIGITAL_GPIO_SetOutputHigh(&LED_RED);
	DIGITAL_GPIO_SetOutputHigh(&LED_BLUE);

	// turn on BGT
	DIGITAL_GPIO_SetOutputLow(&BGT24);
}

static void ParaInit(void)
{

}



void DIGITAL_GPIO_Init(const DIGITAL_IO_type *const handler)
{
  XMC_ASSERT("DIGITAL_IO_Init: handler null pointer", handler != NULL);

  /* Initializes input / output characteristics */
  XMC_GPIO_Init(handler->gpio_port, handler->gpio_pin, &handler->gpio_config);

  /*Configure hardware port control*/
  XMC_GPIO_SetHardwareControl(handler->gpio_port, handler->gpio_pin, handler->hwctrl);
}

void DIGITAL_GPIO_SetOutputHigh(const DIGITAL_IO_type *const handler)
{
  XMC_ASSERT("DIGITAL_IO_SetOutputHigh: handler null pointer", handler != NULL);
  XMC_GPIO_SetOutputHigh(handler->gpio_port, handler->gpio_pin);
}

void DIGITAL_GPIO_SetOutputLow(const DIGITAL_IO_type *const handler)
{
  XMC_ASSERT("DIGITAL_IO_SetOutputLow: handler null pointer", handler != NULL);
  XMC_GPIO_SetOutputLow(handler->gpio_port,handler->gpio_pin);
}

void DIGITAL_GPIO_ToggleOutput(const DIGITAL_IO_type *const handler)
{
  XMC_ASSERT("DIGITAL_IO_ToggleOutput: handler null pointer", handler != NULL);
  XMC_GPIO_ToggleOutput(handler->gpio_port, handler->gpio_pin);
}

uint32_t DIGITAL_GPIO_GetInput(const DIGITAL_IO_type *const handler)
{
  XMC_ASSERT("DIGITAL_IO_GetInput: handler null pointer", handler != NULL);
  return XMC_GPIO_GetInput(handler->gpio_port, handler->gpio_pin);
}



