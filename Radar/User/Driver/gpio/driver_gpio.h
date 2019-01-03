/*
 * led.h
 *
 *  Created on: Dec 17, 2018
 *      Author: xianghe
 */

#ifndef USER_DRIVER_GPIO_H_
#define USER_DRIVER_GPIO_H_



/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include <driver_DAVE.h>
#include "cpu.h"
#include "xmc_gpio.h"
//#include "DIGITAL_IO/digital_io.h"

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
	XMC_GPIO_PORT_t *const gpio_port;             /**< port number */
	const XMC_GPIO_CONFIG_t gpio_config;          /**< mode, initial output level and pad driver strength / hysteresis */
	const uint8_t gpio_pin;                       /**< pin number */
	const XMC_GPIO_HWCTRL_t hwctrl;               /**< Hardware port control */
}DIGITAL_IO_type;

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

extern const DIGITAL_IO_type LED_ORANGE;
extern const DIGITAL_IO_type BGT24;
extern const DIGITAL_IO_type LED_RED;
extern const DIGITAL_IO_type LED_BLUE;

/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/




/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void Multi_IO_Init(void);

void DIGITAL_GPIO_Init(const DIGITAL_IO_type *const handler);
void DIGITAL_GPIO_SetOutputHigh(const DIGITAL_IO_type *const handler);
void DIGITAL_GPIO_SetOutputLow(const DIGITAL_IO_type *const handler);
void DIGITAL_GPIO_ToggleOutput(const DIGITAL_IO_type *const handler);
uint32_t DIGITAL_GPIO_GetInput(const DIGITAL_IO_type *const handler);



/*
*********************************************************************************************************
*                                               END
*********************************************************************************************************
*/

#endif /* USER_DRIVER_LED_GPIO_H_ */
