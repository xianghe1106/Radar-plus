/*
 * driver_uart.c
 *
 *  Created on: Dec 11, 2018
 *      Author: xianghe
 */




/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "driver_uart.h"
#include "xmc_gpio.h"
#include "xmc_uart.h"

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



/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


void Driver_uart_init(void)
{
	XMC_UART_CH_CONFIG_t uart_config;
	XMC_GPIO_CONFIG_t rx_pin_config;
	XMC_GPIO_CONFIG_t tx_pin_config;

	/* Configure RX pin */
	rx_pin_config.mode             = XMC_GPIO_MODE_INPUT_TRISTATE;
	rx_pin_config.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH;
	rx_pin_config.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;
	XMC_GPIO_Init(XMC_GPIO_PORT2, 6, &rx_pin_config);


	/* Configure TX pin */
	tx_pin_config.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT6;
	tx_pin_config.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH;
	XMC_GPIO_Init(XMC_GPIO_PORT2, 0, &tx_pin_config);

	/* UART configuration */
	uart_config.baudrate      = UART_BAUDRATE;
	uart_config.data_bits     = 8U;
	uart_config.frame_length  = 8U;
	uart_config.stop_bits     = 1U;
	uart_config.oversampling  = 16U;
	uart_config.parity_mode   = XMC_USIC_CH_PARITY_MODE_NONE;

	/* Configure UART channel */
	XMC_UART_CH_Init(XMC_UART0_CH0, &uart_config);

	XMC_UART_CH_SetInputSource(XMC_UART0_CH0, XMC_UART_CH_INPUT_RXD1, USIC0_C0_DX3_P2_6);

	XMC_USIC_CH_SetInputSource(XMC_UART0_CH0, XMC_USIC_CH_INPUT_DX0, 6U);
	XMC_USIC_CH_SetInputSource(XMC_UART0_CH0, XMC_USIC_CH_INPUT_DX3, 4U);
	XMC_USIC_CH_SetInputSource(XMC_UART0_CH0, XMC_USIC_CH_INPUT_DX5, 0U);

	/* Start UART channel */
	XMC_UART_CH_Start(XMC_UART0_CH0);

	XMC_USIC_CH_EnableEvent(XMC_UART0_CH0, XMC_USIC_CH_EVENT_STANDARD_RECEIVE); // enable interrupt RI


	XMC_UART_CH_EnableEvent(XMC_UART0_CH0, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
	XMC_USIC_CH_SetInterruptNodePointer(XMC_UART0_CH0,
				XMC_USIC_CH_INTERRUPT_NODE_POINTER_RECEIVE, 1);

	/* enable receive interrupt */
	NVIC_SetPriority(USIC0_1_IRQn,2);// CMSIS function for NVIC control: NVIC_SetPriority(IRQn_t IRQn, uint32_t priority): priority=0..63
	NVIC_EnableIRQ(USIC0_1_IRQn);    // CMSIS function for NVIC control: enable IRQn
}


/*
void IRQ_Hdlr_10(void)
{
	 uint16_t received_data;
	 received_data = XMC_UART_CH_GetReceivedData(XMC_UART0_CH0);  // receive data
	  XMC_UART_CH_Transmit(XMC_UART0_CH0, received_data);

	DIGITAL_IO_ToggleOutput(&LED_BLUE);
}
*/


