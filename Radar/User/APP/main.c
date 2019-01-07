/**
 * \file main.c
 * \brief This file implements the top-level functionality for the motion detection on Sense2GoL radar board.
 * UART feature is also used in this implementation to dump IQ raw data samples to the HOST.
 * UART Configurations: Full-duplex, Direct mode, 9600 baud rate, 8 data-bits, 1 stop-bit, no parity
 * Data format of transmission: Completely transmits I_adc samples of buffer length BUFF_SIZE,
 * followed by Q_adc samples of same buffer length of BUFF_SIZE so in total 2 * BUFF_SIZE samples are transmitted
 *
 * \author Assad Ali
 *
 * \cond
 * ********************************************************************************************
 * Copyright (c) 2017, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (thomas.finke@infineon.com).
 * ********************************************************************************************
 * \endcond
 *
 */

/****************************************************************
 * HEADER FILES
 ***************************************************************/
#include <driver_DAVE.h>
#include <DAVE.h>
#include "radarsense2gol_library.h"
//#include "HostCommUART.h"
#include "config.h"

#include "SCH_Core.h"
#include "radar.h"
#include "Protocol.h"
#include "Bsp.h"

#include "driver.h"

/**********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/



/************************************************************************************************************/

/*!
 * \brief Top-level function for the motion detection using radar Sense2GoL board
 */

void TASK_uart(void);

int main(void)
{
//	DAVE_Init();
	Driver_DAVE_Init();  //Initialization of DAVE APPs
//	DAVE_Init();

	BSP_IntDis();

	SystemInit();

	BSP_HardwareInit();

	Protocol_init();

	SCH_Init();

	/* Add Task */
	SCH_Add_Task(RADAR_Test 					, 		1  , 		100  );

	SCH_Add_Task(Protocol_process 				, 		2  , 		10   );//RADAR_TestTime

	SCH_Add_Task(Protocol_heart_beat 			, 		10 , 		8   );//Protocol_heart_beat

	SCH_Start();


	while (1)
	{
		SCH_Dispatch_Tasks();
		Protocol_preprocessing();
//		Protocol_process();
		RADAR_Process();
	}

}





