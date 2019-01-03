/**
    @file: config.h

    Configuration file for Sense2GoL project.
*/

/* ===========================================================================
** Copyright (C) 2012-2017 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorisation.
** ===========================================================================
*/

#ifndef CONFIG_H
#define CONFIG_H

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

/**
 * @defgroup grp_template This is a template.
 * @ingroup grp_wher_this_module_belongs_to
 * @brief Configuration file for Sense2GoL project
 */

///@addtogroup grp_template
///@{

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

#define SAMPLING_FREQ_HZ (3000) 	/**< sampling frequency in Hz.
									PLEASE CHANGE ACCORDING TO YOUR
									REQUIREMENTS!!! */
#define DETECTION_THRESHOLD (200) 	/**< threshold of fft magnitude. all
 	 	 	 	 	 	 	 	 	magnitudes below this value will be
 	 	 	 	 	 	 	 	 	ignored.
 	 	 	 	 	 	 	 	 	PLEASE CHANGE ACCORDING TO YOUR
 	 	 	 	 	 	 	 	 	REQUIREMENTS!!! */
#define FFT_SIZE (128)    			/**< number of fft points */
#define UART_RAW_DATA (1) 			/**control for execution of UART feature to
									transmit raw IQ from ADC. BE CAREFUL: WHEN
									SENDING RAW DATA VIA UART INTERFACE GUI
									WILL BE VERY SLOW AND NOT USABLE.TURN OFF
									WHEN USING GUI */
#define MOTION_MIN_VELOCITY (0.5) 	/**< min velocity to detect motion */
#define MOTION_MAX_VELOCITY (20) 	/**< max velocity to detect motion */

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/*
==============================================================================
   4. EXPORTED DATA
==============================================================================
*/

/*
==============================================================================
   5. FUNCTION PROTOTYPES AND INLINE FUNCTIONS
==============================================================================
*/

///@}

#endif

/* --- End of File ------------------------------------------------ */

