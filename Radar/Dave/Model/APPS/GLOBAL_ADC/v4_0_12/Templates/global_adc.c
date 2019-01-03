/**
 * @file global_adc.c
 * @date 2015-03-18
 *
 * NOTE:
 * This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
 *
 * @cond
 ***********************************************************************************************************************
 * GLOBAL_ADC v4.0.12 - Initializes VADC GLOBAL and GROUP resources.
 *
 * Copyright (c) 2016, Infineon Technologies AG
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
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * Change History
 * --------------
 *
 * 2015-02-16:
 *     - Initial version for DAVEv4.<BR>
 *
 * 2015-04-27:
 *     - Configuration structure modified.<BR>
 *     - GLOBAL_ADC_group_t changed to GLOBAL_ADC_GROUP_t.<BR>
 *
 * 2015-06-20:
 *     - Updated the copyright section.<BR>
 *
 * 2015-09-01:
 *     - Added support for XMC14 and XMC48/47.<BR>
 *
 * 2015-10-01:
 *     - Analog clock default value changed to 26Mhz in XMC42.<BR>
 *
 * 2015-10-08:
 *     - Added support for XMC4700/XMC4800 devices.<BR>
 *
 * 2015-12-03:
 *     - Added support for XMC4300 devices.<BR>
 *     - Optimized the manifest.<BR>
 *
 * 2016-03-18:
 *     -Modified the minimum configurable value for Desired analog clock in XMC1x to 1MHz.<BR>
 *
 * @endcond
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

/** Inclusion of header file */
#include "global_adc.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

 /**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/

/*This function returns the version of the GLOBAL_ADC APP*/
DAVE_APP_VERSION_t GLOBAL_ADC_GetAppVersion(void)
{
  DAVE_APP_VERSION_t version;

  version.major = (uint8_t) GLOBAL_ADC_MAJOR_VERSION;
  version.minor = (uint8_t) GLOBAL_ADC_MINOR_VERSION;
  version.patch = (uint8_t) GLOBAL_ADC_PATCH_VERSION;

  return version;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * This function initializes all instances of the ADC Global APP and low level app.
 */
GLOBAL_ADC_STATUS_t GLOBAL_ADC_Init(GLOBAL_ADC_t *const handle_ptr)
{
  XMC_ASSERT("GLOBAL_ADC_Init:Invalid handle_ptr", (handle_ptr != NULL))
#if (XMC_VADC_GROUP_AVAILABLE == 1U)
  uint32_t group_index;
#endif

  if (GLOBAL_ADC_UNINITIALIZED == handle_ptr->init_state)
  {  
    /* Initialize an instance of Global hardware */
    XMC_VADC_GLOBAL_Init(handle_ptr->module_ptr, handle_ptr->global_config_handle);

    /* Initialize all the Groups */
#if (XMC_VADC_GROUP_AVAILABLE == 1U)
    for(group_index = (uint32_t)0; group_index < XMC_VADC_MAXIMUM_NUM_GROUPS; group_index++)
    {
      /*Initialize Group*/
      XMC_VADC_GROUP_Init(handle_ptr->group_ptrs_array[group_index]->group_handle,
    		            handle_ptr->group_ptrs_array[group_index]->group_config_handle);

      /* Switch on the converter of the Group[group_index]*/
      XMC_VADC_GROUP_SetPowerMode(handle_ptr->group_ptrs_array[group_index]->group_handle,
                                  XMC_VADC_GROUP_POWERMODE_NORMAL);

      /* Disable the post calibration option for the respective group*/
      if ((bool)false == handle_ptr->group_ptrs_array[group_index]->post_calibration)
      {
        XMC_VADC_GLOBAL_DisablePostCalibration(handle_ptr->module_ptr,group_index);
      }

#if(XMC_VADC_SHS_AVAILABLE == 1U)
      XMC_VADC_GLOBAL_SHS_EnableAcceleratedMode(handle_ptr->global_shs_ptr, (XMC_VADC_GROUP_INDEX_t)group_index);
#endif

      handle_ptr->group_ptrs_array[group_index]->state = GLOBAL_ADC_SUCCESS;
    }
#endif /* _XMC_VADC_GROUP_AVAILABLE_ */
    if((bool)true == handle_ptr->enable_startup_calibration)
    {
    	XMC_VADC_GLOBAL_StartupCalibration(handle_ptr->module_ptr);
    }
    handle_ptr->init_state = GLOBAL_ADC_SUCCESS;
  }
  return (handle_ptr->init_state);
}
