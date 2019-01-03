/**
 * @file rtc.c
 * @date 2015-11-18
 *
 * NOTE:
 * This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
 */
/**
 * @cond
 ***********************************************************************************************************************
 * RTC v4.1.10 Facilitates to generate real time clock and alarm
 *
 * Copyright (c) 2015, Infineon Technologies AG
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
 *     - Initial version<br>
 *
 * 2015-05-18:
 *     - Modified SetTime,SetAlarmTime APIs <br>
 *
 * 2015-06-16:
 *     - GLOBAL_SCU enum elements are passed in GLOBAL_SCU_XMC4_RegisterCallback().<br>
 * 2015-11-18:
 *     - Updated the RTC_lConfigureInterrupts() to enable the NMI request for timer and alarm interrupts.<br>
 * @endcond
 *
 */

/**
* @file rtc.c
*
*
* @brief  Real Time Clock APP Implementation Source File.
*
*
*/

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "rtc.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
/* Constant used for the number of days in an year */
#define RTC_DAYS_IN_AN_YEAR     (365U)
/* Constant used for the seconds in a day */
#define RTC_SECONDS_IN_A_DAY    (24U * 60U * 60U)
/* Constant used for the seconds in an hour */
#define RTC_SECONDS_IN_AN_HOUR  (60U * 60U)
/* Constant used for the seconds in a minute */
#define RTC_SECONDS_IN_A_MINUTE (60U)
/* Constant used for the epoch year */
#define RTC_EPOCH_YEAR          (1970U)

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/
/* Constant array used to store the number of days in each month */
const uint32_t RTC_DAYS_IN_MONTH[13] =
{
  /* Index from 1, hence skip 0*/
  0U,
  /*Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec*/
  31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U
};

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
uint8_t RTC_lleapyear(uint16_t year);
bool RTC_lConfigureInterrupts(const RTC_t *const handler);
XMC_RTC_STATUS_t RTC_lRegister_Callbacks(const RTC_t *const handler);

/**********************************************************************************************************************
 * API IMPLEMENTATION
***********************************************************************************************************************/
/*
 * API to retrieve the version of the RTC APP
 */
/*DAVE_APP_VERSION_t RTC_GetAppVersion(void)
{
  DAVE_APP_VERSION_t version;

  version.major = RTC_MAJOR_VERSION;
  version.minor = RTC_MINOR_VERSION;
  version.patch = RTC_PATCH_VERSION;

  return (version);
}*/

/*
  Initialization function for the APP. Configures the registers
  based on options selected in UI.
*/
RTC_STATUS_t RTC_Init(RTC_t *const handler)
{
  XMC_RTC_STATUS_t status;
  RTC_STATUS_t rtc_initstatus;
  bool interrupt_configured;

  XMC_ASSERT("RTC_Init: NULL Handler", handler != NULL);

  status = XMC_RTC_STATUS_OK;
  rtc_initstatus = RTC_STATUS_FAILURE;

#if (RTC_INTERRUPT_ENABLED == 1)
#if (UC_FAMILY == XMC4)
  rtc_initstatus = (RTC_STATUS_t)GLOBAL_SCU_XMC4_Init(GLOBAL_SCU_HANDLE);
#else
  rtc_initstatus = (RTC_STATUS_t)GLOBAL_SCU_XMC1_Init(GLOBAL_SCU_HANDLE);
#endif
  if (rtc_initstatus == RTC_STATUS_SUCCESS)
  {
#endif
      if (handler->initialized == false)
      {
        /* Initialize the clock source and pre-scalar */
        status = XMC_RTC_Init(handler->time_alarm_config);

        if (status == XMC_RTC_STATUS_OK)
        {
          /* Configure periodic, alarm and hibernate periodic interrupts */
          interrupt_configured = RTC_lConfigureInterrupts(handler);

          if (interrupt_configured == true)
          {
            status = RTC_lRegister_Callbacks(handler);
          }

          if (status == XMC_RTC_STATUS_OK)
          {
          	/* Check RTC start during init is set or not in UI */
            if (handler->config->start == RTC_START_ENABLE)
            {
              RTC_Start();
            }
            handler->initialized = true;
            rtc_initstatus = RTC_STATUS_SUCCESS;
          }
        }
        else
        {
          rtc_initstatus = RTC_STATUS_FAILURE;
        }
      }
      else
      {
        rtc_initstatus = RTC_STATUS_SUCCESS;
      }
#if (RTC_INTERRUPT_ENABLED == 1)
   } /* end of if(rtc_initstatus == GLOBAL_SCU_XMC4_STATUS_OK) */
#endif

  return (rtc_initstatus);
}
/*
 *  This function configures periodic and alarm interrupts
 */
bool RTC_lConfigureInterrupts(const RTC_t *const handler)
{
  uint32_t regval;
  bool interrupt_configured = false;

  /* Enable periodic seconds, minutes, hours days, months and years interrupts */
  regval = (((uint32_t)handler->config->periodic_sec_intr << RTC_MSKSR_MPSE_Pos)
           | ((uint32_t)handler->config->periodic_min_intr << RTC_MSKSR_MPMI_Pos)
           | ((uint32_t)handler->config->periodic_hour_intr << RTC_MSKSR_MPHO_Pos)
           | ((uint32_t)handler->config->periodic_day_intr << RTC_MSKSR_MPDA_Pos)
           | ((uint32_t)handler->config->periodic_month_intr << RTC_MSKSR_MPMO_Pos)
           | ((uint32_t)handler->config->periodic_year_intr << RTC_MSKSR_MPYE_Pos));

  /* Enable RTC periodic interrupt in SCU when any of the periodic interrupts
   * are enabled */
  if (regval != 0U)
  {
    XMC_RTC_EnableEvent(regval);
#if ((UC_FAMILY == XMC4) && (RTC_TIMER_EVENT_TRIG_TO_NMI == 1))
		XMC_SCU_INTERRUPT_EnableEvent((XMC_SCU_INTERRUPT_EVENT_t)XMC_SCU_NMIREQ_RTC_PI);
		XMC_SCU_INTERRUPT_EnableNmiRequest((uint32_t)XMC_SCU_NMIREQ_RTC_PI);
#endif
#if ((UC_FAMILY == XMC4) && (RTC_TIMER_EVENT_TRIG_TO_SCU == 1))
		GLOBAL_SCU_XMC4_EnableEvent((GLOBAL_SCU_XMC4_EVENT_t)GLOBAL_SCU_XMC4_EVENT_RTC_PERIODIC);
#endif
    interrupt_configured = true;
  }


	if (handler->config->alarm_intr == RTC_INT_ALARM_ENABLE)
	{
		XMC_RTC_EnableEvent((uint32_t)XMC_RTC_EVENT_ALARM);
#if ((UC_FAMILY == XMC4) && (RTC_ALARM_EVENT_TRIG_TO_NMI == 1))
		XMC_SCU_INTERRUPT_EnableEvent((XMC_SCU_INTERRUPT_EVENT_t)XMC_SCU_NMIREQ_RTC_AI);
		XMC_SCU_INTERRUPT_EnableNmiRequest((uint32_t)XMC_SCU_NMIREQ_RTC_AI);
#endif
#if ((UC_FAMILY == XMC4) && (RTC_ALARM_EVENT_TRIG_TO_SCU == 1))
		GLOBAL_SCU_XMC4_EnableEvent((GLOBAL_SCU_XMC4_EVENT_t)GLOBAL_SCU_XMC4_EVENT_RTC_ALARM);
#endif

		interrupt_configured = true;
	}


  return (interrupt_configured);
}

/*
 *  Interface to register the RTC call backs
 */
XMC_RTC_STATUS_t RTC_lRegister_Callbacks(const RTC_t *const handler)
{
  XMC_RTC_STATUS_t pi_status;
  XMC_RTC_STATUS_t ai_status;

  pi_status = XMC_RTC_STATUS_OK;
  ai_status = XMC_RTC_STATUS_OK;

#if (RTC_INTERRUPT_ENABLED == 1)
  #if (UC_FAMILY == XMC4)
    pi_status = (XMC_RTC_STATUS_t)GLOBAL_SCU_XMC4_RegisterCallback(
    		(GLOBAL_SCU_XMC4_EVENT_t)GLOBAL_SCU_XMC4_EVENT_RTC_PERIODIC, handler->config->pi_listener);
    if (handler->config->alarm_intr == RTC_INT_ALARM_ENABLE)
    {
      ai_status = (XMC_RTC_STATUS_t)GLOBAL_SCU_XMC4_RegisterCallback(
      		(GLOBAL_SCU_XMC4_EVENT_t)GLOBAL_SCU_XMC4_EVENT_RTC_ALARM,handler->config->ai_listener);
    }
  #else
    pi_status = (XMC_RTC_STATUS_t)GLOBAL_SCU_XMC1_RegisterCallback((GLOBAL_SCU_XMC1_EVENT_t)GLOBAL_SCU_XMC1_EVENT_RTC_PERIODIC,
                                                                   handler->config->pi_listener);

    if (handler->config->alarm_intr == RTC_INT_ALARM_ENABLE)
    {
      ai_status = (XMC_RTC_STATUS_t)GLOBAL_SCU_XMC1_RegisterCallback((GLOBAL_SCU_XMC1_EVENT_t)GLOBAL_SCU_XMC1_EVENT_RTC_ALARM,
                                                                     handler->config->ai_listener);
    }
  #endif
#endif

  return (XMC_RTC_STATUS_t)((uint32_t)pi_status & (uint32_t)ai_status);
}

/*
 *  This function is used to set RTC time.
 */
RTC_STATUS_t RTC_SetTime(XMC_RTC_TIME_t *current_time)
{
  RTC_STATUS_t status = RTC_STATUS_SUCCESS;
  XMC_RTC_TIME_t time_val;

  XMC_ASSERT("RTC_SetTime: NULL pointer", current_time != NULL);

  /* copy to local structure to keep data safe */
  time_val.year = current_time->year;
  time_val.month = current_time->month;
  time_val.days = current_time->days;
  time_val.hours = current_time->hours;
  time_val.minutes = current_time->minutes;
  time_val.seconds = current_time->seconds;

  if ((time_val.days != 0U) && (time_val.month != 0U))
  {
    time_val.days = time_val.days - 1U;
    time_val.month = time_val.month - 1U;

    XMC_RTC_SetTime(&time_val);
  }
  else
  {
    status = RTC_STATUS_FAILURE;
  }

  return (status);
}

/*
 *  This function is used to get RTC time.
 */
void RTC_GetTime(XMC_RTC_TIME_t *current_time)
{
  XMC_ASSERT("RTC_GetTime: NULL pointer", current_time != NULL);

  XMC_RTC_GetTime(current_time);

  current_time->days = current_time->days + 1U;
  current_time->month = current_time->month + 1U;
}

/*
 *  This function is used to set Alarm time.
 */
RTC_STATUS_t RTC_SetAlarmTime(XMC_RTC_ALARM_t *alarm)
{
  RTC_STATUS_t status = RTC_STATUS_SUCCESS;
  XMC_RTC_ALARM_t alarm_val;

  XMC_ASSERT("RTC_SetAlarmTime: NULL pointer", alarm != NULL);

  /* copy to local structure to keep data safe */
  alarm_val.year = alarm->year;
  alarm_val.month = alarm->month;
  alarm_val.days = alarm->days;
  alarm_val.hours = alarm->hours;
  alarm_val.minutes = alarm->minutes;
  alarm_val.seconds = alarm->seconds;

  if ((alarm_val.days != 0U) && (alarm_val.month != 0U))
  {
    alarm_val.days = alarm_val.days - 1U;
    alarm_val.month = alarm_val.month - 1U;

    XMC_RTC_SetAlarm(&alarm_val);
  }
  else
  {
    status = RTC_STATUS_FAILURE;
  }

  return (status);
}

/*
 *  This function is used to get Alarm time from XMC.
 *  And returns in standard time format.
 */
void RTC_GetAlarmTime(XMC_RTC_ALARM_t *alarm)
{
  XMC_ASSERT("RTC_GetAlarmTime: NULL pointer", alarm != NULL);

  XMC_RTC_GetAlarm(alarm);

  alarm->days = alarm->days + 1U;
  alarm->month = alarm->month + 1U;
}

/*
 *  This function is used to get event status.
 */
uint32_t RTC_GetFlagStatus(void)
{
  uint32_t event_status;

  event_status = XMC_RTC_GetEventStatus();

  return (event_status);
}

/*
 *  This function is to get the time in seconds calculated from Epoch time
 *  (01/01/1970).
 */
RTC_STATUS_t RTC_Time(time_t* time_value)
{
  uint32_t elapsedyear;
  uint32_t elapsedmonth;
  uint32_t elapseddays;
  uint32_t elapsedseconds;

  RTC_STATUS_t status;
  XMC_RTC_TIME_t curr_time;
  
  XMC_ASSERT("RTC_Time: NULL pointer", time_value != NULL);

  /*Check if RTC module is enabled and no NULL pointer*/
  if (true == XMC_RTC_IsRunning())
  {
    /* Read values from TIM0 and TIM1 registers */
    XMC_RTC_GetTime(&curr_time);

    /*Count number of Days for Elapsed Years since Epoch*/
    elapseddays = ((uint32_t)curr_time.year - RTC_EPOCH_YEAR) * RTC_DAYS_IN_AN_YEAR;

    /* Add the number of days to be adjusted for leap years,
       start from previous year and check backward */
    for (elapsedyear = ((uint32_t)curr_time.year - 1U); elapsedyear>= (uint32_t)1970; elapsedyear--)
    {
      if (RTC_lleapyear((uint16_t)elapsedyear))
      {
        elapseddays++;
      }
    }
    /*If current year is leap year add 1 only if March or later*/
    if (RTC_lleapyear(curr_time.year))
    {
      if(curr_time.month > 2U)
      {
        elapseddays++;
      }
    }

    /*Add number of Days from Elapsed months from current year*/
    for (elapsedmonth = (curr_time.month); elapsedmonth != 0U; elapsedmonth--)
    {
      elapseddays += RTC_DAYS_IN_MONTH[elapsedmonth];
    }

    /*Add Elapsed days from current month*/
    elapseddays += curr_time.days;

    /*Accumulate the total seconds for ElapsedDays*/
    elapsedseconds = (elapseddays * RTC_SECONDS_IN_A_DAY);

    /*Add seconds for current hour, minute and seconds*/
    elapsedseconds += ((uint32_t)curr_time.hours * RTC_SECONDS_IN_AN_HOUR);
    elapsedseconds += ((uint32_t)curr_time.minutes * RTC_SECONDS_IN_A_MINUTE);
    elapsedseconds += (uint32_t)curr_time.seconds;

    *time_value = (time_t)elapsedseconds;
     status = RTC_STATUS_SUCCESS;
  }
  else
  {
    status = RTC_STATUS_FAILURE;
  }
  return (status);
}
	
/* This function returns 1 if it is leap year otherwise 0.*/
uint8_t RTC_lleapyear(uint16_t year)
{
  uint8_t valid = 0U;

  if ((((year) % 400U) == 0U) || ((((year) % 100U) != 0U) &&
       (((year) %4U) == 0U)))
  {
    valid = 1U;
  }
  return (valid);
}
