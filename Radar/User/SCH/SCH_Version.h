#ifndef _SCH_VERSION_H
#define _SCH_VERSION_H


#define SCH_VERSION						"1.4"

/*
*********************************************************************************************************
*    
**Created by:		Xianghe
**Date:	   			2018/05/17
**Revision:			V1.4
					a. SCH_RTC: SCH_RTC_Type add INT16U msec;
					b. Function SCH_RTC_Init ,SCH_RTC_Update and SCH_Get_RTC are updated;
					
Legacy issues:		
					a. RT_APP: SCH_Task
					b. SCH_Queue: Optimize the pointer operation
					c. uart: Queues
					
---------------------------------------------------------------------------------------------------------

/*
*********************************************************************************************************
*    
**Created by:		Xianghe
**Date:	   			2017/12/20
**Revision:			V1.3
					a. delete SCH_Msg.c
					b. SCH_RTC: the data type of schRTC changed, INT16U hour changed to INT32U hour.
					c. delete the codes which disable the system interrupt, it may cause uart and other 
					   module data loss.
						i. CPU_SR_ALLOC();
						   SCH_CRITICAL_ENTER();
						   SCH_CRITICAL_EXIT();
						   
						ii. CPU_SR_ALLOC();
							CPU_CRITICAL_ENTER();
							CPU_CRITICAL_EXIT();
					
---------------------------------------------------------------------------------------------------------

**Created by:		Xianghe
**Date:	   			2017/06/28
**Revision:			V1.2
					a. add SCH_Version.h to record change log.
					b. add SCH_Queue.c and SCH_Queue.h.
					c. SCH_Msg.c update
					d. add SCH_cfg.h
					
---------------------------------------------------------------------------------------------------------

**Created by:		Xianghe
**Date:	   			2016/06/24
**Revision:			V1.1
					a. add APP_TASK_Update()
					b. add APP_Time_Update()
					
---------------------------------------------------------------------------------------------------------

Modify records
**Created by:		Xianghe
**Date:	   			2014/06/18
**Revision:			V1.0	

					
*********************************************************************************************************
*/














#endif



