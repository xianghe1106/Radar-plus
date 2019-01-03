/*
*********************************************************************************************************
*                                                
*
* File    : mem.c
* By      : XH
* Version : V1.0
*
* Comments:
* ---------------
*     
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "mem.h"


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
*                                              Mem_Clr()
*
* Description : Clear data buffer (see Note #2).
*
* Argument(s) : pmem        Pointer to memory buffer to clear.
*
*               size        Number of data buffer octets to clear.
*
* Return(s)   : none.
*
* Caller(s)   : various.
*
* Note(s)     : (1) Null clears allowed (i.e. 0-octet size).
*
*                   See also 'Mem_Set()  Note #1'.
*
*               (2) Clear data by setting each data octet to 0.
*********************************************************************************************************
*/

void MEM_Clr (void *pmem, CPU_SIZE_T size)
{
    MEM_Set((void *)pmem, (INT8U)0, (CPU_SIZE_T)size);				/* See Note #2.*/
}


/*
*********************************************************************************************************
*                                              Mem_Set()
*
* Description : Fill data buffer with specified data octet.
*
* Argument(s) : pmem        Pointer to memory buffer to fill with specified data octet.
*
*               data_val    Data fill octet value.
*
*               size        Number of data buffer octets to fill.
*
* Return(s)   : none.
*
* Caller(s)   : various.
*
* Note(s)     : (1) Null sets allowed (i.e. 0-octet size).
*
*               (2) For best CPU performance, optimized to fill data buffer using 'CPU_ALIGN'-sized data words.
*
*                   (a) Since many word-aligned processors REQUIRE that multi-octet words be accessed on 
*                       word-aligned addresses, 'CPU_ALIGN'd words MUST be accessed on 'CPU_ALIGN'd addresses.
*
*               (3) Modulo arithmetic is used to determine whether a memory buffer starts on a 'CPU_ALIGN'
*                   address boundary.
*
*                   Modulo arithmetic in ANSI-C REQUIREs operations performed on integer values.  Thus, 
*                   address values MUST be cast to an appropriately-sized integer value PRIOR to any
*                   mem_align_modulo arithmetic operation.
*********************************************************************************************************
*/

void MEM_Set (void *pmem, INT8U data_val, CPU_SIZE_T size)
{
    CPU_SIZE_T  size_rem;
    CPU_ALIGN   data_align;
    CPU_ALIGN   *pmem_align;
    INT8U  		*pmem_08;
    INT8U   	mem_align_modulo;
    INT8U   	i;


    if (size < 1) {                                             /* See Note #1.                                         */
        return;
    }
    if (pmem == (void *)0) {
        return;
    }


    data_align = 0;
    for (i = 0; i < sizeof(CPU_ALIGN); i++) {                   /* Fill each data_align octet with data val.            */
        data_align <<=  8;
        data_align  |= (CPU_ALIGN)data_val;
    }

    size_rem         = (CPU_SIZE_T)size;
    mem_align_modulo = (INT8U)((CPU_ADDR)pmem % sizeof(CPU_ALIGN));    /* See Note #3.                             */

    pmem_08 = (INT8U *)pmem;
    if (mem_align_modulo != 0) {                                /* If leading octets avail,                   ...       */
        i = mem_align_modulo;
        while ((size_rem > 0) &&                                /* ... start mem buf fill with leading octets ...       */
               (i        < sizeof(CPU_ALIGN ))) {               /* ... until next CPU_ALIGN word boundary.              */
           *pmem_08++ = data_val;
            size_rem -= sizeof(INT8U);
            i++;
        }
    }

    pmem_align = (CPU_ALIGN *)pmem_08;                          /* See Note #2a.                                        */
    while (size_rem >= sizeof(CPU_ALIGN)) {                     /* While mem buf aligned on CPU_ALIGN word boundaries,  */
       *pmem_align++ = data_align;                              /* ... fill mem buf with    CPU_ALIGN-sized data.       */
        size_rem    -= sizeof(CPU_ALIGN);
    }

    pmem_08 = (INT8U *)pmem_align;
    while (size_rem > 0) {                                      /* Finish mem buf fill with trailing octets.            */
       *pmem_08++   = data_val;
        size_rem   -= sizeof(INT8U);
    }
}

/*
*********************************************************************************************************
*                                             Mem_Copy()
*
* Description : Copy data octets from one buffer to another buffer.
*
* Argument(s) : pdest       Pointer to destination memory buffer.
*
*               psrc        Pointer to source      memory buffer.
*
*               size        Number of data buffer octets to copy.
*
* Return(s)   : none.
*
* Caller(s)   : various.
*
* Note(s)     : (1) Null copies allowed (i.e. 0-octet size).
*
*               (2) Memory buffers NOT checked for overlapping.
*
*               (3) For best CPU performance, optimized to fill data buffer using 'CPU_ALIGN'-sized data words.
*
*                   (a) Since many word-aligned processors REQUIRE that multi-octet words be accessed on 
*                       word-aligned addresses, 'CPU_ALIGN'd words MUST be accessed on 'CPU_ALIGN'd addresses.
*
*               (4) Modulo arithmetic is used to determine whether a memory buffer starts on a 'CPU_ALIGN'
*                   address boundary.
*
*                   Modulo arithmetic in ANSI-C REQUIREs operations performed on integer values.  Thus, 
*                   address values MUST be cast to an appropriately-sized integer value PRIOR to any
*                   mem_align_modulo arithmetic operation.
*********************************************************************************************************
*/
void MEM_Copy (void *pdest, void *psrc, CPU_SIZE_T size)
{
    CPU_SIZE_T   size_rem;
    CPU_ALIGN   *pmem_align_dest;
    CPU_ALIGN   *pmem_align_src;
    INT8U   	*pmem_08_dest;
    INT8U   	*pmem_08_src;
    INT8U    	i;
    INT8U    	mem_align_modulo_dest;
    INT8U    	mem_align_modulo_src;
    BOOLEAN   	mem_aligned;


    if (size < 1) {                                             /* See Note #1.                                         */
        return;
    }
    if (pdest == (void *)0) {
        return;
    }
    if (psrc  == (void *)0) {
        return;
    }


    size_rem              = (CPU_SIZE_T  )size;

    pmem_08_dest          = (INT8U *)pdest;
    pmem_08_src           = (INT8U *)psrc;
                                                                /* See Note #4.                                         */
    mem_align_modulo_dest = (INT8U  )((CPU_ADDR)pmem_08_dest % sizeof(CPU_ALIGN));
    mem_align_modulo_src  = (INT8U  )((CPU_ADDR)pmem_08_src  % sizeof(CPU_ALIGN));

    mem_aligned           = (mem_align_modulo_dest == mem_align_modulo_src) ? DEF_YES : DEF_NO;

    if (mem_aligned == DEF_YES) {                               /* If mem bufs' alignment offset equal, ...             */
                                                                /* ... optimize copy for mem buf alignment.             */
        if (mem_align_modulo_dest != 0) {                       /* If leading octets avail,                   ...       */
            i = mem_align_modulo_dest;
            while ((size_rem   >  0) &&                         /* ... start mem buf copy with leading octets ...       */
                   (i          <  sizeof(CPU_ALIGN ))) {        /* ... until next CPU_ALIGN word boundary.              */
               *pmem_08_dest++ = *pmem_08_src++;
                size_rem      -=  sizeof(INT8U);
                i++;
            }
        }

        pmem_align_dest = (CPU_ALIGN *)pmem_08_dest;            /* See Note #3a.                                        */
        pmem_align_src  = (CPU_ALIGN *)pmem_08_src;
        while (size_rem      >=  sizeof(CPU_ALIGN)) {           /* While mem bufs aligned on CPU_ALIGN word boundaries, */
           *pmem_align_dest++ = *pmem_align_src++;              /* ... copy psrc to pdest with CPU_ALIGN-sized words.   */
            size_rem         -=  sizeof(CPU_ALIGN);
        }

        pmem_08_dest = (INT8U *)pmem_align_dest;
        pmem_08_src  = (INT8U *)pmem_align_src;
    }

    while (size_rem > 0) {                                      /* For unaligned mem bufs or trailing octets, ...       */
       *pmem_08_dest++ = *pmem_08_src++;                        /* ... copy psrc to pdest by octets.                    */
        size_rem      -=  sizeof(INT8U);
    }
}

/*
*********************************************************************************************************
*                                              Mem_Cmp()
*
* Description : Verify that ALL data octets in two memory buffers are identical in sequence.
*
* Argument(s) : p1_mem      Pointer to first  memory buffer.
*
*               p2_mem      Pointer to second memory buffer.
*
*               size        Number of data buffer octets to compare.
*
* Return(s)   : DEF_YES, if 'size' number of data octets are identical in both memory buffers.
*
*               DEF_NO,  otherwise.
*
* Caller(s)   : various.
*
* Note(s)     : (1) Null compares allowed (i.e. 0-octet size); 'DEF_YES' returned to indicate identical 
*                   null compare.
*
*               (2) Many memory buffer comparisons vary ONLY in the least significant octets -- e.g. 
*                   network address buffers.  Consequently, memory buffer comparison is more efficient 
*                   if the comparison starts from the end of the memory buffers which will abort sooner 
*                   on dissimilar memory buffers that vary only in the least significant octets.
*
*               (3) For best CPU performance, optimized to fill data buffer using 'CPU_ALIGN'-sized data words.
*
*                   (a) Since many word-aligned processors REQUIRE that multi-octet words be accessed on 
*                       word-aligned addresses, 'CPU_ALIGN'd words MUST be accessed on 'CPU_ALIGN'd addresses.
*
*               (4) Modulo arithmetic is used to determine whether a memory buffer starts on a 'CPU_ALIGN'
*                   address boundary.
*
*                   Modulo arithmetic in ANSI-C REQUIREs operations performed on integer values.  Thus, 
*                   address values MUST be cast to an appropriately-sized integer value PRIOR to any
*                   mem_align_modulo arithmetic operation.
********************************************************************************************************
*/
BOOLEAN MEM_Cmp (void *p1_mem, void *p2_mem, CPU_SIZE_T size)
{
    CPU_SIZE_T  size_rem;
    CPU_ALIGN   *p1_mem_align;
    CPU_ALIGN   *p2_mem_align;
    INT8U   	*p1_mem_08;
    INT8U   	*p2_mem_08;
    INT8U    	i;
    INT8U    	mem_align_modulo_1;
    INT8U    	mem_align_modulo_2;
    BOOLEAN   	mem_aligned;
    BOOLEAN   	mem_cmp;


    if (size < 1) {                                             /* See Note #1.                                         */
        return (DEF_YES);
    }
    if (p1_mem == (void *)0) {
        return (DEF_NO);
    }
    if (p2_mem == (void *)0) {
        return (DEF_NO);
    }


    mem_cmp            =  DEF_YES;
    size_rem           =  size;
                                                                /* Start @ end of mem bufs (see Note #2).               */
    p1_mem_08          = (INT8U *)p1_mem + size;
    p2_mem_08          = (INT8U *)p2_mem + size;
                                                                /* See Note #4.                                         */
    mem_align_modulo_1 = (INT8U  )((CPU_ADDR)p1_mem_08 % sizeof(CPU_ALIGN));
    mem_align_modulo_2 = (INT8U  )((CPU_ADDR)p2_mem_08 % sizeof(CPU_ALIGN));

    mem_aligned        = (mem_align_modulo_1 == mem_align_modulo_2) ? DEF_YES : DEF_NO;

    if (mem_aligned == DEF_YES) {                               /* If mem bufs' alignment offset equal, ...             */
                                                                /* ... optimize cmp for mem buf alignment.              */
        if (mem_align_modulo_1 != 0) {                          /* If trailing octets avail,                  ...       */
            i = mem_align_modulo_1;
            while ((mem_cmp == DEF_YES) &&                      /* ... cmp mem bufs while identical &         ...       */
                   (size_rem > 0)       &&                      /* ... start mem buf cmp with trailing octets ...       */
                   (i        > 0)) {                            /* ... until next CPU_ALIGN word boundary.              */
                p1_mem_08--;
                p2_mem_08--;
                if (*p1_mem_08 != *p2_mem_08) {                 /* If ANY data octet(s) NOT identical, cmp fails.       */
                     mem_cmp = DEF_NO;
                }
                size_rem -= sizeof(INT8U);
                i--;
            }
        }

        if (mem_cmp == DEF_YES) {                               /* If cmp still identical, cmp aligned mem bufs.        */
            p1_mem_align = (CPU_ALIGN *)p1_mem_08;              /* See Note #3a.                                        */
            p2_mem_align = (CPU_ALIGN *)p2_mem_08;

            while ((mem_cmp  == DEF_YES) &&                     /* Cmp mem bufs while identical & ...                   */
                   (size_rem >= sizeof(CPU_ALIGN))) {           /* ... mem bufs aligned on CPU_ALIGN word boundaries.   */
                p1_mem_align--;
                p2_mem_align--;
                if (*p1_mem_align != *p2_mem_align) {           /* If ANY data octet(s) NOT identical, cmp fails.       */
                     mem_cmp = DEF_NO;
                }
                size_rem -= sizeof(CPU_ALIGN);
            }

            p1_mem_08 = (INT8U *)p1_mem_align;
            p2_mem_08 = (INT8U *)p2_mem_align;
        }
    }

    while ((mem_cmp == DEF_YES) &&                              /* Cmp mem bufs while identical ...                     */
           (size_rem > 0)) {                                    /* ... for unaligned mem bufs or trailing octets.       */
        p1_mem_08--;
        p2_mem_08--;
        if (*p1_mem_08 != *p2_mem_08) {                         /* If ANY data octet(s) NOT identical, cmp fails.       */
             mem_cmp = DEF_NO;
        }
        size_rem -= sizeof(INT8U);
    }

    return (mem_cmp);
}


INT32S MEM_GetNBytesData(const INT8U *dataStart, const INT8U size)
{
	INT8U  i;
	INT32S output = 0;
	
    if(dataStart == (void *)0) 
	{
        return 0;
    }
	
	if(size > 4)
	{
		return 0;
	}
	
	for(i = 0; i < size; i++)
	{
		output = output << 8;
		output = output + dataStart[i];
	}
	
	return output;
}

/****************************************************************************
 *
 * get16
 *
 * get a 16bit value always in large endian
 *
 * INPUTS
 *
 * s - data pointer where the data is
 *
 * RETURNS
 *
 * 16bit number
 *
 ***************************************************************************/

INT16U get16(INT8U *s) 
{
	INT16U output;
	
	output = s[0];
	output <<= 8;
	output |= s[1];
	
	return output;
}

/****************************************************************************
 *
 * get32
 *
 * get a 32bit value always in large endian
 *
 * INPUTS
 *
 * s - data pointer where the data is
 *
 * RETURNS
 *
 * 32bit number
 *
 ***************************************************************************/

INT32U get32(INT8U *s) 
{
	INT32U output;

	output = get16(s);
	output <<= 16;
	output |= get16(s + 2);

	return output;
}




