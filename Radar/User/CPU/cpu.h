#ifndef _CPU_H
#define _CPU_H

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/


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

typedef unsigned char  							BOOLEAN;
typedef unsigned char  							INT8U; 	/* Unsigned  8 bit quantity         */
typedef signed   char  							INT8S;	/* Signed    8 bit quantity         */
typedef unsigned short 							INT16U;	/* Unsigned 16 bit quantity         */
typedef signed   short 							INT16S;	/* Signed   16 bit quantity         */
typedef unsigned long  							INT32U;	/* Unsigned 32 bit quantity         */
typedef signed   long  							INT32S;	/* Signed   32 bit quantity         */
typedef float          							FP32;	/* Single precision floating point  */
typedef double         							FP64;	/* Double precision floating point  */

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/

													/* ----------------------- CPU WORD SIZE ---------------------- */
#define CPU_WORD_SIZE_08         				1   /*  8-bit word size = sizeof(CPU_INT08x).                       */
#define CPU_WORD_SIZE_16               			2   /* 16-bit word size = sizeof(CPU_INT16x).                       */
#define CPU_WORD_SIZE_32                       	4   /* 32-bit word size = sizeof(CPU_INT32x).                       */
#define CPU_WORD_SIZE_64                       	8   /* 64-bit word size = sizeof(CPU_INT64x) [see Note #1a].        */


                                                    /* ------------------- CPU WORD-ENDIAN ORDER ------------------ */
#define CPU_ENDIAN_TYPE_NONE                   	0   /*                                                              */
#define CPU_ENDIAN_TYPE_BIG                    	1   /* Big-   endian word order (CPU words' most  significant ...   */
                                                    /*                           ... octet @ lowest mem addr).      */
#define CPU_ENDIAN_TYPE_LITTLE                 	2   /* Little-endian word order (CPU words' least significant ...   */
                                                    /*    
													... octet @ lowest mem addr).      */
													
#define CPU_CFG_ADDR_SIZE              			CPU_WORD_SIZE_32        /* Defines CPU address word size.                       */

#define CPU_CFG_DATA_SIZE              			CPU_WORD_SIZE_32        /* Defines CPU data    word size.                       */
#define CPU_CFG_ENDIAN_TYPE            			CPU_ENDIAN_TYPE_LITTLE  /* Defines CPU data    word-memory order.               */


/*
*********************************************************************************************************
*                                 CONFIGURE CPU ADDRESS & DATA TYPES
*********************************************************************************************************
*/

															/* CPU address type based on address bus size.          */
#if     (CPU_CFG_ADDR_SIZE == CPU_WORD_SIZE_32)
typedef  INT32U  								CPU_ADDR;
#elif   (CPU_CFG_ADDR_SIZE == CPU_WORD_SIZE_16)
typedef  INT16U  								CPU_ADDR;
#else
typedef  INT8U   								CPU_ADDR;
#endif

															/* CPU data    type based on data    bus size.          */
#if     (CPU_CFG_DATA_SIZE == CPU_WORD_SIZE_32)
typedef  INT32U  								CPU_DATA;
#elif   (CPU_CFG_DATA_SIZE == CPU_WORD_SIZE_16)
typedef  INT16U  								CPU_DATA;
#else
typedef  INT8U   								CPU_DATA;
#endif


typedef  CPU_DATA    							CPU_ALIGN;	/* Defines CPU data-word-alignment size.                */
typedef  CPU_DATA    							CPU_SIZE_T;	/* Defines CPU standard 'size_t'   size.                */
typedef  INT32U        							CPU_SR;		/* Defines   CPU status register size (see Note #3b).   */


#define CPU_SR_ALLOC()             				CPU_SR  cpu_sr = (CPU_SR)0

#define CPU_INT_DIS()         					do { cpu_sr = CPU_SR_Save(); } while (0) /* Save    CPU status word & disable interrupts.*/
#define CPU_INT_EN()          					do { CPU_SR_Restore(cpu_sr); } while (0) /* Restore CPU status word.                     */

#define CPU_CRITICAL_ENTER()  					do { CPU_INT_DIS(); } while (0)          /* Disable   interrupts.                        */
#define CPU_CRITICAL_EXIT()   					do { CPU_INT_EN();  } while (0)          /* Re-enable interrupts.                        */


#define nop()									__nop()

#define DWORD_LOW(dword)						((INT16U) ((INT32U)(dword) & 0xFFFF))
#define DWORD_HIGH(dword)						((INT16U) ((INT32U)(dword) >> 16))
#define WORD_LOW(word)  						((INT8U) ((INT16U)(word) & 255)) 
#define WORD_HIGH(word)  						((INT8U) ((INT16U)(word) >> 8)) 

//#define SET_BIT(REG, BIT)     					((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   					((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    					((REG) & (BIT))

#define CLEAR_REG(REG)        					((REG) = (0x0))

#define WRITE_REG(REG, VAL)   					((REG) = (VAL))

//static union
//{
//	char c[4];
//	INT32U l;
//} endian_test = { { 'l', '?', '?', 'b' } };
//#define ENDIANNESS 								((char)endian_test.l)


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void CPU_IntDis(void);

void CPU_IntEn(void);

CPU_SR CPU_SR_Save(void);

void CPU_SR_Restore(CPU_SR cpu_sr);


#endif

