/*
 * cpu.c
 *
 *  Created on: Dec 8, 2018
 *      Author: xianghe
 */

#include "cpu.h"
#include "XMC1300.h"

CPU_SR CPU_SR_Save(void)
{
	CPU_SR cpu_sr;

	cpu_sr = __get_PRIMASK();
	__disable_irq ();

	return cpu_sr;
}

void CPU_SR_Restore(CPU_SR  cpu_sr)
{
	__set_PRIMASK(cpu_sr);
}

void CPU_IntDis(void)
{
	__disable_irq();
}

void CPU_IntEn(void)
{
	__enable_irq();
}
