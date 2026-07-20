/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MTK_PMIC_IRQ_MT6351_H
#define __MTK_PMIC_IRQ_MT6351_H

/*
 * MT6351 interrupt controller layout. MT6351 uses the legacy 3.10 INT model
 * (INT_CONx + _SET/_CLR), not the MT6355 raw_address/mask model. Macro and
 * structs from 3.10 mach/pmic.h, so the interrupts[] table in pmic_irq.c
 * matches.
 */
#define PMIC_M_INTS_GEN(adr, enA, setA, clearA, interrupt)	\
	{	\
		.address =  adr,	\
		.en =  enA,	\
		.set =  setA,	\
		.clear =  clearA,	\
		.interrupts = interrupt,	\
	}

struct pmic_interrupt_bit {
	const char *name;
	void (*callback)(void);
	unsigned int times;
};

struct pmic_interrupts {
	unsigned int address;
	unsigned int en;
	unsigned int set;
	unsigned int clear;
	struct pmic_interrupt_bit *interrupts;
};

#endif /* __MTK_PMIC_IRQ_MT6351_H */
