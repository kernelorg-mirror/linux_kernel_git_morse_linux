/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_IRQ_H
#define __ASM_IRQ_H

#ifndef __ASSEMBLER__

#include <asm-generic/irq.h>

/*
 * We need a value to serve as a irq-type for LPIs. Choose one that will
 * hopefully pique the interest of the reviewer.
 */
#define GIC_IRQ_TYPE_LPI		0xa110c8ed
#define GIC_IRQ_TYPE_PARTITION		(GIC_IRQ_TYPE_LPI + 1)

struct pt_regs;

int set_handle_irq(void (*handle_irq)(struct pt_regs *));
#define set_handle_irq	set_handle_irq
int set_handle_fiq(void (*handle_fiq)(struct pt_regs *));

static inline int nr_legacy_irqs(void)
{
	return 0;
}

#endif /* !__ASSEMBLER__ */
#endif
