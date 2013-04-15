/* linux/arch/arm/mach-msm/gpio.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/mach/irq.h>
#include <mach/gpiomux.h>
#include "gpio_hw.h"
#include "proc_comm.h"
#include "smd_private.h"

enum {
	GPIO_DEBUG_SLEEP = 1U << 0,
};
static int msm_gpio_debug_mask;
module_param_named(debug_mask, msm_gpio_debug_mask, int,
		   S_IRUGO | S_IWUSR | S_IWGRP);

#define FIRST_GPIO_IRQ MSM_GPIO_TO_INT(0)

#define MSM_GPIO_BANK(bank, first, last)				\
	{								\
		.regs = {						\
			.out =         MSM_GPIO_OUT_##bank,		\
			.in =          MSM_GPIO_IN_##bank,		\
			.int_status =  MSM_GPIO_INT_STATUS_##bank,	\
			.int_clear =   MSM_GPIO_INT_CLEAR_##bank,	\
			.int_en =      MSM_GPIO_INT_EN_##bank,		\
			.int_edge =    MSM_GPIO_INT_EDGE_##bank,	\
			.int_pos =     MSM_GPIO_INT_POS_##bank,		\
			.oe =          MSM_GPIO_OE_##bank,		\
		},							\
		.chip = {						\
			.base = (first),				\
			.ngpio = (last) - (first) + 1,			\
			.get = msm_gpio_get,				\
			.set = msm_gpio_set,				\
			.direction_input = msm_gpio_direction_input,	\
			.direction_output = msm_gpio_direction_output,	\
			.to_irq = msm_gpio_to_irq,			\
			.request = msm_gpio_request,			\
			.free = msm_gpio_free,				\
		}							\
	}

#define MSM_GPIO_BROKEN_INT_CLEAR 1

struct msm_gpio_regs {
	void __iomem *out;
	void __iomem *in;
	void __iomem *int_status;
	void __iomem *int_clear;
	void __iomem *int_en;
	void __iomem *int_edge;
	void __iomem *int_pos;
	void __iomem *oe;
};

struct msm_gpio_chip {
	spinlock_t		lock;
	struct gpio_chip	chip;
	struct msm_gpio_regs	regs;
#if MSM_GPIO_BROKEN_INT_CLEAR
	unsigned                int_status_copy;
#endif
	unsigned int            both_edge_detect;
	unsigned int            int_enable[2]; /* 0: awake, 1: sleep */
};

static int msm_gpio_write(struct msm_gpio_chip *msm_chip,
			  unsigned offset, unsigned on)
{
	unsigned mask = BIT(offset);
	unsigned val;

	val = __raw_readl(msm_chip->regs.out);
	if (on)
		__raw_writel(val | mask, msm_chip->regs.out);
	else
		__raw_writel(val & ~mask, msm_chip->regs.out);
	return 0;
}

static void msm_gpio_update_both_edge_detect(struct msm_gpio_chip *msm_chip)
{
	int loop_limit = 100;
	unsigned pol, val, val2, intstat;
	do {
		val = __raw_readl(msm_chip->regs.in);
		pol = __raw_readl(msm_chip->regs.int_pos);
		pol = (pol & ~msm_chip->both_edge_detect) |
		      (~val & msm_chip->both_edge_detect);
		__raw_writel(pol, msm_chip->regs.int_pos);
		intstat = __raw_readl(msm_chip->regs.int_status);
		val2 = __raw_readl(msm_chip->regs.in);
		if (((val ^ val2) & msm_chip->both_edge_detect & ~intstat) == 0)
			return;
	} while (loop_limit-- > 0);
	printk(KERN_ERR "msm_gpio_update_both_edge_detect, "
	       "failed to reach stable state %x != %x\n", val, val2);
}

static int msm_gpio_clear_detect_status(struct msm_gpio_chip *msm_chip,
					unsigned offset)
{
	unsigned bit = BIT(offset);

#if MSM_GPIO_BROKEN_INT_CLEAR
	/* Save interrupts that already triggered before we loose them. */
	/* Any interrupt that triggers between the read of int_status */
	/* and the write to int_clear will still be lost though. */
	msm_chip->int_status_copy |= __raw_readl(msm_chip->regs.int_status);
	msm_chip->int_status_copy &= ~bit;
#endif
	__raw_writel(bit, msm_chip->regs.int_clear);
	msm_gpio_update_both_edge_detect(msm_chip);
	return 0;
}

static int msm_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_chip *msm_chip;
	unsigned long irq_flags;

	msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	__raw_writel(__raw_readl(msm_chip->regs.oe) & ~BIT(offset),
			msm_chip->regs.oe);
	mb();
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
	return 0;
}

static int
msm_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_chip *msm_chip;
	unsigned long irq_flags;

	msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	msm_gpio_write(msm_chip, offset, value);
	__raw_writel(__raw_readl(msm_chip->regs.oe) | BIT(offset),
			msm_chip->regs.oe);
	mb();
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
	return 0;
}

static int msm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_chip *msm_chip;
	int rc;

	msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	rc = (__raw_readl(msm_chip->regs.in) & (1U << offset)) ? 1 : 0;
	mb();
	return rc;
}

static void msm_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_chip *msm_chip;
	unsigned long irq_flags;

	msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	msm_gpio_write(msm_chip, offset, value);
	mb();
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
}

static int msm_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return MSM_GPIO_TO_INT(chip->base + offset);
}

#ifdef CONFIG_MSM_GPIOMUX
static int msm_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return msm_gpiomux_get(chip->base + offset);
}

static void msm_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	msm_gpiomux_put(chip->base + offset);
}
#else
#define msm_gpio_request NULL
#define msm_gpio_free NULL
#endif

struct msm_gpio_chip msm_gpio_chips[] = {
#if defined(CONFIG_ARCH_MSM7X00A)
	MSM_GPIO_BANK(0,   0,  15),
	MSM_GPIO_BANK(1,  16,  42),
	MSM_GPIO_BANK(2,  43,  67),
	MSM_GPIO_BANK(3,  68,  94),
	MSM_GPIO_BANK(4,  95, 106),
	MSM_GPIO_BANK(5, 107, 121),
#elif defined(CONFIG_ARCH_MSM7X25) || defined(CONFIG_ARCH_MSM7X27)
	MSM_GPIO_BANK(0,   0,  15),
	MSM_GPIO_BANK(1,  16,  42),
	MSM_GPIO_BANK(2,  43,  67),
	MSM_GPIO_BANK(3,  68,  94),
	MSM_GPIO_BANK(4,  95, 106),
	MSM_GPIO_BANK(5, 107, 132),
#elif defined(CONFIG_ARCH_MSM7X30)
	MSM_GPIO_BANK(0,   0,  15),
	MSM_GPIO_BANK(1,  16,  43),
	MSM_GPIO_BANK(2,  44,  67),
	MSM_GPIO_BANK(3,  68,  94),
	MSM_GPIO_BANK(4,  95, 106),
	MSM_GPIO_BANK(5, 107, 133),
	MSM_GPIO_BANK(6, 134, 150),
	MSM_GPIO_BANK(7, 151, 181),
#elif defined(CONFIG_ARCH_QSD8X50)
	MSM_GPIO_BANK(0,   0,  15),
	MSM_GPIO_BANK(1,  16,  42),
	MSM_GPIO_BANK(2,  43,  67),
	MSM_GPIO_BANK(3,  68,  94),
	MSM_GPIO_BANK(4,  95, 103),
	MSM_GPIO_BANK(5, 104, 121),
	MSM_GPIO_BANK(6, 122, 152),
	MSM_GPIO_BANK(7, 153, 164),
#endif
};

static void msm_gpio_irq_ack(struct irq_data *d)
{
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip = irq_get_chip_data(d->irq);
	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	msm_gpio_clear_detect_status(msm_chip,
			     d->irq - gpio_to_irq(msm_chip->chip.base));
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
}

static void msm_gpio_irq_mask(struct irq_data *d)
{
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip = irq_get_chip_data(d->irq);
	unsigned offset = d->irq - gpio_to_irq(msm_chip->chip.base);

	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	/* level triggered interrupts are also latched */
	if (!(__raw_readl(msm_chip->regs.int_edge) & BIT(offset)))
		msm_gpio_clear_detect_status(msm_chip, offset);
	msm_chip->int_enable[0] &= ~BIT(offset);
	__raw_writel(msm_chip->int_enable[0], msm_chip->regs.int_en);
	mb();
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
}

static void msm_gpio_irq_unmask(struct irq_data *d)
{
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip = irq_get_chip_data(d->irq);
	unsigned offset = d->irq - gpio_to_irq(msm_chip->chip.base);

	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	/* level triggered interrupts are also latched */
	if (!(__raw_readl(msm_chip->regs.int_edge) & BIT(offset)))
		msm_gpio_clear_detect_status(msm_chip, offset);
	msm_chip->int_enable[0] |= BIT(offset);
	__raw_writel(msm_chip->int_enable[0], msm_chip->regs.int_en);
	mb();
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
}

static int msm_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip = irq_get_chip_data(d->irq);
	unsigned offset = d->irq - gpio_to_irq(msm_chip->chip.base);

	spin_lock_irqsave(&msm_chip->lock, irq_flags);

	if (on)
		msm_chip->int_enable[1] |= BIT(offset);
	else
		msm_chip->int_enable[1] &= ~BIT(offset);

	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
	return 0;
}

static int msm_gpio_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip = irq_get_chip_data(d->irq);
	unsigned offset = d->irq - gpio_to_irq(msm_chip->chip.base);
	unsigned val, mask = BIT(offset);

	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	val = __raw_readl(msm_chip->regs.int_edge);
	if (flow_type & IRQ_TYPE_EDGE_BOTH) {
		__raw_writel(val | mask, msm_chip->regs.int_edge);
		__irq_set_handler_locked(d->irq, handle_edge_irq);
	} else {
		__raw_writel(val & ~mask, msm_chip->regs.int_edge);
		__irq_set_handler_locked(d->irq, handle_level_irq);
	}
	if ((flow_type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
		msm_chip->both_edge_detect |= mask;
		msm_gpio_update_both_edge_detect(msm_chip);
	} else {
		msm_chip->both_edge_detect &= ~mask;
		val = __raw_readl(msm_chip->regs.int_pos);
		if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH))
			__raw_writel(val | mask, msm_chip->regs.int_pos);
		else
			__raw_writel(val & ~mask, msm_chip->regs.int_pos);
	}
	mb();
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
	return 0;
}

static void msm_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int i, j, mask;
	unsigned val;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		struct msm_gpio_chip *msm_chip = &msm_gpio_chips[i];
		val = __raw_readl(msm_chip->regs.int_status);
		val &= msm_chip->int_enable[0];
		while (val) {
			mask = val & -val;
			j = fls(mask) - 1;
			/* printk("%s %08x %08x bit %d gpio %d irq %d\n",
				__func__, v, m, j, msm_chip->chip.start + j,
				FIRST_GPIO_IRQ + msm_chip->chip.start + j); */
			val &= ~mask;
			generic_handle_irq(FIRST_GPIO_IRQ +
					   msm_chip->chip.base + j);
		}
	}

	chained_irq_exit(chip, desc);
}

static struct irq_chip msm_gpio_irq_chip = {
	.name      = "msmgpio",
	.irq_ack	= msm_gpio_irq_ack,
	.irq_mask	= msm_gpio_irq_mask,
	.irq_unmask	= msm_gpio_irq_unmask,
	.irq_set_wake	= msm_gpio_irq_set_wake,
	.irq_set_type	= msm_gpio_irq_set_type,
};

#define NUM_GPIO_SMEM_BANKS 6
#define GPIO_SMEM_NUM_GROUPS 2
#define GPIO_SMEM_MAX_PC_INTERRUPTS 8
struct tramp_gpio_smem {
	uint16_t num_fired[GPIO_SMEM_NUM_GROUPS];
	uint16_t fired[GPIO_SMEM_NUM_GROUPS][GPIO_SMEM_MAX_PC_INTERRUPTS];
	uint32_t enabled[NUM_GPIO_SMEM_BANKS];
	uint32_t detection[NUM_GPIO_SMEM_BANKS];
	uint32_t polarity[NUM_GPIO_SMEM_BANKS];
};

static void msm_gpio_sleep_int(unsigned long arg)
{
	int i, j;
	struct tramp_gpio_smem *smem_gpio;

	BUILD_BUG_ON(NR_GPIO_IRQS > NUM_GPIO_SMEM_BANKS * 32);

	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio));
	if (smem_gpio == NULL)
		return;

	local_irq_disable();
	for (i = 0; i < GPIO_SMEM_NUM_GROUPS; i++) {
		int count = smem_gpio->num_fired[i];
		for (j = 0; j < count; j++) {
			/* TODO: Check mask */
			generic_handle_irq(
				MSM_GPIO_TO_INT(smem_gpio->fired[i][j]));
		}
	}
	local_irq_enable();
}

static DECLARE_TASKLET(msm_gpio_sleep_int_tasklet, msm_gpio_sleep_int, 0);

void msm_gpio_enter_sleep(int from_idle)
{
	int i;
	struct tramp_gpio_smem *smem_gpio;

	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio));

	if (smem_gpio) {
		for (i = 0; i < ARRAY_SIZE(smem_gpio->enabled); i++) {
			smem_gpio->enabled[i] = 0;
			smem_gpio->detection[i] = 0;
			smem_gpio->polarity[i] = 0;
		}
	}

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		__raw_writel(msm_gpio_chips[i].int_enable[!from_idle],
		       msm_gpio_chips[i].regs.int_en);
		if (smem_gpio) {
			uint32_t tmp;
			int start, index, shiftl, shiftr;
			start = msm_gpio_chips[i].chip.base;
			index = start / 32;
			shiftl = start % 32;
			shiftr = 32 - shiftl;
			tmp = msm_gpio_chips[i].int_enable[!from_idle];
			smem_gpio->enabled[index] |= tmp << shiftl;
			smem_gpio->enabled[index+1] |= tmp >> shiftr;
			smem_gpio->detection[index] |=
				__raw_readl(msm_gpio_chips[i].regs.int_edge) <<
				shiftl;
			smem_gpio->detection[index+1] |=
				__raw_readl(msm_gpio_chips[i].regs.int_edge) >>
				shiftr;
			smem_gpio->polarity[index] |=
				__raw_readl(msm_gpio_chips[i].regs.int_pos) <<
				shiftl;
			smem_gpio->polarity[index+1] |=
				__raw_readl(msm_gpio_chips[i].regs.int_pos) >>
				shiftr;
		}
	}
	mb();

	if (smem_gpio) {
		if (msm_gpio_debug_mask & GPIO_DEBUG_SLEEP)
			for (i = 0; i < ARRAY_SIZE(smem_gpio->enabled); i++) {
				printk("msm_gpio_enter_sleep gpio %d-%d: enable"
				       " %08x, edge %08x, polarity %08x\n",
				       i * 32, i * 32 + 31,
				       smem_gpio->enabled[i],
				       smem_gpio->detection[i],
				       smem_gpio->polarity[i]);
			}
		for (i = 0; i < GPIO_SMEM_NUM_GROUPS; i++)
			smem_gpio->num_fired[i] = 0;
	}
}

void msm_gpio_exit_sleep(void)
{
	int i;
	struct tramp_gpio_smem *smem_gpio;

	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio));

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		__raw_writel(msm_gpio_chips[i].int_enable[0],
		       msm_gpio_chips[i].regs.int_en);
	}
	mb();

	if (smem_gpio && (smem_gpio->num_fired[0] || smem_gpio->num_fired[1])) {
		if (msm_gpio_debug_mask & GPIO_DEBUG_SLEEP)
			printk(KERN_INFO "gpio: fired %x %x\n",
			      smem_gpio->num_fired[0], smem_gpio->num_fired[1]);
		tasklet_schedule(&msm_gpio_sleep_int_tasklet);
	}
}


int gpio_tlmm_config(unsigned config, unsigned disable)
{
	return msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, &disable);
}
EXPORT_SYMBOL(gpio_tlmm_config);

#define ZTE_FEATURE_SLEEP_GPIO_CNF_APP
#include <linux/debugfs.h>


#ifdef ZTE_FEATURE_SLEEP_GPIO_CNF_APP
int gpio_sleep_tlmm_config(unsigned config, unsigned disable)
{
	return msm_proc_comm(PCOM_CUSTOMER_CMD3, &config, &disable);
}
EXPORT_SYMBOL(gpio_sleep_tlmm_config);

#endif

int msm_gpios_request_enable(const struct msm_gpio *table, int size)
{
	int rc = msm_gpios_request(table, size);
	if (rc)
		return rc;
	rc = msm_gpios_enable(table, size);
	if (rc)
		msm_gpios_free(table, size);
	return rc;
}
EXPORT_SYMBOL(msm_gpios_request_enable);

void msm_gpios_disable_free(const struct msm_gpio *table, int size)
{
	msm_gpios_disable(table, size);
	msm_gpios_free(table, size);
}
EXPORT_SYMBOL(msm_gpios_disable_free);

int msm_gpios_request(const struct msm_gpio *table, int size)
{
	int rc;
	int i;
	const struct msm_gpio *g;
	for (i = 0; i < size; i++) {
		g = table + i;
		rc = gpio_request(GPIO_PIN(g->gpio_cfg), g->label);
		if (rc) {
			pr_err("gpio_request(%d) <%s> failed: %d\n",
			       GPIO_PIN(g->gpio_cfg), g->label ?: "?", rc);
			goto err;
		}
	}
	return 0;
err:
	msm_gpios_free(table, i);
	return rc;
}
EXPORT_SYMBOL(msm_gpios_request);

void msm_gpios_free(const struct msm_gpio *table, int size)
{
	int i;
	const struct msm_gpio *g;
	for (i = size-1; i >= 0; i--) {
		g = table + i;
		gpio_free(GPIO_PIN(g->gpio_cfg));
	}
}
EXPORT_SYMBOL(msm_gpios_free);

int msm_gpios_enable(const struct msm_gpio *table, int size)
{
	int rc;
	int i;
	const struct msm_gpio *g;
	for (i = 0; i < size; i++) {
		g = table + i;
		rc = gpio_tlmm_config(g->gpio_cfg, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("gpio_tlmm_config(0x%08x, GPIO_CFG_ENABLE)"
			       " <%s> failed: %d\n",
			       g->gpio_cfg, g->label ?: "?", rc);
			pr_err("pin %d func %d dir %d pull %d drvstr %d\n",
			       GPIO_PIN(g->gpio_cfg), GPIO_FUNC(g->gpio_cfg),
			       GPIO_DIR(g->gpio_cfg), GPIO_PULL(g->gpio_cfg),
			       GPIO_DRVSTR(g->gpio_cfg));
			goto err;
		}
	}
	return 0;
err:
	msm_gpios_disable(table, i);
	return rc;
}
EXPORT_SYMBOL(msm_gpios_enable);

int msm_gpios_disable(const struct msm_gpio *table, int size)
{
	int rc = 0;
	int i;
	const struct msm_gpio *g;
	for (i = size-1; i >= 0; i--) {
		int tmp;
		g = table + i;
		tmp = gpio_tlmm_config(g->gpio_cfg, GPIO_CFG_DISABLE);
		if (tmp) {
			pr_err("gpio_tlmm_config(0x%08x, GPIO_CFG_DISABLE)"
			       " <%s> failed: %d\n",
			       g->gpio_cfg, g->label ?: "?", rc);
			pr_err("pin %d func %d dir %d pull %d drvstr %d\n",
			       GPIO_PIN(g->gpio_cfg), GPIO_FUNC(g->gpio_cfg),
			       GPIO_DIR(g->gpio_cfg), GPIO_PULL(g->gpio_cfg),
			       GPIO_DRVSTR(g->gpio_cfg));
			if (!rc)
				rc = tmp;
		}
	}

	return rc;
}
EXPORT_SYMBOL(msm_gpios_disable);

/* Locate the GPIO_OUT register for the given GPIO and return its address
 * and the bit position of the gpio's bit within the register.
 *
 * This function is used by gpiomux-v1 in order to support output transitions.
 */
void msm_gpio_find_out(const unsigned gpio, void __iomem **out,
	unsigned *offset)
{
	struct msm_gpio_chip *msm_chip = msm_gpio_chips;

	while (gpio >= msm_chip->chip.base + msm_chip->chip.ngpio)
		++msm_chip;

	*out = msm_chip->regs.out;
	*offset = gpio - msm_chip->chip.base;
}

static int __devinit msm_gpio_probe(struct platform_device *dev)
{
	int i, j = 0;
	int grp_irq;

	for (i = FIRST_GPIO_IRQ; i < FIRST_GPIO_IRQ + NR_GPIO_IRQS; i++) {
		if (i - FIRST_GPIO_IRQ >=
			msm_gpio_chips[j].chip.base +
			msm_gpio_chips[j].chip.ngpio)
			j++;
		irq_set_chip_data(i, &msm_gpio_chips[j]);
		irq_set_chip_and_handler(i, &msm_gpio_irq_chip,
					 handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i = 0; i < dev->num_resources; i++) {
		grp_irq = platform_get_irq(dev, i);
		if (grp_irq < 0)
			return -ENXIO;

		irq_set_chained_handler(grp_irq, msm_gpio_irq_handler);
		irq_set_irq_wake(grp_irq, (i + 1));
	}

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		spin_lock_init(&msm_gpio_chips[i].lock);
		__raw_writel(0, msm_gpio_chips[i].regs.int_en);
		gpiochip_add(&msm_gpio_chips[i].chip);
	}

	mb();
	return 0;
}

static struct platform_driver msm_gpio_driver = {
	.probe = msm_gpio_probe,
	.driver = {
		.name = "msmgpio",
		.owner = THIS_MODULE,
	},
};
#if defined(CONFIG_DEBUG_FS)

#define ZTE_PLATFORM_CONFIGURE_GPIO_SYS 
#ifdef	ZTE_PLATFORM_CONFIGURE_GPIO_SYS
static int zte_gpio_output_result = 0;
static int gpio_num_to_get = 0;
static int zte_gpio_output_high_set(void *data, u64 val)
{
	pr_info("%s set gpio %d output_high\n",__func__,(int)val);
	gpio_num_to_get = val;
	zte_gpio_output_result = gpio_direction_output(val,1);
	return 0;
}
static int zte_gpio_output_low_set(void *data, u64 val)
{
	pr_info("%s set gpio %d output_low\n",__func__,(int)val);
	gpio_num_to_get = val;
	zte_gpio_output_result = gpio_direction_output(val,0);
	return 0;
}
static int zte_gpio_num_set(void *data, u64 val)
{
	pr_info("%s Going to get gpio %d 's status\n",__func__,(int)val);
	gpio_num_to_get = val;
	return 0;
}
static int zte_gpio_get(void *data, u64 *val)
{
	unsigned int result = 0;
	result = gpio_get_value(gpio_num_to_get);	
	if (result)
		*val = 1;
	else
		*val = 0;
	pr_info("%s GET gpio %d statue is %s,result = %d\n",__func__,gpio_num_to_get,result?"HIGH":"LOW",result);
	return 0;
}
#endif



#ifdef ZTE_FEATURE_SLEEP_GPIO_CNF_APP
static int msm_gpio_debug_result = 1;
/*GPIO_CFG in ARM9
define GPIO_CFG(gpio, func, dir, pull, drvstr, rmt) \
         (((gpio)&0xFF)<<8|((rmt)&0xF)<<4|((func)&0xF)|((dir)&0x1)<<16| \
         ((pull)&0x3)<<17|((drvstr)&0x7)<<19)
//		printk("%02d: info 0x%x\n",i,info);
*/

void gpio_printk_temp(u64 gpio_config,int debug_result)
{
	u16 gpio_number;
	u16 func_val;
	u16 dir_val;
	u16 pull_val;
	u16 drvstr_val;
	u16 rmtval; 
	gpio_number = (((gpio_config) >> 8) & 0xFF);
	  rmtval =  (((gpio_config) >> 4) & 0xF);
	  func_val =    ( (gpio_config) & 0xF);
	  dir_val =     (((gpio_config) >> 16) & 0x1);
	  pull_val =    (((gpio_config) >> 17) & 0x3);
	  drvstr_val =  (((gpio_config) >> 19) & 0x7);

	printk("GPIO %02d;func %02d;dir %02d;pull %02d;drvstr %02d;rmt %02d;\n",gpio_number,func_val,dir_val,pull_val,drvstr_val,rmtval);
		printk(" debug_result %02d\n",debug_result);
}

static int gpio_sleep_enable_set(void *data, u64 val)
{
	msm_gpio_debug_result = gpio_sleep_tlmm_config(val, 1);
	gpio_printk_temp(val,msm_gpio_debug_result);
	
	return 0;
}
static int gpio_sleep_disable_set(void *data, u64 val)
{
	msm_gpio_debug_result = gpio_sleep_tlmm_config(val, 0);
	gpio_printk_temp(val,msm_gpio_debug_result);
	return 0;
}

#endif
#if defined(ZTE_PLATFORM_CONFIGURE_GPIO_SYS) || defined(ZTE_FEATURE_SLEEP_GPIO_CNF_APP)
static int gpio_debug_get(void *data, u64 *val)
{
	unsigned int result = msm_gpio_debug_result;
	msm_gpio_debug_result = 1;
	if (result)
		*val = 1;
	else
		*val = 0;
	return 0;
}
#endif
						
#ifdef ZTE_PLATFORM_CONFIGURE_GPIO_SYS
DEFINE_SIMPLE_ATTRIBUTE(zte_gpio_out_high_fops, zte_gpio_get,
						zte_gpio_output_high_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(zte_gpio_out_low_fops, zte_gpio_get,
						zte_gpio_output_low_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(zte_gpio_get_fops, zte_gpio_get,
						zte_gpio_num_set, "%llu\n");
#endif

#ifdef  ZTE_FEATURE_SLEEP_GPIO_CNF_APP
DEFINE_SIMPLE_ATTRIBUTE(gpio_sleep_enable_fops, gpio_debug_get,
						gpio_sleep_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(gpio_sleep_disable_fops, gpio_debug_get,
						gpio_sleep_disable_set, "%llu\n");
#endif

#endif

static int __init msm_gpio_init(void)
{
#if defined(CONFIG_DEBUG_FS)
#if defined(ZTE_PLATFORM_CONFIGURE_GPIO_SYS) || defined(ZTE_FEATURE_SLEEP_GPIO_CNF_APP)
	struct dentry *dent;
	dent = debugfs_create_dir("gpio_debug", 0);
	if (IS_ERR(dent))
		pr_info(" fail to create gpio_debug debugfs\n");
#endif

	
#ifdef ZTE_PLATFORM_CONFIGURE_GPIO_SYS
	debugfs_create_file("gpio_out_h", 0666, dent, 0, &zte_gpio_out_high_fops);
	debugfs_create_file("gpio_out_l", 0666, dent, 0, &zte_gpio_out_low_fops);
	debugfs_create_file("gpio_get", 0666, dent, 0, &zte_gpio_get_fops);//first echo XX > gpio_get to set which gpio to get status,then cat gpio_get to show the status
#endif

#ifdef  ZTE_FEATURE_SLEEP_GPIO_CNF_APP
	debugfs_create_file("enable_sleep", 0644, dent, 0, &gpio_sleep_enable_fops);
	debugfs_create_file("disable_sleep", 0644, dent, 0, &gpio_sleep_disable_fops);
#endif
#endif
	return platform_driver_register(&msm_gpio_driver);
}
postcore_initcall(msm_gpio_init);
