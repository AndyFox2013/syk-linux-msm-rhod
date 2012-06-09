/* linux/arch/arm/mach-msm/gpio.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include "gpio_hw.h"
#include "gpiomux.h"
#include <mach/gpio.h>
#include "gpio_hw.h"
#include "gpio_chip.h"
#ifndef CONFIG_MSM_AMSS_VERSION_WINCE
#include "proc_comm.h"
#else
#include "proc_comm_wince.h"
#endif
#include "smd_private.h"

enum {
	GPIO_DEBUG_SLEEP = 1U << 0,
};
static int msm_gpio_debug_mask;
module_param_named(debug_mask, msm_gpio_debug_mask, int, 
		   S_IRUGO | S_IWUSR | S_IWGRP);

#define MSM_GPIO_BROKEN_INT_CLEAR 1

 
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
			.start = (first),				\
			.end = (last),					\
				.old_chip = {	\
			.base = (first),				\
			.ngpio = (last) - (first) + 1,			\
			.get = (int(*)(gpio_chip_t *chip,unsigned offset))msm_gpio_get,				\
			.set = (void(*)(gpio_chip_t *chip,unsigned offset,int value))msm_gpio_set,				\
			.direction_input = (int(*)(gpio_chip_t *chip,unsigned offset))msm_gpio_direction_input,	\
			.direction_output = (int(*)(gpio_chip_t *chip,unsigned offset,int value))msm_gpio_direction_output,	\
			.to_irq = (int(*)(gpio_chip_t *chip,unsigned offset))msm_gpio_to_irq,			\
	/*		.request = (int(*)(gpio_chip_t *chip,unsigned offset))msm_gpio_request,			\
			.free = (int(*)(gpio_chip_t *chip,unsigned offset))msm_gpio_free,				\*/\
			} \
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
	void __iomem *owner;
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

	val = readl(msm_chip->regs.out);
	if (on)
		writel(val | mask, msm_chip->regs.out);
	else
		writel(val & ~mask, msm_chip->regs.out);
	return 0;
}

static void msm_gpio_update_both_edge_detect(struct msm_gpio_chip *msm_chip)
{
	int loop_limit = 100;
	unsigned pol, val, val2, intstat;
	do {
		val = readl(msm_chip->regs.in);
		pol = readl(msm_chip->regs.int_pos);
		pol = (pol & ~msm_chip->both_edge_detect) |
		      (~val & msm_chip->both_edge_detect);
		writel(pol, msm_chip->regs.int_pos);
		intstat = readl(msm_chip->regs.int_status);
		val2 = readl(msm_chip->regs.in);
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
	msm_chip->int_status_copy |= readl(msm_chip->regs.int_status);
	msm_chip->int_status_copy &= ~bit;
#endif
	writel(bit, msm_chip->regs.int_clear);
	msm_gpio_update_both_edge_detect(msm_chip);
	return 0;
}

static int msm_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_chip *msm_chip;
	unsigned long irq_flags;

	msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	writel(readl(msm_chip->regs.oe) & ~BIT(offset), msm_chip->regs.oe);
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
	writel(readl(msm_chip->regs.oe) | BIT(offset), msm_chip->regs.oe);
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
	return 0;
}

static int msm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_chip *msm_chip;

	msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	return (readl(msm_chip->regs.in) & (1U << offset)) ? 1 : 0;
}

static void msm_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_chip *msm_chip;
	unsigned long irq_flags;

	msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	msm_gpio_write(msm_chip, offset, value);
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
}

static int msm_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return MSM_GPIO_TO_INT(chip->old_chip.base + offset);
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
	struct msm_gpio_chip *msm_chip = irq_data_get_irq_chip_data(d);
	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	msm_gpio_clear_detect_status(msm_chip,
				     d->irq - gpio_to_irq(msm_chip->chip.old_chip.base));
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
}

static void msm_gpio_irq_mask(struct irq_data *d)
{
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip = irq_data_get_irq_chip_data(d);
	unsigned offset = d->irq - gpio_to_irq(msm_chip->chip.old_chip.base);

	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	/* level triggered interrupts are also latched */
	if (!(readl(msm_chip->regs.int_edge) & BIT(offset)))
		msm_gpio_clear_detect_status(msm_chip, offset);
	msm_chip->int_enable[0] &= ~BIT(offset);
	writel(msm_chip->int_enable[0], msm_chip->regs.int_en);
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
}

static void msm_gpio_irq_unmask(struct irq_data *d)
{
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip = irq_data_get_irq_chip_data(d);
	unsigned offset = d->irq - gpio_to_irq(msm_chip->chip.old_chip.base);

	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	/* level triggered interrupts are also latched */
	if (!(readl(msm_chip->regs.int_edge) & BIT(offset)))
		msm_gpio_clear_detect_status(msm_chip, offset);
	msm_chip->int_enable[0] |= BIT(offset);
	writel(msm_chip->int_enable[0], msm_chip->regs.int_en);
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
}

static int msm_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip = irq_data_get_irq_chip_data(d);
	unsigned offset = d->irq - gpio_to_irq(msm_chip->chip.old_chip.base);

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
	struct msm_gpio_chip *msm_chip = irq_data_get_irq_chip_data(d);
	unsigned offset = d->irq - gpio_to_irq(msm_chip->chip.old_chip.base);
	unsigned val, mask = BIT(offset);

	spin_lock_irqsave(&msm_chip->lock, irq_flags);
	val = readl(msm_chip->regs.int_edge);
	if (flow_type & IRQ_TYPE_EDGE_BOTH) {
		writel(val | mask, msm_chip->regs.int_edge);
		__irq_set_handler_locked(d->irq, handle_edge_irq);
	} else {
		writel(val & ~mask, msm_chip->regs.int_edge);
		__irq_set_handler_locked(d->irq, handle_level_irq);
	}
	if ((flow_type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
		msm_chip->both_edge_detect |= mask;
		msm_gpio_update_both_edge_detect(msm_chip);
	} else {
		msm_chip->both_edge_detect &= ~mask;
		val = readl(msm_chip->regs.int_pos);
		if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH))
			writel(val | mask, msm_chip->regs.int_pos);
		else
			writel(val & ~mask, msm_chip->regs.int_pos);
	}
	spin_unlock_irqrestore(&msm_chip->lock, irq_flags);
	return 0;
}

static void msm_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int i, j, mask;
	unsigned val;

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		struct msm_gpio_chip *msm_chip = &msm_gpio_chips[i];
		val = readl(msm_chip->regs.int_status);
		val &= msm_chip->int_enable[0];
		while (val) {
			mask = val & -val;
			j = fls(mask) - 1;
			/* printk("%s %08x %08x bit %d gpio %d irq %d\n",
				__func__, v, m, j, msm_chip->chip.start + j,
				FIRST_GPIO_IRQ + msm_chip->chip.start + j); */
			val &= ~mask;
			generic_handle_irq(FIRST_GPIO_IRQ +
					   msm_chip->chip.old_chip.base + j);
		}
	}
	desc->irq_data.chip->irq_ack(&desc->irq_data);
}

static struct irq_chip msm_gpio_irq_chip = {
	.name          = "msmgpio",
	.irq_ack       = msm_gpio_irq_ack,
	.irq_mask      = msm_gpio_irq_mask,
	.irq_unmask    = msm_gpio_irq_unmask,
	.irq_set_wake  = msm_gpio_irq_set_wake,
	.irq_set_type  = msm_gpio_irq_set_type,
};

static int __init msm_init_gpio(void)
{
	int i, j = 0;

	for (i = FIRST_GPIO_IRQ; i < FIRST_GPIO_IRQ + NR_GPIO_IRQS; i++) {
		if (i - FIRST_GPIO_IRQ >=
			msm_gpio_chips[j].chip.old_chip.base +
			msm_gpio_chips[j].chip.old_chip.ngpio)
			j++;
		irq_set_chip_data(i, &msm_gpio_chips[j]);
		irq_set_chip_and_handler(i, &msm_gpio_irq_chip,
					 handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		spin_lock_init(&msm_gpio_chips[i].lock);
		writel(0, msm_gpio_chips[i].regs.int_en);
		gpiochip_add((gpio_chip_t*)&msm_gpio_chips[i].chip);
	}

	irq_set_chained_handler(INT_GPIO_GROUP1, msm_gpio_irq_handler);
	irq_set_chained_handler(INT_GPIO_GROUP2, msm_gpio_irq_handler);
	irq_set_irq_wake(INT_GPIO_GROUP1, 1);
	irq_set_irq_wake(INT_GPIO_GROUP2, 2);
	return 0;
}

postcore_initcall(msm_init_gpio);
int gpio_tlmm_config(unsigned config, unsigned disable)
{
#if defined(CONFIG_MSM_AMSS_VERSION_WINCE)
	void __iomem *addr, __iomem *addr2;
	struct msm_gpio_chip *msm_chip;
	unsigned cfg, gpio, i;
	unsigned long flags_gpio;

	gpio = GPIO_PIN(config);

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		msm_chip = &msm_gpio_chips[i];
		if (msm_chip->chip.start <= gpio && msm_chip->chip.end >= gpio)
			break;
		msm_chip = NULL;

	}
	
	if (!msm_chip) {
		printk("%s: could not find the gpio %d\n", __func__, gpio);
		return -EINVAL;
	}

	spin_lock_irqsave(&msm_chip->chip.lock, flags_gpio);

	if (GPIO_PIN(config) < 16 || GPIO_PIN(config) > 42) {
		addr = (void __iomem *)(MSM_GPIOCFG1_BASE + 0x20);
		addr2 = (void __iomem *)(MSM_GPIOCFG1_BASE + 0x24);
	}	
	else {
		addr = (void __iomem *)(MSM_GPIOCFG2_BASE + 0x410);
		addr2 = (void __iomem *)(MSM_GPIOCFG2_BASE + 0x414);
	}

	if (!addr || !addr2) {
		printk(KERN_WARNING "%s: could not find addr\n", __func__);
		spin_unlock_irqrestore(&msm_chip->chip.lock, flags_gpio);
		return 0;
	}

	writel(gpio, addr);
	cfg =
	    (GPIO_DRVSTR(config) << 6) | (GPIO_FUNC(config) << 2) |
	    (GPIO_PULL(config));
	writel(cfg, addr2);

	printk("%s(%x, %x)\n", __func__, gpio, cfg);
	if (readl(addr) != gpio)
		printk(KERN_WARNING "%s: could not set alt func %u => %u\n",
		       __func__, gpio, GPIO_FUNC(config));

	spin_unlock_irqrestore(&msm_chip->chip.lock, flags_gpio);

	gpio_request(gpio, NULL);

	if (GPIO_DIR(config))
		gpio_direction_output(gpio, !disable);
	else
		gpio_direction_input(gpio);

	return 0;
#else
	return msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, &disable);
#endif
}
EXPORT_SYMBOL(gpio_tlmm_config);

static DEFINE_SPINLOCK(gpio_flag_lock);

void msm_gpio_set_flags(unsigned gpio, unsigned long flags)
{

	int i;
	unsigned long irq_flags;
	struct msm_gpio_chip *msm_chip;
	unsigned b;
	unsigned v;

	spin_lock_irqsave(&gpio_flag_lock, irq_flags);
	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		msm_chip = &msm_gpio_chips[i];
		if (msm_chip->chip.start <= gpio && msm_chip->chip.end >= gpio)
			break;
		msm_chip = NULL;

	}

	if (!msm_chip)
		return;

	b = 1U << (gpio - msm_chip->chip.start);

	if (msm_chip->regs.owner) {
		v = readl(msm_chip->regs.owner);
		if (flags & GPIOF_OWNER_ARM11) {
			writel(v | b, msm_chip->regs.owner);
		} else {
			writel(v & ~b, msm_chip->regs.owner);
		}
	}
	spin_unlock_irqrestore(&gpio_flag_lock, irq_flags);
}

EXPORT_SYMBOL(msm_gpio_set_flags);
int msm_gpios_enable(const struct msm_gpio *table, int size)
{
	int rc;
	int i;
	const struct msm_gpio *g;
	for (i = 0; i < size; i++) {
		g = table + i;
		rc = gpio_tlmm_config(g->gpio_cfg, GPIO_ENABLE);
		if (rc) {
			pr_err("gpio_tlmm_config(0x%08x, GPIO_ENABLE)"
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


void msm_gpios_disable(const struct msm_gpio *table, int size)
{
	int rc;
	int i;
	const struct msm_gpio *g;
	for (i = size-1; i >= 0; i--) {
		g = table + i;
		rc = gpio_tlmm_config(g->gpio_cfg, GPIO_DISABLE);
		if (rc) {
			pr_err("gpio_tlmm_config(0x%08x, GPIO_DISABLE)"
			       " <%s> failed: %d\n",
			       g->gpio_cfg, g->label ?: "?", rc);
			pr_err("pin %d func %d dir %d pull %d drvstr %d\n",
			       GPIO_PIN(g->gpio_cfg), GPIO_FUNC(g->gpio_cfg),
			       GPIO_DIR(g->gpio_cfg), GPIO_PULL(g->gpio_cfg),
			       GPIO_DRVSTR(g->gpio_cfg));
		}
	}
}
EXPORT_SYMBOL(msm_gpios_disable);

int msm_gpios_request_enable(const struct msm_gpio *table, int size)
{
	int rc = msm_gpios_enable(table, size);
	return rc;
}
EXPORT_SYMBOL(msm_gpios_request_enable);

int msm_gpios_request(const struct msm_gpio *table, int size)
{
	int i, rc;
	for (i = 0; i < size; i++) {
		rc = gpio_request(GPIO_PIN(table[i].gpio_cfg), table[i].label);
		if (rc)
			goto err;
	}

	return 0;
err:
	for (i--; i >= 0;i--)
		gpio_free(GPIO_PIN(table[i].gpio_cfg));
	return rc;
}
EXPORT_SYMBOL(msm_gpios_request);

void msm_gpios_disable_free(const struct msm_gpio *table, int size)
{
	msm_gpios_disable(table, size);
}
EXPORT_SYMBOL(msm_gpios_disable_free);

#define NUM_GPIO_INT_REGISTERS 6
#define GPIO_SMEM_NUM_GROUPS 2
#define GPIO_SMEM_MAX_PC_INTERRUPTS 8
struct tramp_gpio_smem
{
	uint16_t num_fired[GPIO_SMEM_NUM_GROUPS];
	uint16_t fired[GPIO_SMEM_NUM_GROUPS][GPIO_SMEM_MAX_PC_INTERRUPTS];
	uint32_t enabled[NUM_GPIO_INT_REGISTERS];
	uint32_t detection[NUM_GPIO_INT_REGISTERS];
	uint32_t polarity[NUM_GPIO_INT_REGISTERS];
};

static void msm_gpio_sleep_int(unsigned long arg)
{
	int i, j;
	struct tramp_gpio_smem *smem_gpio;
	BUILD_BUG_ON(ARRAY_SIZE(msm_gpio_chips) != ARRAY_SIZE(smem_gpio->enabled));
	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio)); 
	if (smem_gpio == NULL)
		return;

	local_irq_disable();
	for(i = 0; i < GPIO_SMEM_NUM_GROUPS; i++) {
		int count = smem_gpio->num_fired[i];
		smem_gpio->num_fired[i]=0;
		for(j = 0; j < count; j++) {
			/* TODO: Check mask */
//			printk("msm_gpio_sleep_int %d\n",smem_gpio->fired[i][j]);
			generic_handle_irq(MSM_GPIO_TO_INT(smem_gpio->fired[i][j]));
			smem_gpio->fired[i][j]=0;
		}
	}
	local_irq_enable();
}

static DECLARE_TASKLET(msm_gpio_sleep_int_tasklet, msm_gpio_sleep_int, 0);

void msm_gpio_enter_sleep(int from_idle)
{
	int i,j;
	struct tramp_gpio_smem *smem_gpio;

//	for(i=0x6f;i<0x7a;i++)
//		msm_gpio_set_function(DEX_GPIO_CFG(i,0,GPIO_OUTPUT,GPIO_NO_PULL,GPIO_2MA,0));
//	for(i=0x20;i<0x24;i++)
//		msm_gpio_set_function(DEX_GPIO_CFG(i,0,GPIO_OUTPUT,GPIO_NO_PULL,GPIO_2MA,0));

	BUILD_BUG_ON(ARRAY_SIZE(msm_gpio_chips) != ARRAY_SIZE(smem_gpio->enabled));

	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio));
		
	udelay(10);
//	if (msm_gpio_debug_mask & GPIO_DEBUG_SLEEP)
//		dumpgpios();
	if (smem_gpio) {
		for (i = 0; i < ARRAY_SIZE(smem_gpio->enabled); i++) {
			smem_gpio->enabled[i] = 0;
			smem_gpio->detection[i] = 0;
			smem_gpio->polarity[i] = 0;
		}
	}
	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		writel(msm_gpio_chips[i].int_enable[!from_idle], msm_gpio_chips[i].regs.int_en);
		if(!from_idle && msm_gpio_debug_mask&GPIO_DEBUG_SLEEP)
			printk("GPIO %d, int_enable=%x\n", i, msm_gpio_chips[i].int_enable[!from_idle]);
		if (smem_gpio) {
			uint32_t tmp;
			int start, index, shiftl, shiftr;
			start = msm_gpio_chips[i].chip.start;
			index = start / 32;
			shiftl = start % 32;
			shiftr = 32 - shiftl;
			tmp = msm_gpio_chips[i].int_enable[!from_idle];
			smem_gpio->enabled[index] |= tmp << shiftl;
			smem_gpio->enabled[index+1] |= tmp >> shiftr;
			smem_gpio->detection[index] |= (
			readl(msm_gpio_chips[i].regs.int_edge) & tmp) << shiftl;
			smem_gpio->detection[index+1] |= (readl(msm_gpio_chips[i].regs.int_edge) & tmp) >> shiftr;
			smem_gpio->polarity[index] |= (readl(msm_gpio_chips[i].regs.int_pos) & tmp) << shiftl;
			smem_gpio->polarity[index+1] |= (readl(msm_gpio_chips[i].regs.int_pos) & tmp) >> shiftr;
		}
		
	}

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
		for(i = 0; i < GPIO_SMEM_NUM_GROUPS; i++) {
			smem_gpio->num_fired[i] = 0;
			for(j=0;j<GPIO_SMEM_MAX_PC_INTERRUPTS;j++)
				smem_gpio->fired[i][j]=0;
		}
			
	}
}

void msm_gpio_exit_sleep(void)
{
	int i;
	struct tramp_gpio_smem *smem_gpio;

	BUILD_BUG_ON(ARRAY_SIZE(msm_gpio_chips) != ARRAY_SIZE(smem_gpio->enabled));

	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio)); 

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		writel(msm_gpio_chips[i].int_enable[0], msm_gpio_chips[i].regs.int_en);
	}
	if (smem_gpio && (smem_gpio->num_fired[0] || smem_gpio->num_fired[1])) {
		if (msm_gpio_debug_mask & GPIO_DEBUG_SLEEP)
			printk(KERN_INFO "gpio: fired %x %x\n",
			      smem_gpio->num_fired[0], smem_gpio->num_fired[1]);
		tasklet_schedule(&msm_gpio_sleep_int_tasklet);
	}
//	msm_gpio_set_function(DEX_GPIO_CFG(97,1,GPIO_INPUT,GPIO_NO_PULL,GPIO_2MA,0));
}

