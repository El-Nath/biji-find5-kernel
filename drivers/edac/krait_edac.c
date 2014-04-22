/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/krait-l2-accessors.h>

#include "edac_core.h"

#define CESR_DCTPE		BIT(0)
#define CESR_DCDPE		BIT(1)
#define CESR_ICTPE		BIT(2)
#define CESR_ICDPE		BIT(3)
#define CESR_DCTE		(BIT(4) | BIT(5))
#define CESR_ICTE		(BIT(6) | BIT(7))
#define CESR_TLBMH		BIT(16)
#define CESR_I_MASK		0x000000cc
/* Print a message for everything but TLB MH events */
#define CESR_PRINT_MASK		0x000000ff

#define L2ESR			0x204
#define L2ESR_MPDCD		BIT(0)
#define L2ESR_MPSLV		BIT(1)
#define L2ESR_TSESB		BIT(2)
#define L2ESR_TSEDB		BIT(3)
#define L2ESR_DSESB		BIT(4)
#define L2ESR_DSEDB		BIT(5)
#define L2ESR_MSE		BIT(6)
#define L2ESR_MPLDREXNOK	BIT(8)
#define L2ESR_CPU_MASK		0xf
#define L2ESR_CPU_SHIFT		16
#define L2ESR_SP		BIT(20)

#define L2ESYNR0		0x208
#define L2ESYNR1		0x209
#define L2EAR0			0x20c
#define L2EAR1			0x20d

struct krait_edac {
	int l1_irq;
	struct edac_device_ctl_info * __percpu *edev;
	struct notifier_block notifier;
};

struct krait_edac_error {
	const char * const msg;
	void (*func)(struct edac_device_ctl_info *edac_dev,
			int inst_nr, int block_nr, const char *msg);
};

static unsigned int read_cesr(void)
{
	unsigned int cesr;

	asm volatile ("mrc p15, 7, %0, c15, c0, 1 @ cesr" : "=r" (cesr));
	return cesr;
}

static void write_cesr(unsigned int cesr)
{
	asm volatile ("mcr p15, 7, %0, c15, c0, 1 @ cesr" : : "r" (cesr));
}

static unsigned int read_cesynr(void)
{
	unsigned int cesynr;

	asm volatile ("mrc p15, 7, %0, c15, c0, 3 @ cesynr" : "=r" (cesynr));
	return cesynr;
}

static irqreturn_t krait_l1_irq(int irq, void *dev_id)
{
	struct edac_device_ctl_info **edac_p = dev_id;
	struct edac_device_ctl_info *edac = *edac_p;
	unsigned int cesr = read_cesr();
	unsigned int i_cesynr, d_cesynr;
	unsigned int cpu = smp_processor_id();
	int print_regs = cesr & CESR_PRINT_MASK;
	int i;
	static const struct krait_edac_error errors[] = {
		{ "D-cache tag parity error", edac_device_handle_ue },
		{ "D-cache data parity error", edac_device_handle_ue },
		{ "I-cache tag parity error", edac_device_handle_ce },
		{ "I-cache data parity error", edac_device_handle_ce },
		{ "D-cache tag timing error", edac_device_handle_ue },
		{ "D-cache data timing error", edac_device_handle_ue },
		{ "I-cache tag timing error", edac_device_handle_ce },
		{ "I-cache data timing error", edac_device_handle_ce }
	};

	if (print_regs) {
		pr_alert("L1 / TLB Error detected on CPU %d!\n", cpu);
		pr_alert("CESR      = 0x%08x\n", cesr);
	}

	for (i = 0; i < ARRAY_SIZE(errors); i++)
		if (BIT(i) & cesr)
			errors[i].func(edac, cpu, 0, errors[i].msg);

	if (cesr & CESR_TLBMH) {
		asm ("mcr p15, 0, r0, c8, c7, 0");
		edac_device_handle_ce(edac, cpu, 0, "TLB Multi-Hit error");
	}

	if (cesr & (CESR_ICTPE | CESR_ICDPE | CESR_ICTE)) {
		i_cesynr = read_cesynr();
		pr_alert("I-side CESYNR = 0x%08x\n", i_cesynr);
		write_cesr(CESR_I_MASK);

		/*
		 * Clear the I-side bits from the captured CESR value so that we
		 * don't accidentally clear any new I-side errors when we do
		 * the CESR write-clear operation.
		 */
		cesr &= ~CESR_I_MASK;
	}

	if (cesr & (CESR_DCTPE | CESR_DCDPE | CESR_DCTE)) {
		d_cesynr = read_cesynr();
		pr_alert("D-side CESYNR = 0x%08x\n", d_cesynr);
	}

	/* Clear the interrupt bits we processed */
	write_cesr(cesr);

	return IRQ_HANDLED;
}

static irqreturn_t krait_l2_irq(int irq, void *dev_id)
{
	struct edac_device_ctl_info *edac = dev_id;
	unsigned int l2esr;
	unsigned int l2esynr0;
	unsigned int l2esynr1;
	unsigned int l2ear0;
	unsigned int l2ear1;
	unsigned long cpu;
	int i;
	static const struct krait_edac_error errors[] = {
		{ "master port decode error", edac_device_handle_ce },
		{ "master port slave error", edac_device_handle_ce },
		{ "tag soft error, single-bit", edac_device_handle_ce },
		{ "tag soft error, double-bit", edac_device_handle_ue },
		{ "data soft error, single-bit", edac_device_handle_ce },
		{ "data soft error, double-bit", edac_device_handle_ue },
		{ "modified soft error", edac_device_handle_ce },
		{ "slave port exclusive monitor not available",
			edac_device_handle_ue},
		{ "master port LDREX received Normal OK response",
			edac_device_handle_ce },
	};

	l2esr = krait_get_l2_indirect_reg(L2ESR);
	pr_alert("Error detected!\n");
	pr_alert("L2ESR    = 0x%08x\n", l2esr);

	if (l2esr & (L2ESR_TSESB | L2ESR_TSEDB | L2ESR_MSE | L2ESR_SP)) {
		l2esynr0 = krait_get_l2_indirect_reg(L2ESYNR0);
		l2esynr1 = krait_get_l2_indirect_reg(L2ESYNR1);
		l2ear0 = krait_get_l2_indirect_reg(L2EAR0);
		l2ear1 = krait_get_l2_indirect_reg(L2EAR1);

		pr_alert("L2ESYNR0 = 0x%08x\n", l2esynr0);
		pr_alert("L2ESYNR1 = 0x%08x\n", l2esynr1);
		pr_alert("L2EAR0   = 0x%08x\n", l2ear0);
		pr_alert("L2EAR1   = 0x%08x\n", l2ear1);
	}

	cpu = (l2esr >> L2ESR_CPU_SHIFT) & L2ESR_CPU_MASK;
	cpu = __ffs(cpu);
	if (cpu)
		cpu--;
	for (i = 0; i < ARRAY_SIZE(errors); i++)
		if (BIT(i) & l2esr)
			errors[i].func(edac, cpu, 1, errors[i].msg);

	krait_set_l2_indirect_reg(L2ESR, l2esr);

	return IRQ_HANDLED;
}

static void enable_l1_irq(void *info)
{
	const struct krait_edac *k = info;

	enable_percpu_irq(k->l1_irq, IRQ_TYPE_LEVEL_HIGH);
}

static void disable_l1_irq(void *info)
{
	const struct krait_edac *k = info;

	disable_percpu_irq(k->l1_irq);
}

static int
krait_edac_notify(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	struct krait_edac *p = container_of(nfb, struct krait_edac, notifier);

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		enable_l1_irq(p);
		break;

	case CPU_DYING:
		disable_l1_irq(p);
		break;
	}
	return NOTIFY_OK;
}

static int krait_edac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct edac_device_ctl_info *edev;
	struct krait_edac *p;
	int l1_irq, l2_irq;
	int ret, cpu;
	struct device_node *cpunode, *l1node, *l2node;

	cpunode = of_get_cpu_node(0, NULL);
	if (!cpunode)
		return -ENODEV;

	l1node = of_parse_phandle(cpunode, "next-level-cache", 0);
	of_node_put(cpunode);
	if (!l1node)
		return -ENODEV;

	l1_irq = irq_of_parse_and_map(l1node, 0);
	l2node = of_parse_phandle(l1node, "next-level-cache", 0);
	of_node_put(l1node);
	if (!l2node)
		return -ENODEV;

	l2_irq = irq_of_parse_and_map(l2node, 0);
	of_node_put(l2node);

	if (l1_irq < 0)
		return l1_irq;
	if (l2_irq < 0)
		return l2_irq;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	platform_set_drvdata(pdev, p);
	p->l1_irq = l1_irq;

	p->edev = alloc_percpu(struct edac_device_ctl_info *);
	if (!p->edev)
		return -ENOMEM;

	edev = edac_device_alloc_ctl_info(0, "cpu", num_possible_cpus(),
						 "L", 2, 1, NULL, 0,
						 edac_device_alloc_index());
	if (!edev) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	edev->dev = dev;
	edev->mod_name = dev_name(dev);
	edev->dev_name = dev_name(dev);
	edev->ctl_name = "cache";

	for_each_possible_cpu(cpu)
		*per_cpu_ptr(p->edev, cpu) = edev;

	ret = edac_device_add_device(edev);
	if (ret)
		goto err_add;

	ret = request_percpu_irq(l1_irq, krait_l1_irq, "L1 err",
				p->edev);
	if (ret)
		goto err_l1_irq;

	ret = devm_request_irq(dev, l2_irq, krait_l2_irq, 0, "L2 err",
				edev);
	if (ret)
		goto err_l2_irq;

	p->notifier.notifier_call = krait_edac_notify;
	register_hotcpu_notifier(&p->notifier);
	on_each_cpu(enable_l1_irq, p, true);

	return 0;
err_l2_irq:
	free_percpu_irq(p->l1_irq, p->edev);
err_l1_irq:
	edac_device_del_device(dev);
err_add:
	edac_device_free_ctl_info(edev);
err_alloc:
	free_percpu(p->edev);
	return ret;
}

static int krait_edac_remove(struct platform_device *pdev)
{
	struct krait_edac *p = platform_get_drvdata(pdev);

	unregister_hotcpu_notifier(&p->notifier);
	on_each_cpu(disable_l1_irq, p, true);
	free_percpu_irq(p->l1_irq, p->edev);
	edac_device_del_device(&pdev->dev);
	edac_device_free_ctl_info(*__this_cpu_ptr(p->edev));
	free_percpu(p->edev);

	return 0;
}

static struct platform_driver krait_edac_driver = {
	.probe = krait_edac_probe,
	.remove = krait_edac_remove,
	.driver = {
		.name = "krait_edac",
		.owner = THIS_MODULE,
	},
};

static struct platform_device *krait_edacp;

static int __init krait_edac_driver_init(void)
{
	struct device_node *np;

	np = of_get_cpu_node(0, NULL);
	if (!np)
		return 0;

	if (!krait_edacp && of_device_is_compatible(np, "qcom,krait"))
		krait_edacp = of_platform_device_create(np, "krait_edac", NULL);
	of_node_put(np);

	return platform_driver_register(&krait_edac_driver);
}
module_init(krait_edac_driver_init);

static void __exit krait_edac_driver_exit(void)
{
	platform_driver_unregister(&krait_edac_driver);
	platform_device_unregister(krait_edacp);
}
module_exit(krait_edac_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Krait CPU cache error reporting driver");
MODULE_ALIAS("platform:krait_edac");
