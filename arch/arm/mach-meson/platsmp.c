#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <asm/smp_scu.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <linux/reset.h>
#include <linux/reset-controller.h>

#define MESON_CPU_CTRL_ADDR_REG(c)	(0x04 + ((c - 1) << 2))
#define MESON_CPU_CTRL_REG		(0x00)
#define MESON_CPU_POWER_CTRL_REG	(0x08)

#define MESON_CPU_CTRL_ID(cpu)		((1 << (cpu)) | 1)

static void __iomem *cpucfg_membase;
static void __iomem *scu_membase;
static struct device_node *node;

static DEFINE_SPINLOCK(cpu_lock);

extern void meson8b_secondary_startup(void);

static void __init meson8b_smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int i;

	node = of_find_compatible_node(NULL, NULL, "arm,cortex-a5-scu");
	if (!node) {
		pr_err("Missing Meson6 SCU node\n");
		return;
	}

	scu_membase = of_iomap(node, 0);
	if (!scu_membase) {
		pr_err("Couln't map Meson6 SCU registers\n");
		return;
	}

	node = of_find_compatible_node(NULL, NULL, "amlogic,meson8b-cpuconfig");
	if (!node) {
		pr_err("Missing Meson6 CPU config node\n");
		return;
	}

	cpucfg_membase = of_iomap(node, 0);
	if (!cpucfg_membase) {
		pr_err("Couldn't map Meson6 CPU config registers\n");
		return;
	}

	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	scu_enable(scu_membase);
}

static int meson8b_smp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	struct reset_control *reset_ctrl;
	char cpu_str[4];

	if (!cpucfg_membase)
		return -EFAULT;

	sprintf(cpu_str, "CPU%d", cpu);
	reset_ctrl = of_reset_control_get(node, cpu_str);

	spin_lock(&cpu_lock);

	writel(virt_to_phys(meson8b_secondary_startup), cpucfg_membase + MESON_CPU_CTRL_ADDR_REG(cpu));
	writel(MESON_CPU_CTRL_ID(cpu), cpucfg_membase + MESON_CPU_CTRL_REG);

	smp_wmb();
	mb();
	dsb_sev();

	spin_unlock(&cpu_lock);

	return 0;
}

void __init meson8b_smp_init_cpus(void)
{
	unsigned int i;

	for (i = 0; i < NR_CPUS; i++)
		set_cpu_possible(i, true);
}

static struct smp_operations meson8b_smp_ops __initdata = {
	.smp_init_cpus          = meson8b_smp_init_cpus,
	.smp_prepare_cpus	= meson8b_smp_prepare_cpus,
	.smp_boot_secondary	= meson8b_smp_boot_secondary,
};

CPU_METHOD_OF_DECLARE(meson8b_smp, "amlogic,meson8b-smp", &meson8b_smp_ops);
