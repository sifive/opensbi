/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Sifive Corporation.
 *
 * Authors:
 *   Zong Li <zong.li@sifive.com>
 */

#include <sbi/sbi_platform.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_console.h>
#include <sbi_utils/irqchip/plic.h>
#include <libfdt.h>

/* Freedom Metal */
#include <machine.h>
#include <metal/tty.h>

/* clang-format off */

/*
 * Default is running on U-series SoC. There are 5 HARTs, only U-cores have
 * S mode. Enable only HARTs 1 to 4.
 * Eventually, these information should be available from device tree.
 */
#define METAL_HART_DISABLED		~(1 << 1 | 1 << 2 | 1 << 3 | 1 << 4)
#define METAL_HART_STACK_SIZE		8192
#define METAL_HART_COUNT		__METAL_DT_MAX_HARTS
#define METAL_PLIC_NDEV			METAL_RISCV_PLIC0_C000000_RISCV_NDEV

#define METAL_THIS_CPU			metal_cpus[sbi_current_hartid()]
struct metal_cpu *metal_cpus[METAL_HART_COUNT];

/* clang-format on */

static void metal_modify_dt(void *fdt)
{
	u32 i, size;
	int cpu_offset, err;
	int chosen_offset;
	char cpu_node[32] = "";
	const char *mmu_type;

	/* Expand FDT size for patching */
	size = fdt_totalsize(fdt);
	err  = fdt_open_into(fdt, fdt, size + 256);
	if (err < 0)
		sbi_printf(
			"Device Tree can't be expanded to accmodate new node");

	/*
         * Disable the hart which has no mmu. It needs to consider no-mmu Linux
         * in the future.
         */
	for (i = 0; i < METAL_HART_COUNT; i++) {
		sbi_sprintf(cpu_node, "/cpus/cpu@%d", i);
		cpu_offset = fdt_path_offset(fdt, cpu_node);
		mmu_type   = fdt_getprop(fdt, cpu_offset, "mmu-type", NULL);
		if (mmu_type && (!strcmp(mmu_type, "riscv,sv39") ||
				 !strcmp(mmu_type, "riscv,sv48")))
			continue;
		else
			fdt_setprop_string(fdt, cpu_offset, "status",
					   "disabled");
		memset(cpu_node, 0, sizeof(cpu_node));
	}

	/* Specify stdout device */
	chosen_offset = fdt_path_offset(fdt, "/chosen");
	fdt_setprop_string(fdt, chosen_offset, "stdout-path",
			   "/soc/serial@10010000:115200");

	/* Mask machine mode external interrupt */
	plic_fdt_fixup(fdt, "riscv,plic0");
}

static int metal_early_init(bool cold_boot)
{
	/* Initialize metal_cpu for quickly access later. */
	u32 hartid	 = sbi_current_hartid();
	metal_cpus[hartid] = metal_cpu_get(hartid);

	/*
	 * The CPU interrupt controller is the top of the interrupt heirarchy.
	 * It must be initialized before any other interrupt controllers are
	 * initialized.
	 */
	if (cold_boot) {
		struct metal_interrupt *cpu_int =
			metal_cpu_interrupt_controller(metal_cpus[hartid]);
		if (!cpu_int) {
			return -1;
		}
		metal_interrupt_init(cpu_int);
	}

	return 0;
}

static int metal_final_init(bool cold_boot)
{
	void *fdt;

	if (!cold_boot)
		return 0;

	fdt = sbi_scratch_thishart_arg1_ptr();
	metal_modify_dt(fdt);

	return 0;
}

static u32 metal_pmp_region_count(u32 hartid)
{
	return metal_pmp_num_regions(hartid);
}

static int metal_pmp_region_info(u32 hartid, u32 index, ulong *prot,
				 ulong *addr, ulong *log2size)
{
	int ret = 0;

	switch (index) {
	case 0:
		*prot     = PMP_R | PMP_W | PMP_X;
		*addr     = 0;
		*log2size = __riscv_xlen;
		break;
	default:
		ret = -1;
		break;
	};

	return ret;
}

static void metal_console_putc(char ch)
{
	metal_tty_putc((int)ch);
}

static int metal_console_getc(void)
{
	int ch;
	metal_tty_getc(&ch);
	return ch;
}

static int metal_console_init(void)
{
	/* TTY constructor */
	return 0;
}

static int metal_get_hart_context_id(int mode)
{
	u32 hartid = sbi_current_hartid();

	/* TODO: these should change to use device tree. */
	if (mode == IRQ_M_EXT)
		return (2 * hartid - 1);
	else if (mode == IRQ_S_EXT)
		return (2 * hartid);
	else
		return -1;
}

#define metal_get_affinity_mask(mode) (1 << metal_get_hart_context_id(mode))

static int metal_irqchip_init(bool cold_boot)
{
	struct metal_interrupt *plic_int =
		metal_interrupt_get_controller(METAL_PLIC_CONTROLLER, 0);
	if (!plic_int)
		return -1;

	if (cold_boot) {
		/*
		 * It would disable all IRQs and clear all threshold in all
		 * hart contexts.
		 */
		metal_interrupt_init(plic_int);
	} else {
		/* By default, enable M-mode threshold */
		metal_affinity context_mask;
		context_mask.bitmask = metal_get_affinity_mask(IRQ_M_EXT);
		metal_interrupt_affinity_set_threshold(plic_int, context_mask,
						       1);
	}
	return 0;
}

static int metal_ipi_init(bool cold_boot)
{
	if (cold_boot) {
		struct metal_interrupt *sw_int =
			metal_cpu_software_interrupt_controller(METAL_THIS_CPU);
		if (!sw_int)
			return -1;
		metal_interrupt_init(sw_int);
	} else {
		metal_cpu_software_clear_ipi(METAL_THIS_CPU,
					     sbi_current_hartid());
	}

	return 0;
}

static void metal_ipi_send(u32 target_hart)
{
	metal_cpu_software_set_ipi(METAL_THIS_CPU, target_hart);
}

static void metal_ipi_clear(u32 target_hart)
{
	metal_cpu_software_clear_ipi(METAL_THIS_CPU, target_hart);
}

static u64 metal_timer_value(void)
{
	return metal_cpu_get_mtime(METAL_THIS_CPU);
}

static void metal_timer_event_start(u64 next_event)
{
	metal_cpu_set_mtimecmp(METAL_THIS_CPU, next_event);
}

static void metal_timer_event_stop(void)
{
	metal_cpu_set_mtimecmp(METAL_THIS_CPU, -1ULL);
}

static int metal_timer_init(bool cold_boot)
{
	if (cold_boot) {
		struct metal_interrupt *timer_int =
			metal_cpu_timer_interrupt_controller(METAL_THIS_CPU);
		if (!timer_int)
			return -1;
		metal_interrupt_init(timer_int);
	} else {
		metal_cpu_set_mtimecmp(METAL_THIS_CPU, -1ULL);
	}

	return 0;
}

static int metal_system_reboot(u32 type)
{
	sbi_printf("System reboot\n");

	return 0;
}

static int metal_system_shutdown(u32 type)
{
	sbi_printf("System shutdown\n");

	return 0;
}

const struct sbi_platform_operations platform_ops = {
	.early_init	= metal_early_init,
	.final_init	= metal_final_init,
	.pmp_region_count  = metal_pmp_region_count,
	.pmp_region_info   = metal_pmp_region_info,
	.console_putc      = metal_console_putc,
	.console_getc      = metal_console_getc,
	.console_init      = metal_console_init,
	.irqchip_init      = metal_irqchip_init,
	.ipi_send	  = metal_ipi_send,
	.ipi_clear	 = metal_ipi_clear,
	.ipi_init	  = metal_ipi_init,
	.timer_value       = metal_timer_value,
	.timer_event_stop  = metal_timer_event_stop,
	.timer_event_start = metal_timer_event_start,
	.timer_init	= metal_timer_init,
	.system_reboot     = metal_system_reboot,
	.system_shutdown   = metal_system_shutdown
};

const struct sbi_platform platform = {
	.opensbi_version    = OPENSBI_VERSION,
	.platform_version   = SBI_PLATFORM_VERSION(0x0, 0x01),
	.name		    = "SiFive Freedom Metal",
	.features	   = SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count	 = METAL_HART_COUNT,
	.hart_stack_size    = METAL_HART_STACK_SIZE,
	.disabled_hart_mask = METAL_HART_DISABLED,
	.platform_ops_addr  = (unsigned long)&platform_ops
};
