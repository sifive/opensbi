/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Atish Patra <atish.patra@wdc.com>
 */

#include <libfdt.h>
#include <fdt.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_platform.h>
#include <sbi/riscv_io.h>
#include <sbi_utils/serial/sifive-uart.h>
#include "plic.h"
#include "clint.h"
#include "platform.h"

/* clang-format off */

#define VCU118_TLCLK				100000000

#define VCU118_CLINT_ADDR			0x2000000

#define VCU118_PLIC_ADDR			0xc000000
#define VCU118_PLIC_NUM_SOURCES			45
#define VCU118_PLIC_NUM_PRIORITIES		7

#define VCU118_UART0_ADDR			0x10010000
#define VCU118_UART_BAUDRATE			115200

/**
 * Hawksbill project consist ofs two VCU118 boards.
 * Each VCU118 has 5 HARTs but HART ID 0 doesn't have S mode.
 */
#define VCU118_HARITD_DISABLED			(1 << 0 | 1 << 8 | 1 << 16 | 1 << 24)
#define VCU118_HART_COUNT			(HAWKSBILL_MAX_NODE_NUM * HAWKSBILL_MAX_HART_PER_NODE)
#define VCU118_HART_STACK_SIZE			8192

/* clang-format on */

static void vcu118_modify_dt(void *fdt)
{
	u32 i, size;
	int chosen_offset, err, numa_offset;
	int cpu_offset;
	char cpu_node[32] = "";
	const char *mmu_type;

	size = fdt_totalsize(fdt);
	err  = fdt_open_into(fdt, fdt, size + 256);
	if (err < 0)
		sbi_printf(
			"Device Tree can't be expanded to accmodate new node");

	for (i = 0; i < VCU118_HART_COUNT; i++) {
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

	chosen_offset = fdt_path_offset(fdt, "/chosen");
	fdt_setprop_string(fdt, chosen_offset, "serial0",
			   "/soc/serial@10010000:115200");

	vcu118_plic_fdt_fixup(fdt, "sifive,plic-1.0.0");

	// Eliminate all devices/cpus/memory with a distance > hawksbill_node_num
	for (numa_offset = fdt_next_node(fdt, -1, NULL);
	     numa_offset >= 0;
	     numa_offset = fdt_next_node(fdt, numa_offset, NULL))
	{
		int len;
		const u32 *val;

		val = fdt_getprop(fdt, numa_offset, "numa-node-id", &len);

		if (val && len == 4 && fdt32_to_cpu(*val) >= hawksbill_node_num()) {
			fdt_del_node(fdt, numa_offset);
			numa_offset = -1;
		}
	}
}

static int vcu118_final_init(bool cold_boot)
{
	void *fdt;

	if (!cold_boot)
		return 0;

	fdt = sbi_scratch_thishart_arg1_ptr();
	vcu118_modify_dt(fdt);

	return 0;
}

static u32 vcu118_pmp_region_count(u32 hartid)
{
	return 1;
}

static int vcu118_pmp_region_info(u32 hartid, u32 index, ulong *prot,
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

static int vcu118_console_init(void)
{
	return sifive_uart_init(VCU118_UART0_ADDR, VCU118_TLCLK,
				VCU118_UART_BAUDRATE);
}

static int vcu118_irqchip_init(bool cold_boot)
{
	int rc;
	u32 hartid = sbi_current_hartid();

	if (cold_boot) {
		rc = vcu118_plic_cold_irqchip_init(VCU118_PLIC_ADDR,
						   VCU118_PLIC_NUM_SOURCES,
						   VCU118_HART_COUNT);
		if (rc)
			return rc;
	}

	return vcu118_plic_warm_irqchip_init(hartid);
}

static int vcu118_ipi_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = vcu118_clint_cold_ipi_init(VCU118_CLINT_ADDR,
						VCU118_HART_COUNT);
		if (rc)
			return rc;
	}

	return vcu118_clint_warm_ipi_init();
}

static int vcu118_timer_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = vcu118_clint_cold_timer_init(VCU118_CLINT_ADDR,
						  VCU118_HART_COUNT);
		if (rc)
			return rc;
	}

	return vcu118_clint_warm_timer_init();
}

static int vcu118_system_down(u32 type)
{
	/* For now nothing to do. */
	return 0;
}

const struct sbi_platform_operations platform_ops = {
	.pmp_region_count  = vcu118_pmp_region_count,
	.pmp_region_info   = vcu118_pmp_region_info,
	.final_init	   = vcu118_final_init,
	.console_putc      = sifive_uart_putc,
	.console_getc      = sifive_uart_getc,
	.console_init      = vcu118_console_init,
	.irqchip_init      = vcu118_irqchip_init,
	.ipi_send	   = vcu118_clint_ipi_send,
	.ipi_clear	   = vcu118_clint_ipi_clear,
	.ipi_init	   = vcu118_ipi_init,
	.timer_value       = vcu118_clint_timer_value,
	.timer_event_stop  = vcu118_clint_timer_event_stop,
	.timer_event_start = vcu118_clint_timer_event_start,
	.timer_init	   = vcu118_timer_init,
	.system_reboot     = vcu118_system_down,
	.system_shutdown   = vcu118_system_down
};

const struct sbi_platform platform = {
	.opensbi_version    = OPENSBI_VERSION,
	.platform_version   = SBI_PLATFORM_VERSION(0x0, 0x01),
	.name		    = "SiFive VCU118",
	.features	    = SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count	    = VCU118_HART_COUNT,
	.hart_stack_size    = VCU118_HART_STACK_SIZE,
	.disabled_hart_mask = VCU118_HARITD_DISABLED,
	.platform_ops_addr  = (unsigned long)&platform_ops
};

/* Packet routing control (in bytes) */
#define PP_TX_BASE		0x100a0000UL
#define PP_TX_HEADER_INSERT	0x00
#define PP_TX_HEADER_DATA	0x40
#define PP_RX_BASE		0x100b0000UL
#define PP_RX_HEADER_STRIP	0x00
#define	PP_RX_HEADER_MASK	0x08
#define PP_RX_HEADER_DATA	0x40
#define PP_VCX			0x80

#define ETHERNET_HEADER		14

/* Addressing control (in bytes) */
#define AC_BASE			0x10070000UL
#define AC_DIPS			0
#define AC_HART_PREFIX		8
#define AC_C_PREFIX		16
#define AC_M_PREFIX		24

/* TLoE control (in bytes) */
#define TL_C_BASE	0x10080000UL // cbus
#define TL_M_BASE	0x10090000UL // mbus
#define TL_ROUTEX	0x10
#define TL_ROUTE_BASE	0
#define TL_ROUTE_MASK	8
#define TL_VCX		0x1000
#define TL_VC_ENABLE	0
#define TL_VC_VC	4
#define TL_VC_FLIT	8

/* L2 control (in bytes) */
#define L2_BASE			0x2010000UL
#define L2_FLUSH64		0x200

/* UART (in bytes) */
#define UART_BASE		0x10010000UL

/* Hacky register pokers */
#define SQ(x, v)	(*(volatile uint64_t*)(x)) = (v)
#define SW(x, v)	(*(volatile uint32_t*)(x)) = (v)
#define SB(x, v)	(*(volatile uint8_t *)(x)) = (v)
#define LB(x)		(*(volatile uint8_t *)(x))

static void set_header(uint64_t base, uint8_t *header) {
	for (int i = 0; i < ETHERNET_HEADER; ++i)
		SB(base + i, header[i]);
}

#ifdef DEBUG
static void local_printc(char ch, uintptr_t x) {
    int32_t r;
    do {
      __asm__ __volatile__ (
        "amoor.w %0, %2, %1\n"
        : "=r" (r), "+A" (*(uint32_t*)(x + UART_BASE))
        : "r" (ch));
    } while (r < 0);
}
#define DEBUGC(c)	local_printc(c, 0)
#define DEBUGC2(c, x)	local_printc(c, x)
#else
#define DEBUGC(c)	do { } while (0)
#define DEBUGC2(c, x)	do { } while (0)
#endif

// !!! This next bit is extremely unsafe. We are about to relocate DDR ... where we are running!
// Fortunately, for our initial devwork, the chips have identical memory layout
// ... we would ideally execute the next bit (flush, move, flush, enable) from a local SRAM/ROM
// It is vital that this function fit inside a single cache line so we don't get stuck.
static void __attribute__ ((noinline)) flip_memory(uintptr_t flush64, uintptr_t prefix, uintptr_t val, uintptr_t first_enable, uintptr_t last_enable) {
#define STR1(s) #s
#define STR2(s) STR1(s)
	__asm__ __volatile__ (
		// Make sure the code from here fits into one I$ block
		".align 6\n\t"
		// Flush L2 cache (write-back all dirty data)
		// From here until the next flush, no stores to memory are allowed
		"call	_flush\n\t"
		// This switches main memory, essentially rendering everything invalid
		"sd	%2, (%1)\n\t"			// *prefix = val
		"fence\n\t"
		// Flush L2 cache (invalidate anything cached since last flush)
		"call	_flush\n\t"

		// Caches are now consistent with new memory map and routes are established; allow TLoE traffic
		// This transition (invalid => valid) does not need a barrier. TL forbids caching invalid.
		// Turn on all the other mbusses (zero(1) is already enabled)
		"li	t1, " STR2(TL_VCX) "\n\t"
		"li	t0, 1\n\t"
		"1:\n\t"
		"sw	t0, (%3)\n\t"			// *enable = 1
		"add	%3, %3, t1\n\t"			// enable += TL_VCX
		"bne	%3, %4, 1b\n\t"

		"fence.i\n\t"
		"j 	2f\n\t"

		"_flush:\n\t"
		"li	t0, 0x2000000000\n\t"
		"li	t1, 0x2080000000\n\t"
		"1:\n\t"
		"sd	t0, (%0)\n\t"			// *flush64 = address
		"addi	t0, t0, 64\n\t"
		"bne	t0, t1, 1b\n\t"

		"fence\n\t"
		"ret\n\t"

		"2:\n\t"
	: : "r"(flush64), "r"(prefix), "r"(val), "r"(first_enable), "r"(last_enable) : "t0", "t1", "ra");
}

static int his_id(int chan, int my_id) {
	if (chan < my_id) {
		return chan;
	} else {
		return chan+1;
	}
}

static int node_num;
int hawksbill_node_num(void)
{
	return node_num;
}

void omnixtend(void)
{
	const uint8_t mac[6] = { 0x68, 0x05, 0xCA, 0x88, 0x63, 0x00 /* chip */ };
	const uint8_t eth[2] = { 0xAA, 0xAA };

	// Use DIP switch=0 to determine chip ID
	uint8_t dip   = LB(AC_BASE+AC_DIPS);
	int     id    = dip & 3;
	int     nchan = (dip >> 2) & 3;
	DEBUGC('0' + id);
	DEBUGC('0' + nchan);
	node_num = nchan+1;

	/************************ Configure TLoE ethernet RX/TX **************************/

	// Ethernet packets: <6 bytes dest MAC> <6 bytes sender MAC> <2 bytes ethernet type>
	uint8_t header[ETHERNET_HEADER];
	memcpy(&header[ 0], mac, 6);
	memcpy(&header[ 6], mac, 6);
	memcpy(&header[12], eth, 2);

	// Setup TX packet framing/mux
	for (int chan = 0; chan < nchan; ++chan) {
		int hid = his_id(chan, id);
		int instance;

		// TX framing
		header[5]  = hid; // dest   MAC
		header[11] = id;  // sender MAC

		for (int vc = 0; vc < 2; ++vc) { // VC0=cbus VC1=mbus
			instance = chan + ((HAWKSBILL_MAX_NODE_NUM-1) * vc);
			set_header(PP_TX_BASE + (PP_VCX*instance) + PP_TX_HEADER_DATA, header);
			SQ(PP_TX_BASE + (PP_VCX*instance) + PP_TX_HEADER_INSERT, ETHERNET_HEADER);
		}

		// Setup RX packet deframing/demux
		header[5]  = id;  // dest   MAC
		header[11] = hid; // sender MAC
		for (int vc = 0; vc < 2; ++vc) {
			instance = chan + ((HAWKSBILL_MAX_NODE_NUM-1) * vc);
			set_header(PP_RX_BASE + (PP_VCX*instance) + PP_RX_HEADER_DATA, header);
			SQ(PP_RX_BASE + (PP_VCX*instance) + PP_RX_HEADER_STRIP, ETHERNET_HEADER);
			// Match on sender+dest MAC and also the VC field in the TLoE header
			SB(PP_RX_BASE + (PP_VCX*instance) + PP_RX_HEADER_DATA + ETHERNET_HEADER, vc << 5);
			SQ(PP_RX_BASE + (PP_VCX*instance) + PP_RX_HEADER_MASK, (1 << (ETHERNET_HEADER+1)) - 1);
		}

		// Mark packets with VC in TLoE header
		SW(TL_C_BASE+(TL_VCX*(1+chan))+TL_VC_VC, 0);
		SW(TL_M_BASE+(TL_VCX*(1+chan))+TL_VC_VC, 1);
	}

	/************************ Configure TLoE remote regions **************************/

	DEBUGC('d');
	for (int chan = 0; chan < nchan; ++chan) {
		int hid = his_id(chan, id);
		DEBUGC('0' + hid);
		uintptr_t c = ((uint64_t)hid) << 32;
		uintptr_t m = ((uint64_t)hid) << 32;
		uintptr_t mask = 0x7fffffffU;
		SQ(TL_C_BASE + (chan*TL_ROUTEX) + TL_ROUTE_BASE, c);
		SQ(TL_C_BASE + (chan*TL_ROUTEX) + TL_ROUTE_MASK, mask);
		SQ(TL_M_BASE + (chan*TL_ROUTEX) + TL_ROUTE_BASE, m + 0x2000000000UL);
		SQ(TL_M_BASE + (chan*TL_ROUTEX) + TL_ROUTE_MASK, mask);
	}

	/************************ Reposition local memory ********************************/
	uintptr_t c = ((uint64_t)id) << 32;
	uintptr_t m = ((uint64_t)id) << 32;

	// Set our hartid base to 8*id (ie: if we were hartid=0, we are now 8).
	DEBUGC('h');
	SQ(AC_BASE+AC_HART_PREFIX, id * HAWKSBILL_MAX_HART_PER_NODE);

	// This moves the MMIO space; after this line, local devices are re-positioned with offset c
	DEBUGC('c');
	asm volatile ("fence");
	SQ(AC_BASE+AC_C_PREFIX, c);
	asm volatile ("fence");

	// Enable access to all remote cbusses
	for (int chan = 0; chan < nchan; ++chan)
		SW(c+TL_C_BASE+(TL_VCX*(1+chan))+TL_VC_ENABLE, 1);

	DEBUGC('0' + id); // indicate access to board 0

	// Move local memory; effectively like a DDR-wide DMA transfer
	DEBUGC2('m', c);
	if (nchan > 0) {
		flip_memory(
			c+L2_BASE+L2_FLUSH64,
			c+AC_BASE+AC_M_PREFIX,
			m,
			c+TL_M_BASE+(1      )*TL_VCX+TL_VC_ENABLE,
			c+TL_M_BASE+(1+nchan)*TL_VCX+TL_VC_ENABLE);
	}
}
