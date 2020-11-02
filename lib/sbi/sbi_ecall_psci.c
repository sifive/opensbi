/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 SiFive, Inc.
 *
 * Based on code originally:
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */

#include <sbi/sbi_domain.h>
#include <sbi/sbi_ecall.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_system.h>
#include <sbi/sbi_version.h>
#include <sbi/sbi_hsm.h>
#include <sbi/sbi_scratch.h>
#include <sbi/psci_err.h>
#include <sbi/riscv_asm.h>

#include <sbi/sbi_console.h> /* XXX */

/* PSCI_VERSION return value.  For PSCI 1.1 support */
#define PSCI_VERSION_VAL		((1 << 16) | 1)

/* PSCI_FEATURES return value */
#define PSCI_USES_EXT_STATEID		(1 << 1)
#define PSCI_USES_OS_INITIATED_MODE	(1 << 0)
#define PSCI_FUNC_SUPPORTED_VAL		(PSCI_USES_EXT_STATEID | \
					 PSCI_USES_OS_INITIATED_MODE)

/* PSCI_FEATURES supported functions */
static unsigned long psci_funcs_impl[] = {
	SBI_EXT_PSCI_PSCI_VERSION,
	SBI_EXT_PSCI_CPU_SUSPEND,
	SBI_EXT_PSCI64_CPU_SUSPEND,
	SBI_EXT_PSCI_CPU_OFF,
	SBI_EXT_PSCI_CPU_ON,
	SBI_EXT_PSCI64_CPU_ON,
	SBI_EXT_PSCI_AFFINITY_INFO,
	SBI_EXT_PSCI64_AFFINITY_INFO,
	SBI_EXT_PSCI_SYSTEM_OFF,
	SBI_EXT_PSCI_SYSTEM_RESET,
	SBI_EXT_PSCI_PSCI_FEATURES,
	SBI_EXT_PSCI_SYSTEM_RESET2,
	SBI_EXT_PSCI64_SYSTEM_RESET2,
};

/* AFFINITY_INFO-related macros*/
#define PSCI_AFF_INST_ON		0x0
#define PSCI_AFF_INST_OFF		0x1
#define PSCI_AFF_INST_ON_PENDING	0x2

/*
 * Map an OpenSBI hart state to the PSCI affinity instance state
 *
 * Used by AFFINITY_INFO
 */
static long map_hart_state_to_psci_aff_inst_state(int state)
{
	/* PSCI doesn't support reasonable error codes here */
	long ret = PSCI_AFF_INST_ON;

	switch (state) {
	case SBI_HART_STOPPED:
		ret = PSCI_AFF_INST_OFF;
		break;
	case SBI_HART_STOPPING:
		ret = PSCI_AFF_INST_OFF;
		break;
	case SBI_HART_STARTING:
		ret = PSCI_AFF_INST_ON_PENDING;
		break;
	case SBI_HART_STARTED:
		ret = PSCI_AFF_INST_ON;
		break;
	}

	return ret;
}

/*
 * Map the return value from sbi_hsm_hart_start() to what PSCI expects
 *
 * Used by CPU_ON
 */
static long map_hart_start_to_psci_cpu_on_state(int state)
{
	long ret = PSCI_ERR_INTERNAL_FAILURE;

	switch (state) {
	case SBI_EINVAL:
		ret = PSCI_ERR_INVALID_PARAMETERS;
		break;
	case SBI_EALREADY:
		ret = PSCI_ERR_ALREADY_ON;
		break;
	case SBI_ENOTSUPP:
	case SBI_EFAIL:
		ret = PSCI_ERR_INTERNAL_FAILURE;
		break;
	case SBI_ERR_INVALID_ADDRESS:
		ret = PSCI_ERR_INVALID_ADDRESS;
		break;
	}

	return ret;
}

static int sbi_ecall_psci_handler(unsigned long extid, unsigned long funcid,
				  unsigned long *args, unsigned long *out_val,
				  struct sbi_trap_info *out_trap)
{
	ulong smode;
	int ret = 0, hstate, i;
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();

	switch (funcid) {
	case SBI_EXT_PSCI_PSCI_VERSION:
		sbi_printf("PSCI version\n");
		ret = SBI_SUCCESS;
		*out_val = PSCI_VERSION_VAL; /* PSCI v1.1 */
		break;
	case SBI_EXT_PSCI_CPU_SUSPEND:
	case SBI_EXT_PSCI64_CPU_SUSPEND:
		sbi_printf("Entering WFI\n");
		__asm__ __volatile__ ("wfi");
		ret = SBI_SUCCESS;
		*out_val = PSCI_ERR_SUCCESS;
		break;
	case SBI_EXT_PSCI_CPU_OFF:
		sbi_printf("CPU off\n");
		ret = sbi_hsm_hart_stop(scratch, TRUE);
		*out_val = (ret) ? PSCI_ERR_DENIED : PSCI_ERR_SUCCESS;
		break;
	case SBI_EXT_PSCI_CPU_ON:
	case SBI_EXT_PSCI64_CPU_ON:
		sbi_printf("CPU on\n");
		smode = csr_read(CSR_MSTATUS);
		smode = (smode & MSTATUS_MPP) >> MSTATUS_MPP_SHIFT;
		ret = sbi_hsm_hart_start(scratch, sbi_domain_thishart_ptr(),
					 args[0], args[1], smode, args[2]);
		*out_val = map_hart_start_to_psci_cpu_on_state(ret);
		break;
	case SBI_EXT_PSCI_AFFINITY_INFO:
	case SBI_EXT_PSCI64_AFFINITY_INFO:
		sbi_printf("Affinity info\n");
		if (args[1] != 0) {
			*out_val = PSCI_ERR_INVALID_PARAMETERS;
			ret = SBI_ERR_INVALID_PARAM;
			break;
		}
                hstate = sbi_hsm_hart_get_state(sbi_domain_thishart_ptr(),
                                                args[0]);
                *out_val = map_hart_state_to_psci_aff_inst_state(hstate);
		ret = SBI_SUCCESS;
		break;
	case SBI_EXT_PSCI_SYSTEM_OFF:
		sbi_printf("Shutdown\n");
		sbi_system_reset(SBI_PLATFORM_RESET_SHUTDOWN);
		/* Must never return */
		while (1)
			__asm__ __volatile__ ("wfi");
		break;
	case SBI_EXT_PSCI_SYSTEM_RESET:
		sbi_printf("Cold system reset\n");
		sbi_system_reset(SBI_PLATFORM_RESET_COLD);
		/* Must never return */
		while (1)
			__asm__ __volatile__ ("wfi");
		break;
	case SBI_EXT_PSCI_PSCI_FEATURES:
		sbi_printf("Features query\n");
		*out_val = PSCI_ERR_NOT_SUPPORTED;
		ret = SBI_ENOTSUPP;
		for (i = 0; i < array_size(psci_funcs_impl); i++) {
			if (args[0] == psci_funcs_impl[i]) {
				*out_val = PSCI_FUNC_SUPPORTED_VAL;
				ret = SBI_SUCCESS;
				break;
			}
		}
		break;
	case SBI_EXT_PSCI_SYSTEM_RESET2:
	case SBI_EXT_PSCI64_SYSTEM_RESET2:
		sbi_printf("Warm reset\n");
		if (args[0] == 0) {
			sbi_system_reset(SBI_PLATFORM_RESET_WARM);
			/* Should never return */
		};
		ret = SBI_ENOTSUPP;
		*out_val = PSCI_ERR_NOT_SUPPORTED;
		break;
	case SBI_EXT_PSCI_MIGRATE:
	case SBI_EXT_PSCI64_MIGRATE:
	case SBI_EXT_PSCI_MIGRATE_INFO_TYPE:
	case SBI_EXT_PSCI_MIGRATE_INFO_UP_CPU:
	case SBI_EXT_PSCI64_MIGRATE_INFO_UP_CPU:
	case SBI_EXT_PSCI_CPU_FREEZE:
	case SBI_EXT_PSCI_CPU_DEFAULT_SUSPEND:
	case SBI_EXT_PSCI_NODE_HW_STATE:
	case SBI_EXT_PSCI_SYSTEM_SUSPEND:
	case SBI_EXT_PSCI64_SYSTEM_SUSPEND:
	case SBI_EXT_PSCI_SET_SUSPEND_MODE:
	case SBI_EXT_PSCI_STAT_RESIDENCY:
	case SBI_EXT_PSCI_STAT_COUNT:
	case SBI_EXT_PSCI_MEM_PROTECT:
	case SBI_EXT_PSCI_MEM_PROTECT_CHECK_RANGE:
	default:
		ret = SBI_ENOTSUPP;
		*out_val = PSCI_ERR_NOT_SUPPORTED;
	};

	return ret;
}

struct sbi_ecall_extension ecall_psci = {
	.extid_start = SBI_EXT_PSCI,
	.extid_end = SBI_EXT_PSCI,
	.handle = sbi_ecall_psci_handler,
};
