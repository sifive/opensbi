#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2019 Sifive Corporation.
#
# Authors:
#   Zong Li <zong.li@sifive.com>
#

ifndef FREEDOM_METAL_PATH
$(error FREEDOM_METAL_PATH is not set)
endif

# Compiler flags
platform-cflags-y = -I$(FREEDOM_METAL_PATH)/include \
                    -I$(FREEDOM_METAL_PATH)/include/metal
platform-ldflags-y = -L $(FREEDOM_METAL_PATH)/lib/release -lmetal

# Firmware load address configuration. This is mandatory.
FW_TEXT_START=0x80000000

# Dynamic firmware configuration.
FW_DYNAMIC=y

# Jump firmware configuration.
FW_JUMP=y
ifeq ($(PLATFORM_RISCV_XLEN), 32)
FW_JUMP_ADDR=0x80400000
else
FW_JUMP_ADDR=0x80200000
endif
FW_JUMP_FDT_ADDR=0x88000000

# Firmware with payload configuration.
FW_PAYLOAD=y
ifeq ($(PLATFORM_RISCV_XLEN), 32)
FW_PAYLOAD_OFFSET=0x400000
else
FW_PAYLOAD_OFFSET=0x200000
endif
FW_PAYLOAD_FDT_ADDR=0x88000000
