/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 SiFive Corporation.
 *
 * Authors:
 *   Zong Li <zong.li@sifive.com>
 */

#ifndef _VCU118_PLATFORM_H_
#define _VCU118_PLATFORM_H_

#define HAWKSBILL_MAX_NODE_NUM 		4
#define HAWKSBILL_MAX_HART_PER_NODE	8
#define HAWKSBILL_NODE_OFFSET_FACTOR	0x100000000UL

#define hawksbill_get_node_idx_by_target(target_hart) \
	(target_hart / HAWKSBILL_MAX_HART_PER_NODE)

#define hawksbill_get_node_target_idx(target_hart) \
	(target_hart % HAWKSBILL_MAX_HART_PER_NODE)

int hawksbill_node_num(void);

#endif
