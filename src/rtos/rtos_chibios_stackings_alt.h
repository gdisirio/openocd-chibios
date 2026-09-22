/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_RTOS_RTOS_CHIBIOS_STACKINGS_ALT_H
#define OPENOCD_RTOS_RTOS_CHIBIOS_STACKINGS_ALT_H

#include "rtos.h"

/* Decode ARMv7-M-ALT independently of the kernel's registry ABI. The caller
 * supplies the address of port_context and the size of port_intctx, after
 * validating the kernel-specific layout of port_context.
 */
int rtos_chibios_alt_get_thread_reg_list(struct target *target,
		target_addr_t context, unsigned int intctx_size,
		struct rtos_reg **reg_list, int *num_regs);

#endif /* OPENOCD_RTOS_RTOS_CHIBIOS_STACKINGS_ALT_H */
