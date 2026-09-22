// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos_chibios_stackings_alt.h"
#include "helper/bits.h"
#include "helper/log.h"
#include "target/armv7m.h"

int rtos_chibios_alt_get_thread_reg_list(struct target *target,
		target_addr_t context, unsigned int intctx_size,
		struct rtos_reg **reg_list, int *num_regs)
{
	uint8_t saved[48];
	uint8_t frame[32];
	struct rtos_reg *regs;
	bool fpu;
	bool control;
	unsigned int integer_size;
	unsigned int frame_size;
	uint32_t sp;
	uint32_t exc_return;
	uint32_t xpsr;
	int retval;

	*reg_list = NULL;
	*num_regs = 0;
	/* port_intctx holds BASEPRI, R4-R11, optional CONTROL, EXC_RETURN,
	 * then optional S16-S31. Any MPU and syscall fields following these
	 * registers are kernel-specific and do not affect register decoding.
	 */
	if (intctx_size != 40 && intctx_size != 44 &&
			intctx_size != 104 && intctx_size != 108)
		goto unsupported;
	fpu = intctx_size >= 104;
	control = intctx_size == 44 || intctx_size == 108;

	/* Only read the integer part: saved PSP followed by port_intctx's
	 * integer registers. No need to read MPU or floating-point state.
	 */
	integer_size = control ? 48 : 44;
	if (!context || (context & 3) || context > UINT32_MAX - integer_size)
		return ERROR_FAIL;
	retval = target_read_buffer(target, context, integer_size, saved);
	if (retval != ERROR_OK)
		return retval;
	sp = target_buffer_get_u32(target, saved);
	exc_return = target_buffer_get_u32(target, saved + integer_size - 4);
	/* ALT resumes threads on PSP. EXC_RETURN, rather than FPU support
	 * alone, determines the hardware frame size. A newly created thread
	 * uses a basic frame even in an FPU-enabled build.
	 */
	if (exc_return != 0xfffffffd && (exc_return != 0xffffffed || !fpu)) {
		LOG_ERROR("Invalid ChibiOS ARMv7-M-ALT exception return");
		return ERROR_FAIL;
	}
	frame_size = exc_return & BIT(4) ? 32 : 104;
	if (!sp || (sp & 3) || sp > UINT32_MAX - frame_size)
		return ERROR_FAIL;
	retval = target_read_buffer(target, sp, sizeof(frame), frame);
	if (retval != ERROR_OK)
		return retval;
	xpsr = target_buffer_get_u32(target, frame + 28);
	/* Bit 9 records the extra word used to align the exception stack. */
	if (xpsr & BIT(9))
		frame_size += 4;
	if (sp > UINT32_MAX - frame_size)
		return ERROR_FAIL;

	regs = calloc(ARMV7M_NUM_CORE_REGS, sizeof(*regs));
	if (!regs)
		return ERROR_FAIL;
	for (unsigned int i = 0; i < ARMV7M_NUM_CORE_REGS; i++) {
		regs[i].number = i;
		regs[i].size = 32;
	}
	for (unsigned int i = 0; i < 4; i++)
		memcpy(regs[ARMV7M_R0 + i].value, frame + 4 * i, 4);
	for (unsigned int i = 0; i < 8; i++)
		memcpy(regs[ARMV7M_R4 + i].value, saved + 8 + 4 * i, 4);
	memcpy(regs[ARMV7M_R12].value, frame + 16, 4);
	target_buffer_set_u32(target, regs[ARMV7M_R13].value, sp + frame_size);
	memcpy(regs[ARMV7M_R14].value, frame + 20, 4);
	memcpy(regs[ARMV7M_PC].value, frame + 24, 4);
	memcpy(regs[ARMV7M_XPSR].value, frame + 28, 4);
	*reg_list = regs;
	*num_regs = ARMV7M_NUM_CORE_REGS;
	return ERROR_OK;

unsupported:
	LOG_ERROR("Unsupported ChibiOS ARMv7-M-ALT context layout");
	return ERROR_FAIL;
}
