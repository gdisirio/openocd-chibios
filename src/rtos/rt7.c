// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "rtos_chibios_stackings_alt.h"
#include "helper/log.h"
#include "helper/types.h"
#include "target/armv7m.h"

/* ChibiOS/RT 7 debugger ABI, from os/rt/include/chregistry.h. Use byte
 * fields so the target's endianness and structure padding remain explicit.
 * The target record can have trailing padding or additional fields.
 */
struct rt7_signature {
	uint8_t identifier[4];
	uint8_t zero;
	uint8_t size;
	uint8_t version[2];
	uint8_t ptrsize;
	uint8_t timesize;
	uint8_t threadsize;
	uint8_t off_prio;
	uint8_t off_ctx;
	uint8_t off_newer;
	uint8_t off_older;
	uint8_t off_name;
	uint8_t off_stklimit;
	uint8_t off_state;
	uint8_t off_flags;
	uint8_t off_refs;
	uint8_t off_preempt;
	uint8_t off_time;
	uint8_t reserved[4];
	uint8_t intctxsize;
	uint8_t intervalsize;
	uint8_t instancesnum;
	uint8_t off_sys_state;
	uint8_t off_sys_instances;
	uint8_t off_sys_reglist;
	uint8_t off_sys_rfcu;
	uint8_t sys_reserved[4];
	uint8_t off_inst_rlist_current;
	uint8_t off_inst_rlist;
	uint8_t off_inst_vtlist;
	uint8_t off_inst_reglist;
	uint8_t off_inst_core_id;
	uint8_t off_inst_rfcu;
};

enum rt7_symbol {
	RT7_CH_SYSTEM,
	RT7_CH_DEBUG,
};

static const struct symbol_table_elem rt7_symbols[] = {
	{ "ch_system", 0, false },
	{ "ch_debug", 0, false },
	{ NULL, 0, false },
};

static const char * const rt7_thread_states[] = {
	"READY", "CURRENT", "WTSTART", "SUSPENDED", "QUEUED", "WTSEM",
	"WTMTX", "WTCOND", "SLEEPING", "WTEXIT", "WTOREVT", "WTANDEVT",
	"SNDMSGQ", "SNDMSG", "WTMSG", "FINAL",
};

#define RT7_NAME_SIZE 64
/* Bound target reads and allocations if a damaged registry never terminates. */
#define RT7_MAX_THREADS 4096

static bool rt7_target_supported(struct target *target)
{
	return !strcmp(target_type_name(target), "cortex_m") ||
		!strcmp(target_type_name(target), "hla_target");
}

static int rt7_read_signature(struct rtos *rtos, struct rt7_signature *signature)
{
	uint8_t header[8];
	int retval;

	if (!rtos->symbols || !rtos->symbols[RT7_CH_SYSTEM].address ||
			!rtos->symbols[RT7_CH_DEBUG].address)
		return ERROR_FAIL;

	retval = target_read_buffer(rtos->target, rtos->symbols[RT7_CH_DEBUG].address,
			sizeof(header), header);
	if (retval != ERROR_OK)
		return retval;

	if (memcmp(header, "main", 4) || header[4] || header[5] < sizeof(*signature) ||
			(target_buffer_get_u16(rtos->target, header + 6) >> 11) != 7) {
		LOG_ERROR("rt7 requires a ChibiOS/RT 7 debugger signature");
		return ERROR_FAIL;
	}

	retval = target_read_buffer(rtos->target, rtos->symbols[RT7_CH_DEBUG].address,
			sizeof(*signature), (uint8_t *)signature);
	if (retval != ERROR_OK)
		return retval;

	if (signature->ptrsize != 4 || !signature->instancesnum ||
			signature->off_newer + 8 > signature->threadsize ||
			signature->off_older != signature->off_newer + 4 ||
			signature->off_ctx + 4 > signature->threadsize ||
			signature->off_name + 4 > signature->threadsize ||
			signature->off_state >= signature->threadsize) {
		LOG_ERROR("Invalid or unsupported rt7 debugger layout");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static bool rt7_detect(struct target *target)
{
	struct rt7_signature signature;

	return rt7_target_supported(target) &&
		rt7_read_signature(target->rtos, &signature) == ERROR_OK;
}

static int rt7_create(struct target *target)
{
	if (!rt7_target_supported(target)) {
		LOG_ERROR("rt7 does not support target type %s", target_type_name(target));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int rt7_get_symbols(struct symbol_table_elem **symbols)
{
	*symbols = malloc(sizeof(rt7_symbols));
	if (!*symbols)
		return ERROR_FAIL;
	memcpy(*symbols, rt7_symbols, sizeof(rt7_symbols));
	return ERROR_OK;
}

static int rt7_add_thread(struct rtos *rtos, const struct rt7_signature *signature,
		uint32_t thread)
{
	struct thread_detail *details;
	struct thread_detail *detail;
	uint32_t name;
	uint8_t state;
	char text[RT7_NAME_SIZE] = "";
	int retval;

	retval = target_read_u32(rtos->target, thread + signature->off_name, &name);
	if (retval != ERROR_OK)
		return retval;

	/* Names are optional and need not occupy a full RT7_NAME_SIZE bytes of
	 * readable memory. Stop reading at the first terminator.
	 */
	if (name) {
		for (size_t i = 0; i < sizeof(text) - 1; i++) {
			retval = target_read_u8(rtos->target, name + i, (uint8_t *)&text[i]);
			if (retval != ERROR_OK)
				return retval;
			if (!text[i])
				break;
		}
	}

	retval = target_read_u8(rtos->target, thread + signature->off_state, &state);
	if (retval != ERROR_OK)
		return retval;

	details = realloc(rtos->thread_details, (rtos->thread_count + 1) * sizeof(*details));
	if (!details)
		return ERROR_FAIL;
	rtos->thread_details = details;
	detail = &details[rtos->thread_count++];
	*detail = (struct thread_detail) { .threadid = thread, .exists = true };
	detail->thread_name_str = strdup(text[0] ? text : "No Name");
	detail->extra_info_str = alloc_printf("State: %s",
			state < ARRAY_SIZE(rt7_thread_states) ? rt7_thread_states[state] : "Unknown");
	if (!detail->thread_name_str || !detail->extra_info_str)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int rt7_current_execution(struct rtos *rtos)
{
	/* An empty thread list can leave old RTOS threads cached in GDB after
	 * reset. Publish a distinct placeholder until the kernel initializes.
	 */
	rtos->thread_details = calloc(1, sizeof(*rtos->thread_details));
	if (!rtos->thread_details)
		return ERROR_FAIL;
	rtos->thread_count = 1;
	rtos->thread_details[0].threadid = 1;
	rtos->thread_details[0].exists = true;
	rtos->thread_details[0].thread_name_str = strdup("Current Execution");
	if (!rtos->thread_details[0].thread_name_str) {
		rtos_free_threadlist(rtos);
		return ERROR_FAIL;
	}
	rtos->current_thread = 1;
	return ERROR_OK;
}

static int rt7_update_threads(struct rtos *rtos)
{
	struct rt7_signature signature;
	uint32_t instance;
	uint32_t current_thread;
	uint32_t head;
	uint32_t node;
	uint32_t previous;
	uint32_t prev;
	int current_index = -1;
	threadid_t selected_thread = rtos->current_threadid;
	bool selected_found = false;
	int retval;

	rtos_free_threadlist(rtos);
	rtos->current_thread = 0;
	retval = rt7_read_signature(rtos, &signature);
	if (retval != ERROR_OK)
		return retval;

	if (signature.off_sys_reglist || !signature.off_inst_reglist || rtos->target->smp) {
		LOG_ERROR("rt7 currently requires a non-SMP ChibiOS configuration");
		return ERROR_FAIL;
	}

	/* Non-SMP kernels register their local instance at index zero, even
	 * when the port can run on more than one physical core.
	 */
	retval = target_read_u32(rtos->target,
			rtos->symbols[RT7_CH_SYSTEM].address + signature.off_sys_instances, &instance);
	if (retval != ERROR_OK)
		return retval;
	if (!instance)
		return rt7_current_execution(rtos);
	retval = target_read_u32(rtos->target,
			instance + signature.off_inst_rlist_current, &current_thread);
	if (retval != ERROR_OK)
		return retval;
	if (!current_thread)
		return rt7_current_execution(rtos);

	head = instance + signature.off_inst_reglist;
	previous = head;
	retval = target_read_u32(rtos->target, head, &node);
	if (retval != ERROR_OK)
		return retval;

	while (node != head) {
		uint32_t thread;

		if (!node || (node & 3) || node < signature.off_newer ||
				rtos->thread_count == RT7_MAX_THREADS)
			goto corrupt;
		retval = target_read_u32(rtos->target, node + 4, &prev);
		if (retval != ERROR_OK)
			goto error;
		if (prev != previous)
			goto corrupt;

		/* Registry links address thread_t.rqueue, not the thread itself. */
		thread = node - signature.off_newer;
		retval = rt7_add_thread(rtos, &signature, thread);
		if (retval != ERROR_OK)
			goto error;
		if (thread == current_thread)
			current_index = rtos->thread_count - 1;
		selected_found |= thread == selected_thread;
		previous = node;
		retval = target_read_u32(rtos->target, node, &node);
		if (retval != ERROR_OK)
			goto error;
		keep_alive();
	}

	retval = target_read_u32(rtos->target, head + 4, &prev);
	if (retval != ERROR_OK)
		goto error;
	if (prev != previous || current_index < 0)
		goto corrupt;

	/* On the first connection GDB can acquire the live registers before
	 * qSymbol has supplied the kernel symbols. It associates that context
	 * with the first thread we report, so put the running thread first.
	 */
	if (current_index > 0) {
		struct thread_detail current = rtos->thread_details[current_index];

		memmove(rtos->thread_details + 1, rtos->thread_details,
				current_index * sizeof(*rtos->thread_details));
		rtos->thread_details[0] = current;
	}
	rtos->current_thread = current_thread;
	rtos->current_threadid = selected_found ? selected_thread : -1;
	return ERROR_OK;

corrupt:
	LOG_ERROR("rt7 registry integrity check failed");
	retval = ERROR_FAIL;
error:
	rtos_free_threadlist(rtos);
	return retval;
}

static int rt7_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	struct rt7_signature signature;
	struct stack_register_offset offsets[ARMV7M_NUM_CORE_REGS];
	struct rtos_register_stacking stacking = {
		.stack_growth_direction = -1,
		.num_output_registers = ARMV7M_NUM_CORE_REGS,
		.register_offsets = offsets,
	};
	struct armv7m_common *armv7m = target_to_armv7m(rtos->target);
	uint32_t sp;
	unsigned int base;
	bool found = false;
	int retval;

	*reg_list = NULL;
	*num_regs = 0;
	for (int i = 0; i < rtos->thread_count; i++)
		found |= rtos->thread_details[i].threadid == thread_id;
	if (!found || !is_armv7m(armv7m))
		return ERROR_FAIL;

	retval = rt7_read_signature(rtos, &signature);
	if (retval != ERROR_OK)
		return retval;

	/* The standard Cortex-M ports store SP first in port_context. The
	 * ARMv7-M syscall configuration adds two words after SP. ARMv7-M-ALT
	 * embeds its internal context here instead. Restrict the ALT decoder
	 * to ARMv7-M: the ARMv8-M TZ ports have a different embedded layout.
	 */
	if (signature.off_newer != signature.off_ctx + 4 &&
			signature.off_newer != signature.off_ctx + 12) {
		unsigned int minimum_size = 4 + signature.intctxsize;
		int context_size = signature.off_newer - signature.off_ctx;

		/* RT 7 adds three syscall words when CONTROL is saved, and up
		 * to four pairs of MPU registers after the embedded context.
		 */
		if (signature.intctxsize == 44 || signature.intctxsize == 108)
			minimum_size += 12;
		if (armv7m->arm.arch == ARM_ARCH_V7M && context_size >= (int)minimum_size &&
				context_size <= (int)minimum_size + 32 && (context_size - minimum_size) % 8 == 0)
			return rtos_chibios_alt_get_thread_reg_list(rtos->target,
					thread_id + signature.off_ctx, signature.intctxsize, reg_list, num_regs);
		LOG_ERROR("Unsupported rt7 Cortex-M context layout");
		return ERROR_FAIL;
	}
	if (signature.intctxsize < 36 || (signature.intctxsize & 3) ||
			(armv7m->arm.arch != ARM_ARCH_V6M && armv7m->arm.arch != ARM_ARCH_V7M &&
			armv7m->arm.arch != ARM_ARCH_V8M)) {
		LOG_ERROR("Unsupported rt7 Cortex-M context layout");
		return ERROR_FAIL;
	}

	retval = target_read_u32(rtos->target, thread_id + signature.off_ctx, &sp);
	if (retval != ERROR_OK)
		return retval;

	/* R4-R11 and the return address form the final nine words. Optional
	 * MPU regions, FPU registers and the stack limit precede them. Taking
	 * their offset from intctxsize avoids guessing from the CPU's FPU state.
	 */
	stacking.stack_registers_size = signature.intctxsize;
	base = signature.intctxsize - 36;
	for (unsigned int i = 0; i < ARRAY_SIZE(offsets); i++)
		offsets[i] = (struct stack_register_offset) { i, -1, 32 };
	for (unsigned int i = 0; i < 8; i++) {
		unsigned int slot = armv7m->arm.arch == ARM_ARCH_V6M ? (i + 4) % 8 : i;
		offsets[ARMV7M_R4 + i].offset = base + slot * 4;
	}
	offsets[ARMV7M_R13].offset = -2;
	offsets[ARMV7M_PC].offset = base + 32;

	return rtos_generic_stack_read(rtos->target, &stacking, sp, reg_list, num_regs);
}

const struct rtos_type rt7_rtos = {
	.name = "rt7",
	.detect_rtos = rt7_detect,
	.create = rt7_create,
	.update_threads = rt7_update_threads,
	.get_thread_reg_list = rt7_get_thread_reg_list,
	.get_symbol_list_to_lookup = rt7_get_symbols,
};
