// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "helper/bits.h"
#include "helper/log.h"
#include "helper/types.h"
#include "target/armv7m.h"
#include "target/register.h"
#include "target/smp.h"
#include "server/gdb_server.h"

/* RT8's binary descriptor is independent of host C structure layout.
 * The kernel version selects this format; neither DWARF nor kernel headers
 * are needed. See ChibiOS doc/build/rt/src/registry.dox, merged in
 * chibios-upstream/chibios#404 (1505522545).
 */
#define RT8_NONE UINT16_MAX
#define RT8_COMMON_SIZE 74
#define RT8_PORT_SIZE 52
#define RT8_MAX_INSTANCES 64
#define RT8_MAX_THREADS 4096
#define RT8_NAME_SIZE 64
#define RT8_SMP BIT(0)
#define RT8_FPU BIT(0)
#define RT8_MPU BIT(1)

enum rt8_field {
	RT8_SYS_SIZE, RT8_SYS_COUNT, RT8_SYS_STATE, RT8_SYS_INSTANCES,
	RT8_SYS_REGISTRY, RT8_SYS_RFCU,
	RT8_INST_SIZE, RT8_INST_CORE, RT8_INST_CURRENT, RT8_INST_RLIST,
	RT8_INST_VTLIST, RT8_INST_REGISTRY, RT8_INST_RFCU,
	RT8_THREAD_SIZE, RT8_CTX_SIZE, RT8_INNER_SIZE, RT8_THREAD_NODE,
	RT8_THREAD_OWNER, RT8_THREAD_NAME, RT8_THREAD_PRIO, RT8_THREAD_STATE,
	RT8_THREAD_FLAGS, RT8_THREAD_REFS, RT8_THREAD_TICKS, RT8_THREAD_TIME,
	RT8_THREAD_BASE, RT8_THREAD_END, RT8_THREAD_CTX, RT8_FIELD_COUNT,
};

enum rt8_port_field {
	RT8_PORT_FLAGS, RT8_CTX_SP, RT8_CTX_REGS, RT8_INNER_R4, RT8_INNER_R8,
	RT8_INNER_LR, RT8_INNER_EXC_RETURN, RT8_INNER_CONTROL, RT8_INNER_BASEPRI,
	RT8_INNER_PSPLIM, RT8_INNER_S16, RT8_INNER_FPSCR, RT8_FRAME_SIZE,
	RT8_FRAME_R0, RT8_FRAME_R12, RT8_FRAME_LR, RT8_FRAME_PC, RT8_FRAME_XPSR,
	RT8_FRAME_S0, RT8_FRAME_FPSCR, RT8_FPU_MODE, RT8_CTX_REGIONS,
	RT8_REGIONS_COUNT, RT8_REGION_SIZE, RT8_REGION_RBAR, RT8_REGION_ATTR,
	RT8_PORT_FIELD_COUNT,
};

enum rt8_width {
	RT8_PTR, RT8_TIME, RT8_INTERVAL, RT8_PRIO, RT8_STATE, RT8_FLAGS,
	RT8_REFS, RT8_TICKS, RT8_CORE, RT8_SYSTEM_STATE, RT8_WIDTH_COUNT,
};

struct rt8_layout {
	uint8_t flags;
	uint8_t port_id;
	uint8_t width[RT8_WIDTH_COUNT];
	uint16_t field[RT8_FIELD_COUNT];
	uint16_t port[RT8_PORT_FIELD_COUNT];
};

struct rt8_instance {
	uint32_t address;
	uint32_t current;
	uint32_t core;
	struct target *target;
};

struct rt8_snapshot {
	struct rt8_layout layout;
	struct rt8_instance instances[RT8_MAX_INSTANCES];
	unsigned int count;
};

struct rt8_context {
	uint32_t inner;
	uint32_t frame;
	uint32_t sp;
	bool extended;
};

enum rt8_symbol { RT8_CH_SYSTEM, RT8_CH_DEBUG };

static const struct symbol_table_elem rt8_symbols[] = {
	{ "ch_system", 0, false },
	{ "ch_debug", 0, false },
	{ NULL, 0, false },
};

static const char * const rt8_states[] = {
	"READY", "CURRENT", "WTSTART", "SUSPENDED", "QUEUED", "WTSEM",
	"WTMTX", "WTCOND", "SLEEPING", "WTEXIT", "WTOREVT", "WTANDEVT",
	"SNDMSGQ", "SNDMSG", "WTMSG", "FINAL",
};

static bool rt8_target_supported(struct target *target)
{
	return !strcmp(target_type_name(target), "cortex_m") ||
		!strcmp(target_type_name(target), "hla_target");
}

static bool rt8_range(target_addr_t address, unsigned int size)
{
	return address && address <= UINT32_MAX && size &&
		size - 1 <= UINT32_MAX - address;
}

static bool rt8_member(unsigned int size, unsigned int offset, unsigned int width)
{
	return offset != RT8_NONE && width && offset <= size && width <= size - offset;
}

static bool rt8_optional(unsigned int size, unsigned int offset, unsigned int width)
{
	return offset == RT8_NONE || rt8_member(size, offset, width);
}

static int rt8_read_uint(struct target *target, target_addr_t address,
		unsigned int width, uint64_t *value)
{
	uint8_t data[8];
	int retval;

	if (width > sizeof(data) || !rt8_range(address, width))
		return ERROR_FAIL;
	retval = target_read_buffer(target, address, width, data);
	if (retval != ERROR_OK)
		return retval;
	*value = 0;
	for (unsigned int i = 0; i < width; i++) {
		unsigned int shift = target->endianness == TARGET_BIG_ENDIAN ? width - 1 - i : i;
		*value |= (uint64_t)data[i] << (shift * 8);
	}
	return ERROR_OK;
}

static int rt8_read_pointer(struct target *target, target_addr_t address, uint32_t *value)
{
	uint64_t data;
	int retval = rt8_read_uint(target, address, 4, &data);

	if (retval == ERROR_OK)
		*value = data;
	return retval;
}

static bool rt8_validate_layout(const struct rt8_layout *layout)
{
	const uint16_t *f = layout->field;
	const uint16_t *p = layout->port;
	const uint8_t *w = layout->width;
	bool shared = layout->flags & RT8_SMP;
	bool split = layout->port_id == 4 || layout->port_id == 6;
	bool fpu = p[RT8_PORT_FLAGS] & RT8_FPU;
	bool mpu = p[RT8_PORT_FLAGS] & RT8_MPU;

	if ((layout->flags & ~RT8_SMP) || w[RT8_PTR] != 4 ||
			!f[RT8_SYS_COUNT] || f[RT8_SYS_COUNT] > RT8_MAX_INSTANCES ||
			!rt8_member(f[RT8_SYS_SIZE], f[RT8_SYS_STATE], w[RT8_SYSTEM_STATE]) ||
			!rt8_member(f[RT8_SYS_SIZE], f[RT8_SYS_INSTANCES], 4 * f[RT8_SYS_COUNT]) ||
			!rt8_optional(f[RT8_SYS_SIZE], f[RT8_SYS_RFCU], 1) ||
			!rt8_member(f[RT8_INST_SIZE], f[RT8_INST_CORE], w[RT8_CORE]) ||
			!rt8_member(f[RT8_INST_SIZE], f[RT8_INST_CURRENT], 4) ||
			!rt8_member(f[RT8_INST_SIZE], f[RT8_INST_RLIST], 1) ||
			!rt8_member(f[RT8_INST_SIZE], f[RT8_INST_VTLIST], 1) ||
			!rt8_optional(f[RT8_INST_SIZE], f[RT8_INST_RFCU], 1))
		return false;
	if (shared ? (!rt8_member(f[RT8_SYS_SIZE], f[RT8_SYS_REGISTRY], 8) ||
			f[RT8_INST_REGISTRY] != RT8_NONE) :
			(!rt8_member(f[RT8_INST_SIZE], f[RT8_INST_REGISTRY], 8) ||
			f[RT8_SYS_REGISTRY] != RT8_NONE))
		return false;
	if (!rt8_member(f[RT8_THREAD_SIZE], f[RT8_THREAD_NODE], 8) ||
			!rt8_member(f[RT8_THREAD_SIZE], f[RT8_THREAD_CTX], f[RT8_CTX_SIZE]) ||
			!f[RT8_INNER_SIZE])
		return false;
	const enum rt8_field pointers[] = {
		RT8_THREAD_OWNER, RT8_THREAD_NAME, RT8_THREAD_BASE, RT8_THREAD_END,
	};
	for (unsigned int i = 0; i < ARRAY_SIZE(pointers); i++)
		if (!rt8_member(f[RT8_THREAD_SIZE], f[pointers[i]], 4))
			return false;
	for (unsigned int i = 0; i < 4; i++)
		if (!rt8_member(f[RT8_THREAD_SIZE], f[RT8_THREAD_PRIO + i], w[RT8_PRIO + i]))
			return false;
	if (!rt8_optional(f[RT8_THREAD_SIZE], f[RT8_THREAD_TICKS], w[RT8_TICKS]) ||
			!rt8_optional(f[RT8_THREAD_SIZE], f[RT8_THREAD_TIME], w[RT8_TIME]) ||
			(p[RT8_PORT_FLAGS] & ~(RT8_FPU | RT8_MPU)) ||
			!rt8_member(f[RT8_CTX_SIZE], p[RT8_CTX_SP], 4) ||
			!rt8_member(f[RT8_INNER_SIZE], p[RT8_INNER_R4], 16) ||
			!rt8_member(f[RT8_INNER_SIZE], p[RT8_INNER_R8], 16))
		return false;
	for (unsigned int i = RT8_INNER_LR; i <= RT8_INNER_FPSCR; i++)
		if (!rt8_optional(f[RT8_INNER_SIZE], p[i], i == RT8_INNER_S16 ? 64 : 4))
			return false;
	if (split ? (!rt8_member(f[RT8_CTX_SIZE], p[RT8_CTX_REGS], f[RT8_INNER_SIZE]) ||
			p[RT8_INNER_EXC_RETURN] == RT8_NONE || p[RT8_INNER_LR] != RT8_NONE) :
			(p[RT8_CTX_REGS] != RT8_NONE || p[RT8_INNER_LR] == RT8_NONE ||
			p[RT8_INNER_EXC_RETURN] != RT8_NONE))
		return false;
	/* The basic hardware frame contains the integer fields even when the
	 * maximum frame includes floating-point state. */
	for (unsigned int i = RT8_FRAME_R0; i <= RT8_FRAME_XPSR; i++)
		if (!rt8_member(32, p[i], i == RT8_FRAME_R0 ? 16 : 4))
			return false;
	if (fpu) {
		if (layout->port_id <= 2 || p[RT8_FRAME_SIZE] < 104 ||
				p[RT8_INNER_S16] == RT8_NONE ||
				!rt8_member(p[RT8_FRAME_SIZE], p[RT8_FRAME_S0], 64) ||
				!rt8_member(p[RT8_FRAME_SIZE], p[RT8_FRAME_FPSCR], 4) ||
				p[RT8_FPU_MODE] < 1 || p[RT8_FPU_MODE] > (split ? 2 : 1) ||
				(split ? p[RT8_INNER_FPSCR] != RT8_NONE : p[RT8_INNER_FPSCR] == RT8_NONE))
			return false;
	} else if (p[RT8_FRAME_SIZE] != 32 || p[RT8_FPU_MODE] ||
			p[RT8_INNER_S16] != RT8_NONE || p[RT8_INNER_FPSCR] != RT8_NONE ||
			p[RT8_FRAME_S0] != RT8_NONE || p[RT8_FRAME_FPSCR] != RT8_NONE) {
		return false;
	}
	if (mpu)
		return split && p[RT8_REGIONS_COUNT] &&
			rt8_member(f[RT8_CTX_SIZE], p[RT8_CTX_REGIONS], 4) &&
			rt8_member(p[RT8_REGION_SIZE], p[RT8_REGION_RBAR], 4) &&
			rt8_member(p[RT8_REGION_SIZE], p[RT8_REGION_ATTR], 4);
	return p[RT8_CTX_REGIONS] == RT8_NONE && !p[RT8_REGIONS_COUNT] &&
		!p[RT8_REGION_SIZE] && p[RT8_REGION_RBAR] == RT8_NONE && p[RT8_REGION_ATTR] == RT8_NONE;
}

static int rt8_read_layout(struct rtos *rtos, struct rt8_layout *layout)
{
	uint8_t data[UINT8_MAX];
	struct target *target = rtos->target;
	unsigned int offset, size;
	int retval;

	if (!rtos->symbols || !rtos->symbols[RT8_CH_SYSTEM].address ||
			!rt8_range(rtos->symbols[RT8_CH_DEBUG].address, 8))
		return ERROR_FAIL;
	retval = target_read_buffer(target, rtos->symbols[RT8_CH_DEBUG].address, 8, data);
	if (retval != ERROR_OK)
		return retval;
	/* Version mismatches are normal while auto-detection tries RT8 first. */
	if (memcmp(data, "main\0", 5) || (target_buffer_get_u16(target, data + 6) >> 11) != 8)
		return ERROR_FAIL;
	size = data[5];
	if (size < RT8_COMMON_SIZE || !rt8_range(rtos->symbols[RT8_CH_DEBUG].address, size))
		goto invalid;
	retval = target_read_buffer(target, rtos->symbols[RT8_CH_DEBUG].address, size, data);
	if (retval != ERROR_OK)
		return retval;
	if (data[12] != 2 || data[13] < 1 || data[13] > 6 || (data[11] & 0xf0))
		goto invalid;
	layout->flags = data[8];
	layout->port_id = data[13];
	for (unsigned int i = 0; i < RT8_WIDTH_COUNT; i++)
		layout->width[i] = 1U << ((data[9 + i / 4] >> (2 * (i % 4))) & 3);
	for (unsigned int i = 0; i < RT8_FIELD_COUNT; i++)
		layout->field[i] = target_buffer_get_u16(target, data + 18 + 2 * i);
	offset = target_buffer_get_u16(target, data + 14);
	unsigned int port_size = target_buffer_get_u16(target, data + 16);
	if (offset < RT8_COMMON_SIZE || port_size < RT8_PORT_SIZE ||
			!rt8_member(size, offset, port_size))
		goto invalid;
	for (unsigned int i = 0; i < RT8_PORT_FIELD_COUNT; i++)
		layout->port[i] = target_buffer_get_u16(target, data + offset + 2 * i);
	if (!rt8_validate_layout(layout) ||
			!rt8_range(rtos->symbols[RT8_CH_SYSTEM].address, layout->field[RT8_SYS_SIZE]))
		goto invalid;
	return ERROR_OK;

invalid:
	LOG_ERROR("Invalid or unsupported chibios-rt8 debugger descriptor");
	return ERROR_FAIL;
}

static int rt8_snapshot(struct rtos *rtos, struct rt8_snapshot *snapshot)
{
	struct target *target = rtos->target;
	const uint16_t *f = snapshot->layout.field;
	uint64_t state;
	int retval;

	snapshot->count = 0;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	/* A shared registry is meaningful only while every core is stopped.
	 * Even a core without a published instance may be initializing it. */
	if (target->smp) {
		struct target_list *head;
		if (!target->smp_targets || list_empty(target->smp_targets))
			return ERROR_FAIL;
		foreach_smp_target(head, target->smp_targets) {
			struct target *cpu = head->target;
			if (cpu->rtos != rtos || strcmp(target_type_name(cpu), "cortex_m") ||
					cpu->endianness != target->endianness || !target_was_examined(cpu))
				return ERROR_FAIL;
			if (cpu->state != TARGET_HALTED)
				return ERROR_TARGET_NOT_HALTED;
			if (!is_armv7m(target_to_armv7m(cpu)) ||
					target_to_armv7m(cpu)->arm.arch != target_to_armv7m(target)->arm.arch ||
					target_to_armv7m(cpu)->fp_feature != target_to_armv7m(target)->fp_feature)
				return ERROR_FAIL;
		}
	}
	retval = rt8_read_layout(rtos, &snapshot->layout);
	if (retval != ERROR_OK)
		return retval;
	if (!target->smp && (snapshot->layout.flags & RT8_SMP)) {
		LOG_ERROR("chibios-rt8 shared registry requires a configured SMP target group");
		return ERROR_FAIL;
	}
	retval = rt8_read_uint(target, rtos->symbols[RT8_CH_SYSTEM].address + f[RT8_SYS_STATE],
			snapshot->layout.width[RT8_SYSTEM_STATE], &state);
	if (retval != ERROR_OK)
		return retval;
	if (state < 2)
		return ERROR_OK;
	if (state > 3)
		return ERROR_FAIL;
	for (unsigned int i = 0; i < f[RT8_SYS_COUNT]; i++) {
		struct rt8_instance *instance = &snapshot->instances[snapshot->count];
		uint64_t core;
		retval = rt8_read_pointer(target,
				rtos->symbols[RT8_CH_SYSTEM].address + f[RT8_SYS_INSTANCES] + i * 4,
				&instance->address);
		if (retval != ERROR_OK)
			return retval;
		if (!instance->address)
			continue;
		if ((instance->address & 3) || !rt8_range(instance->address, f[RT8_INST_SIZE]))
			return ERROR_FAIL;
		retval = rt8_read_uint(target, instance->address + f[RT8_INST_CORE],
				snapshot->layout.width[RT8_CORE], &core);
		if (retval != ERROR_OK)
			return retval;
		if (core > INT32_MAX)
			return ERROR_FAIL;
		instance->core = core;
		instance->target = target;
		if (target->smp) {
			struct target_list *head;
			instance->target = NULL;
			foreach_smp_target(head, target->smp_targets) {
				if (head->target->coreid != (int32_t)core)
					continue;
				if (instance->target)
					return ERROR_FAIL;
				instance->target = head->target;
			}
			if (!instance->target) {
				LOG_ERROR("chibios-rt8 has no target for core %" PRIu64, core);
				return ERROR_FAIL;
			}
		} else if (snapshot->count) {
			LOG_ERROR("chibios-rt8 currently supports only one initialized instance");
			return ERROR_FAIL;
		}
		retval = rt8_read_pointer(target, instance->address + f[RT8_INST_CURRENT], &instance->current);
		if (retval != ERROR_OK)
			return retval;
		/* An instance is published before its scheduler starts. */
		if (!instance->current)
			continue;
		if ((instance->current & 3) || !rt8_range(instance->current, f[RT8_THREAD_SIZE]))
			return ERROR_FAIL;
		for (unsigned int j = 0; j < snapshot->count; j++) {
			if (snapshot->instances[j].core == instance->core ||
					snapshot->instances[j].address == instance->address ||
					snapshot->instances[j].current == instance->current)
				return ERROR_FAIL;
		}
		snapshot->count++;
	}
	return ERROR_OK;
}

static int rt8_find_thread(struct rtos *rtos, threadid_t thread)
{
	for (int i = 0; i < rtos->thread_count; i++)
		if (rtos->thread_details[i].threadid == thread)
			return i;
	return -1;
}

static struct target *rt8_live_target(const struct rt8_snapshot *snapshot, threadid_t thread)
{
	for (unsigned int i = 0; i < snapshot->count; i++)
		if (snapshot->instances[i].current == thread)
			return snapshot->instances[i].target;
	return NULL;
}

static struct target *rt8_owner_target(struct rtos *rtos,
		const struct rt8_snapshot *snapshot, threadid_t thread)
{
	uint32_t owner;
	if (rt8_read_pointer(rtos->target, thread + snapshot->layout.field[RT8_THREAD_OWNER],
			&owner) != ERROR_OK)
		return NULL;
	for (unsigned int i = 0; i < snapshot->count; i++)
		if (snapshot->instances[i].address == owner)
			return snapshot->instances[i].target;
	return NULL;
}

static unsigned int rt8_stop_priority(struct target *target)
{
	switch (target->debug_reason) {
	case DBG_REASON_SINGLESTEP: return 4;
	case DBG_REASON_BREAKPOINT: return 3;
	case DBG_REASON_WATCHPOINT:
	case DBG_REASON_WPTANDBKPT: return 2;
	case DBG_REASON_DBGRQ: return 1;
	default: return 0;
	}
}

static int rt8_add_thread(struct rtos *rtos, const struct rt8_snapshot *snapshot,
		uint32_t thread, uint32_t registry_instance)
{
	const uint16_t *f = snapshot->layout.field;
	struct target *target = rtos->target;
	uint32_t name, owner;
	uint64_t state, priority;
	char text[RT8_NAME_SIZE] = "";
	unsigned int owner_index;
	int retval;

	if ((thread & 3) || !rt8_range(thread, f[RT8_THREAD_SIZE]) || rt8_find_thread(rtos, thread) >= 0)
		return ERROR_FAIL;
	retval = rt8_read_pointer(target, thread + f[RT8_THREAD_OWNER], &owner);
	if (retval != ERROR_OK)
		return retval;
	for (owner_index = 0; owner_index < snapshot->count; owner_index++)
		if (snapshot->instances[owner_index].address == owner)
			break;
	if (owner_index == snapshot->count || (registry_instance && owner != registry_instance))
		return ERROR_FAIL;
	retval = rt8_read_pointer(target, thread + f[RT8_THREAD_NAME], &name);
	if (retval != ERROR_OK)
		return retval;
	if (name) {
		for (unsigned int i = 0; i < sizeof(text) - 1; i++) {
			uint64_t c;
			retval = rt8_read_uint(target, (target_addr_t)name + i, 1, &c);
			if (retval != ERROR_OK)
				return retval;
			text[i] = c;
			if (!c)
				break;
		}
	}
	retval = rt8_read_uint(target, thread + f[RT8_THREAD_STATE], snapshot->layout.width[RT8_STATE], &state);
	if (retval != ERROR_OK)
		return retval;
	retval = rt8_read_uint(target, thread + f[RT8_THREAD_PRIO], snapshot->layout.width[RT8_PRIO], &priority);
	if (retval != ERROR_OK)
		return retval;
	struct thread_detail *details = realloc(rtos->thread_details,
			(rtos->thread_count + 1) * sizeof(*details));
	if (!details)
		return ERROR_FAIL;
	rtos->thread_details = details;
	struct thread_detail *detail = &details[rtos->thread_count++];
	*detail = (struct thread_detail) { .threadid = thread, .exists = true };
	detail->thread_name_str = strdup(text[0] ? text : "No Name");
	detail->extra_info_str = alloc_printf("State: %s, Priority: %" PRIu64 ", Core: %" PRIu32,
			state < ARRAY_SIZE(rt8_states) ? rt8_states[state] : "Unknown", priority,
			snapshot->instances[owner_index].core);
	return detail->thread_name_str && detail->extra_info_str ? ERROR_OK : ERROR_FAIL;
}

static int rt8_walk_registry(struct rtos *rtos, const struct rt8_snapshot *snapshot,
		uint32_t head, uint32_t registry_instance)
{
	uint32_t node, previous = head, prev;
	int retval = rt8_read_pointer(rtos->target, head, &node);

	if (retval != ERROR_OK)
		return retval;
	while (node != head) {
		if (!node || (node & 3) || node < snapshot->layout.field[RT8_THREAD_NODE] ||
				rtos->thread_count == RT8_MAX_THREADS || !rt8_range(node, 8))
			return ERROR_FAIL;
		retval = rt8_read_pointer(rtos->target, (target_addr_t)node + 4, &prev);
		if (retval != ERROR_OK)
			return retval;
		if (prev != previous)
			return ERROR_FAIL;
		retval = rt8_add_thread(rtos, snapshot, node - snapshot->layout.field[RT8_THREAD_NODE],
				registry_instance);
		if (retval != ERROR_OK)
			return retval;
		previous = node;
		retval = rt8_read_pointer(rtos->target, node, &node);
		if (retval != ERROR_OK)
			return retval;
		keep_alive();
	}
	retval = rt8_read_pointer(rtos->target, (target_addr_t)head + 4, &prev);
	if (retval != ERROR_OK)
		return retval;
	return prev == previous ? ERROR_OK : ERROR_FAIL;
}

static int rt8_current_execution(struct rtos *rtos)
{
	rtos->thread_details = calloc(1, sizeof(*rtos->thread_details));
	if (!rtos->thread_details)
		return ERROR_FAIL;
	rtos->thread_count = 1;
	rtos->thread_details[0] = (struct thread_detail) { .threadid = 1, .exists = true };
	rtos->thread_details[0].thread_name_str = strdup("Current Execution");
	if (!rtos->thread_details[0].thread_name_str) {
		rtos_free_threadlist(rtos);
		return ERROR_FAIL;
	}
	rtos->current_thread = 1;
	return ERROR_OK;
}

static int rt8_update_threads(struct rtos *rtos)
{
	struct rt8_snapshot snapshot;
	threadid_t selected = rtos->current_threadid;
	int retval;
	unsigned int current = 0;

	rtos_free_threadlist(rtos);
	retval = rt8_snapshot(rtos, &snapshot);
	if (retval != ERROR_OK)
		return retval;
	if (!snapshot.count)
		return rt8_current_execution(rtos);
	if (snapshot.layout.flags & RT8_SMP) {
		retval = rt8_walk_registry(rtos, &snapshot,
				rtos->symbols[RT8_CH_SYSTEM].address + snapshot.layout.field[RT8_SYS_REGISTRY], 0);
	} else {
		for (unsigned int i = 0; i < snapshot.count; i++) {
			retval = rt8_walk_registry(rtos, &snapshot,
					snapshot.instances[i].address + snapshot.layout.field[RT8_INST_REGISTRY],
					snapshot.instances[i].address);
			if (retval != ERROR_OK)
				break;
		}
	}
	if (retval != ERROR_OK)
		goto error;
	for (unsigned int i = 0; i < snapshot.count; i++) {
		if (rt8_find_thread(rtos, snapshot.instances[i].current) < 0) {
			retval = ERROR_FAIL;
			goto error;
		}
		unsigned int priority = rt8_stop_priority(snapshot.instances[i].target);
		unsigned int best = rt8_stop_priority(snapshot.instances[current].target);
		if (priority > best || (priority == best &&
				(snapshot.instances[i].current == selected ||
				(snapshot.instances[current].current != selected &&
				snapshot.instances[i].target == rtos->target))))
			current = i;
	}
	rtos->current_thread = snapshot.instances[current].current;
	int index = rt8_find_thread(rtos, rtos->current_thread);
	/* GDB can cache live registers before qSymbol; list that context first. */
	if (index > 0) {
		struct thread_detail detail = rtos->thread_details[index];
		memmove(rtos->thread_details + 1, rtos->thread_details, index * sizeof(detail));
		rtos->thread_details[0] = detail;
	}
	rtos->current_threadid = rt8_find_thread(rtos, selected) >= 0 ? selected : -1;
	return ERROR_OK;

error:
	LOG_ERROR("chibios-rt8 registry integrity/read check failed");
	rtos_free_threadlist(rtos);
	return retval;
}

static int rt8_context(struct rtos *rtos, const struct rt8_layout *layout,
		uint32_t thread, struct rt8_context *context)
{
	const uint16_t *f = layout->field;
	const uint16_t *p = layout->port;
	struct target *target = rtos->target;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	uint32_t base, end, sp, exc_return, xpsr;
	unsigned int frame_size;
	int retval;

	memset(context, 0, sizeof(*context));
	if (!is_armv7m(armv7m) || !rt8_range(thread, f[RT8_THREAD_SIZE]))
		return ERROR_FAIL;
	enum arm_arch arch = layout->port_id <= 2 ? ARM_ARCH_V6M :
		layout->port_id <= 4 ? ARM_ARCH_V7M : ARM_ARCH_V8M;
	if (armv7m->arm.arch != arch)
		return ERROR_FAIL;
	retval = rt8_read_pointer(target, thread + f[RT8_THREAD_BASE], &base);
	if (retval != ERROR_OK)
		return retval;
	retval = rt8_read_pointer(target, thread + f[RT8_THREAD_END], &end);
	if (retval != ERROR_OK)
		return retval;
	retval = rt8_read_pointer(target, thread + f[RT8_THREAD_CTX] + p[RT8_CTX_SP], &sp);
	if (retval != ERROR_OK)
		return retval;
	if (!base || end <= base || sp < base || sp >= end || (sp & 3))
		return ERROR_FAIL;
	if (p[RT8_CTX_REGS] == RT8_NONE) {
		if (f[RT8_INNER_SIZE] > end - sp)
			return ERROR_FAIL;
		context->inner = sp;
		context->sp = sp + f[RT8_INNER_SIZE];
		return ERROR_OK;
	}
	context->inner = thread + f[RT8_THREAD_CTX] + p[RT8_CTX_REGS];
	retval = rt8_read_pointer(target, context->inner + p[RT8_INNER_EXC_RETURN], &exc_return);
	if (retval != ERROR_OK)
		return retval;
	uint32_t basic_return = exc_return | BIT(4);
	if (basic_return != 0xfffffffd &&
			(layout->port_id != 6 || basic_return != 0xffffffbc))
		return ERROR_FAIL;
	context->extended = !(exc_return & BIT(4));
	if (context->extended && !(p[RT8_PORT_FLAGS] & RT8_FPU))
		return ERROR_FAIL;
	frame_size = context->extended ? p[RT8_FRAME_SIZE] : 32;
	if (frame_size > end - sp)
		return ERROR_FAIL;
	retval = rt8_read_pointer(target, sp + p[RT8_FRAME_XPSR], &xpsr);
	if (retval != ERROR_OK)
		return retval;
	if (!(xpsr & BIT(24)))
		return ERROR_FAIL;
	if (xpsr & BIT(9))
		frame_size += 4;
	if (frame_size > end - sp)
		return ERROR_FAIL;
	context->frame = sp;
	context->sp = sp + frame_size;
	return ERROR_OK;
}

static int rt8_live_registers(struct target *target, struct rtos_reg **result, int *count)
{
	struct reg **registers;
	int number;
	int retval = target_get_gdb_reg_list(target, &registers, &number, REG_CLASS_GENERAL);

	if (retval != ERROR_OK)
		return retval;
	struct rtos_reg *out = calloc(number, sizeof(*out));
	if (!out) {
		free(registers);
		return ERROR_FAIL;
	}
	*count = 0;
	for (int i = 0; i < number; i++) {
		struct reg *reg = registers[i];
		if (!reg || !reg->exist || reg->hidden)
			continue;
		if (reg->size > sizeof(out[0].value) * 8) {
			retval = ERROR_FAIL;
			break;
		}
		if (!reg->valid) {
			retval = reg->type->get(reg);
			if (retval != ERROR_OK)
				break;
		}
		out[*count].number = reg->number;
		out[*count].size = reg->size;
		memcpy(out[*count].value, reg->value, DIV_ROUND_UP(reg->size, 8));
		(*count)++;
	}
	free(registers);
	if (retval != ERROR_OK) {
		free(out);
		*count = 0;
		return retval;
	}
	*result = out;
	return ERROR_OK;
}

static int rt8_get_thread_reg_list(struct rtos *rtos, int64_t thread,
		struct rtos_reg **reg_list, int *num_regs)
{
	struct rt8_snapshot snapshot;
	struct rt8_context context;
	struct rtos_reg *regs;
	int retval;

	*reg_list = NULL;
	*num_regs = 0;
	if (rt8_find_thread(rtos, thread) < 0)
		return ERROR_FAIL;
	retval = rt8_snapshot(rtos, &snapshot);
	if (retval != ERROR_OK)
		return retval;
	struct target *live = rt8_live_target(&snapshot, thread);
	if (!snapshot.count && thread == 1)
		live = rtos->target;
	if (live)
		return rt8_live_registers(live, reg_list, num_regs);
	retval = rt8_context(rtos, &snapshot.layout, thread, &context);
	if (retval != ERROR_OK)
		return retval;
	regs = calloc(ARMV7M_NUM_CORE_REGS, sizeof(*regs));
	if (!regs)
		return ERROR_FAIL;
	const uint16_t *p = snapshot.layout.port;
	for (unsigned int i = 0; i < ARMV7M_NUM_CORE_REGS; i++) {
		uint32_t address = 0;
		regs[i].number = i;
		regs[i].size = 32;
		if (i >= ARMV7M_R4 && i <= ARMV7M_R7)
			address = context.inner + p[RT8_INNER_R4] + 4 * (i - ARMV7M_R4);
		else if (i >= ARMV7M_R8 && i <= ARMV7M_R11)
			address = context.inner + p[RT8_INNER_R8] + 4 * (i - ARMV7M_R8);
		else if (i == ARMV7M_R13)
			target_buffer_set_u32(rtos->target, regs[i].value, context.sp);
		else if (context.frame) {
			if (i <= ARMV7M_R3)
				address = context.frame + p[RT8_FRAME_R0] + 4 * i;
			else if (i == ARMV7M_R12)
				address = context.frame + p[RT8_FRAME_R12];
			else if (i == ARMV7M_R14)
				address = context.frame + p[RT8_FRAME_LR];
			else if (i == ARMV7M_PC)
				address = context.frame + p[RT8_FRAME_PC];
			else if (i == ARMV7M_XPSR)
				address = context.frame + p[RT8_FRAME_XPSR];
		} else if (i == ARMV7M_PC) {
			address = context.inner + p[RT8_INNER_LR];
		}
		if (address) {
			retval = target_read_buffer(rtos->target, address, 4, regs[i].value);
			if (retval != ERROR_OK) {
				free(regs);
				return retval;
			}
		}
	}
	*reg_list = regs;
	*num_regs = ARMV7M_NUM_CORE_REGS;
	return ERROR_OK;
}

static bool rt8_detect(struct target *target)
{
	struct rt8_layout layout;
	return !target->smp && rt8_target_supported(target) &&
		rt8_read_layout(target->rtos, &layout) == ERROR_OK && !(layout.flags & RT8_SMP);
}

static int rt8_get_thread_reg_value(struct rtos *rtos, int64_t thread,
		uint32_t number, uint32_t *size, uint8_t **value)
{
	struct rt8_snapshot snapshot;
	struct rt8_context context;
	uint8_t data[8];
	unsigned int bytes = 4;
	uint32_t address = 0;
	struct reg *reg;
	int retval;

	*value = NULL;
	*size = 0;
	if (rt8_find_thread(rtos, thread) < 0)
		return ERROR_FAIL;
	retval = rt8_snapshot(rtos, &snapshot);
	if (retval != ERROR_OK)
		return retval;
	struct target *live = rt8_live_target(&snapshot, thread);
	if (!snapshot.count && thread == 1)
		live = rtos->target;
	/* These ports switch PSP; MSP is the shared per-core exception stack,
	 * which GDB also needs when unwinding through the startup frame. */
	if (number == ARMV7M_MSP && !live) {
		live = rt8_owner_target(rtos, &snapshot, thread);
		if (!live)
			return ERROR_FAIL;
	}
	if (live) {
		reg = register_get_by_number(live->reg_cache, number, true);
		if (!reg || !reg->exist || reg->hidden || reg->size > sizeof(data) * 8)
			return ERROR_FAIL;
		if (!reg->valid) {
			retval = reg->type->get(reg);
			if (retval != ERROR_OK)
				return retval;
		}
		*size = reg->size;
		bytes = DIV_ROUND_UP(*size, 8);
		memcpy(data, reg->value, bytes);
	} else if (number < ARMV7M_NUM_CORE_REGS) {
		struct rtos_reg *regs;
		int count;
		retval = rt8_get_thread_reg_list(rtos, thread, &regs, &count);
		if (retval != ERROR_OK)
			return retval;
		memcpy(data, regs[number].value, 4);
		free(regs);
		*size = 32;
	} else {
		retval = rt8_context(rtos, &snapshot.layout, thread, &context);
		if (retval != ERROR_OK)
			return retval;
		const uint16_t *p = snapshot.layout.port;
		unsigned int offset = RT8_NONE;
		*size = 32;
		if (number == ARMV7M_PSP) {
			target_buffer_set_u32(rtos->target, data, context.sp);
		} else if (number == ARMV7M_BASEPRI || number == ARMV7M_CONTROL || number == ARMV8M_PSPLIM) {
			offset = p[number == ARMV7M_BASEPRI ? RT8_INNER_BASEPRI :
				number == ARMV7M_CONTROL ? RT8_INNER_CONTROL : RT8_INNER_PSPLIM];
			if (offset == RT8_NONE)
				goto unavailable;
			uint64_t word;
			retval = rt8_read_uint(rtos->target, context.inner + offset, 4, &word);
			if (retval != ERROR_OK)
				return retval;
			if (number == ARMV8M_PSPLIM) {
				target_buffer_set_u32(rtos->target, data, word);
			} else {
				*size = number == ARMV7M_CONTROL ? 3 : 8;
				bytes = 1;
				data[0] = word & ((1U << *size) - 1);
			}
		} else if ((number >= ARMV7M_D0 && number <= ARMV7M_D15) || number == ARMV7M_FPSCR) {
			if (!(p[RT8_PORT_FLAGS] & RT8_FPU))
				goto unavailable;
			if (number >= ARMV7M_D8 && number <= ARMV7M_D15) {
				if (p[RT8_FPU_MODE] == 2 && !context.extended)
					goto unavailable;
				address = context.inner + p[RT8_INNER_S16] + 8 * (number - ARMV7M_D8);
			} else if (number == ARMV7M_FPSCR && p[RT8_INNER_FPSCR] != RT8_NONE) {
				address = context.inner + p[RT8_INNER_FPSCR];
			} else {
				if (!context.frame || !context.extended)
					goto unavailable;
				/* Reserved lazy-stack space is not a saved floating-point
				 * context. Do not expose its previous memory contents. */
				/* A pending lazy save can belong to any core, even if the
				 * scheduler has already changed the thread's owner. */
				for (unsigned int i = 0; i < snapshot.count; i++) {
					uint32_t fpccr, fpcar;
					struct target *cpu = snapshot.instances[i].target;
					retval = rt8_read_pointer(cpu, 0xe000ef34, &fpccr);
					if (retval != ERROR_OK)
						return retval;
					if (fpccr & BIT(0)) {
						retval = rt8_read_pointer(cpu, 0xe000ef38, &fpcar);
						if (retval != ERROR_OK)
							return retval;
						if (fpcar >= context.frame && fpcar < context.sp)
							goto unavailable;
					}
				}
				address = context.frame + (number == ARMV7M_FPSCR ? p[RT8_FRAME_FPSCR] :
					p[RT8_FRAME_S0] + 8 * (number - ARMV7M_D0));
			}
			bytes = number == ARMV7M_FPSCR ? 4 : 8;
			*size = bytes * 8;
			retval = target_read_buffer(rtos->target, address, bytes, data);
			if (retval != ERROR_OK)
				return retval;
			/* S(2n) is the low word of Dn, including on big-endian targets. */
			if (bytes == 8 && rtos->target->endianness == TARGET_BIG_ENDIAN) {
				uint8_t low[4];
				memcpy(low, data, 4);
				memcpy(data, data + 4, 4);
				memcpy(data + 4, low, 4);
			}
		} else {
			goto unavailable;
		}
	}
	*value = malloc(bytes);
	if (!*value)
		return ERROR_FAIL;
	memcpy(*value, data, bytes);
	return ERROR_OK;

unavailable:
	/* GDB's unavailable-register encoding avoids substituting live CPU
	 * state and lets it distinguish unsaved registers from read failures. */
	reg = register_get_by_number(rtos->target->reg_cache, number, true);
	if (!reg || !reg->exist || reg->hidden || !reg->size)
		return ERROR_FAIL;
	*size = reg->size;
	return ERROR_OK;
}

static int rt8_target_for_threadid(struct connection *connection, int64_t thread,
		struct target **result)
{
	struct target *target = get_target_from_connection(connection);
	struct rtos *rtos = target->rtos;
	struct rt8_snapshot snapshot;

	/* Several GDB callers ignore errors from this callback. Always provide
	 * a usable target, including Ctrl-C while the CPUs are still running. */
	*result = target;
	if (!rtos || !target->smp || target->state != TARGET_HALTED)
		return ERROR_OK;
	/* GDB sends Hg0 before offering qSymbol during attachment. */
	if (thread <= 0 && !rtos->current_thread)
		return ERROR_OK;
	if (thread <= 0)
		thread = rtos->current_thread;
	int retval = rt8_snapshot(rtos, &snapshot);
	if (retval != ERROR_OK)
		return retval;
	if (!snapshot.count && thread == 1) {
		*result = rtos->target;
		return ERROR_OK;
	}
	if (rt8_find_thread(rtos, thread) < 0)
		return ERROR_FAIL;
	struct target *live = rt8_live_target(&snapshot, thread);
	if (live)
		*result = live;
	return ERROR_OK;
}

static bool rt8_needs_fake_step(struct target *target, int64_t thread)
{
	struct rtos *rtos = target->rtos;
	struct rt8_snapshot snapshot;
	if (!target->smp)
		return thread != rtos->current_thread;
	if (thread <= 0 && !rtos->current_thread)
		return false;
	if (thread <= 0)
		thread = rtos->current_thread;
	if (rt8_find_thread(rtos, thread) < 0 || rt8_snapshot(rtos, &snapshot) != ERROR_OK)
		return true;
	return !rt8_live_target(&snapshot, thread) && (snapshot.count || thread != 1);
}

static int rt8_thread_packet(struct connection *connection, const char *packet, int size)
{
	struct target *target = get_target_from_connection(connection);
	if (target->smp && size > 2 && packet[0] == 'H' && packet[1] == 'g') {
		threadid_t thread;
		struct target *cpu;
		if (sscanf(packet + 2, "%16" SCNx64, &thread) != 1 ||
				rt8_target_for_threadid(connection, thread, &cpu) != ERROR_OK) {
			gdb_put_packet(connection, "E01", 3);
			return ERROR_OK;
		}
		/* GDB's ordinary CPU operations must follow selection of a live
		 * thread on either core. Suspended contexts remain read-only. */
		struct gdb_service *service = connection->service->priv;
		service->target = cpu;
	}
	return rtos_thread_packet(connection, packet, size);
}

static int rt8_set_reg(struct rtos *rtos, uint32_t number, uint8_t *value)
{
	struct rt8_snapshot snapshot;
	int retval = rt8_snapshot(rtos, &snapshot);
	if (retval != ERROR_OK)
		return retval;
	struct target *live = rt8_live_target(&snapshot, rtos->current_threadid);
	if (!snapshot.count && rtos->current_threadid == 1)
		live = rtos->target;
	/* Never let GDB fall back to writing a CPU when a saved thread context
	 * was selected. Only live threads support register modification. */
	if (!live || rt8_find_thread(rtos, rtos->current_threadid) < 0)
		return ERROR_FAIL;
	struct reg *reg = register_get_by_number(live->reg_cache, number, true);
	if (!reg || !reg->exist || reg->hidden || !reg->type || !reg->type->set)
		return ERROR_FAIL;
	retval = reg->type->set(reg, value);
	return retval == ERROR_NOT_IMPLEMENTED ? ERROR_FAIL : retval;
}

static struct target *rt8_swbp_target(struct rtos *rtos, target_addr_t address,
		uint32_t length, enum breakpoint_type type)
{
	/* Shared code memory: keep software breakpoints on a stable target even
	 * when GDB changes its selected core. Hardware breakpoints use the group. */
	return rtos->target;
}

static int rt8_smp_init(struct target *target)
{
	struct target_list *head, *other;
	struct rtos *rtos = target->rtos;
	unsigned int count = 0;

	/* Called during configuration, before examination or symbol lookup. */
	foreach_smp_target(head, target->smp_targets) {
		struct target *cpu = head->target;
		if (strcmp(target_type_name(cpu), "cortex_m") || cpu->coreid < 0 ||
				cpu->rtos_auto_detect || (cpu->rtos && cpu->rtos->type != &rt8_rtos) ||
				++count > RT8_MAX_INSTANCES) {
			LOG_ERROR("chibios-rt8 SMP requires Cortex-M targets with explicit RTOS selection");
			return ERROR_FAIL;
		}
		foreach_smp_target(other, target->smp_targets) {
			if (other != head && other->target->coreid == cpu->coreid) {
				LOG_ERROR("chibios-rt8 SMP requires unique -coreid values matching ChibiOS core IDs");
				return ERROR_FAIL;
			}
		}
	}
	if (!count)
		return ERROR_FAIL;
	/* One firmware image and one GDB connection share symbols, selection
	 * and the thread list. Free independent objects before attaching aliases. */
	foreach_smp_target(head, target->smp_targets) {
		if (head->target->rtos != rtos)
			rtos_destroy(head->target);
	}
	foreach_smp_target(head, target->smp_targets)
		head->target->rtos = rtos;
	LOG_WARNING("chibios-rt8 SMP support is experimental and has not been hardware validated");
	return ERROR_OK;
}

static int rt8_create(struct target *target)
{
	if (!rt8_target_supported(target)) {
		LOG_ERROR("chibios-rt8 does not support target type %s", target_type_name(target));
		return ERROR_FAIL;
	}
	target->rtos->gdb_target_for_threadid = rt8_target_for_threadid;
	target->rtos->gdb_thread_packet = rt8_thread_packet;
	return ERROR_OK;
}

static int rt8_get_symbols(struct symbol_table_elem **symbols)
{
	*symbols = malloc(sizeof(rt8_symbols));
	if (!*symbols)
		return ERROR_FAIL;
	memcpy(*symbols, rt8_symbols, sizeof(rt8_symbols));
	return ERROR_OK;
}

const struct rtos_type rt8_rtos = {
	.name = "chibios-rt8",
	.detect_rtos = rt8_detect,
	.create = rt8_create,
	.smp_init = rt8_smp_init,
	.update_threads = rt8_update_threads,
	.get_thread_reg_list = rt8_get_thread_reg_list,
	.get_thread_reg_value = rt8_get_thread_reg_value,
	.get_symbol_list_to_lookup = rt8_get_symbols,
	.set_reg = rt8_set_reg,
	.needs_fake_step = rt8_needs_fake_step,
	.swbp_target = rt8_swbp_target,
};
