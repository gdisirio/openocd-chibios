// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos/rtos.h"
#include "helper/log.h"
#include "target/armv7m.h"
#include "target/target_type.h"

/* Exercise the driver through its public callbacks with a byte-addressed
 * target. Fixtures use the RT 7 ABI, independently of the driver's C types.
 */
#define CHECK(expr) do { \
	if (!(expr)) { \
		fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
		exit(EXIT_FAILURE); \
	} \
} while (0)

enum {
	SIGNATURE = 0x100,
	SYSTEM = 0x200,
	INSTANCE = 0x300,
	MAIN_THREAD = 0x400,
	WORKER_THREAD = 0x500,
	STACK = 0x600,
	REGISTRY = INSTANCE + 64,
};

static uint8_t memory[4096];
static unsigned int reads;
static target_addr_t read_fault;
static struct target_type target_type = { .name = "cortex_m" };
static struct armv7m_common armv7m;
static struct target target;
static struct rtos rtos;

static void put_u32(uint32_t address, uint32_t value)
{
	CHECK(address <= sizeof(memory) - 4);
	for (unsigned int i = 0; i < 4; i++) {
		unsigned int shift = target.endianness == TARGET_BIG_ENDIAN ? 3 - i : i;
		memory[address + i] = value >> (shift * 8);
	}
}

const char *target_type_name(const struct target *t)
{
	return t->type->name;
}

int target_read_buffer(struct target *t, target_addr_t address, uint32_t size, uint8_t *buffer)
{
	CHECK(++reads < 100000);
	if (!address || address > sizeof(memory) || size > sizeof(memory) - address ||
			(read_fault && address <= read_fault && read_fault - address < size))
		return ERROR_FAIL;
	memcpy(buffer, memory + address, size);
	return ERROR_OK;
}

int target_read_u32(struct target *t, target_addr_t address, uint32_t *value)
{
	uint8_t bytes[4];
	int retval = target_read_buffer(t, address, sizeof(bytes), bytes);

	if (retval != ERROR_OK)
		return retval;
	*value = 0;
	for (unsigned int i = 0; i < 4; i++) {
		unsigned int shift = t->endianness == TARGET_BIG_ENDIAN ? 3 - i : i;
		*value |= (uint32_t)bytes[i] << (shift * 8);
	}
	return ERROR_OK;
}

int target_read_u8(struct target *t, target_addr_t address, uint8_t *value)
{
	return target_read_buffer(t, address, 1, value);
}

uint16_t target_buffer_get_u16(struct target *t, const uint8_t *buffer)
{
	if (t->endianness == TARGET_BIG_ENDIAN)
		return ((uint16_t)buffer[0] << 8) | buffer[1];
	return ((uint16_t)buffer[1] << 8) | buffer[0];
}

uint32_t target_buffer_get_u32(struct target *t, const uint8_t *buffer)
{
	uint32_t value = 0;

	for (unsigned int i = 0; i < 4; i++) {
		unsigned int shift = t->endianness == TARGET_BIG_ENDIAN ? 3 - i : i;

		value |= (uint32_t)buffer[i] << (shift * 8);
	}
	return value;
}

void target_buffer_set_u32(struct target *t, uint8_t *buffer, uint32_t value)
{
	for (unsigned int i = 0; i < 4; i++) {
		unsigned int shift = t->endianness == TARGET_BIG_ENDIAN ? 3 - i : i;

		buffer[i] = value >> (shift * 8);
	}
}

void log_printf_lf(enum log_levels level, const char *file, unsigned int line,
		const char *function, const char *format, ...)
{
}

void keep_alive(void)
{
}

char *alloc_printf(const char *format, ...)
{
	va_list args;
	char *text;
	int result;

	va_start(args, format);
	result = vasprintf(&text, format, args);
	va_end(args);
	return result < 0 ? NULL : text;
}

void rtos_free_threadlist(struct rtos *os)
{
	for (int i = 0; i < os->thread_count; i++) {
		free(os->thread_details[i].thread_name_str);
		free(os->thread_details[i].extra_info_str);
	}
	free(os->thread_details);
	os->thread_details = NULL;
	os->thread_count = 0;
	os->current_thread = 0;
	os->current_threadid = -1;
}

/* Check the stacking contract at the generic-reader boundary. Expected
 * offsets below are based on real RT 7 port frames, not this stub's output.
 */
static struct stack_register_offset saved_offsets[ARMV7M_NUM_CORE_REGS];
static unsigned int saved_size;
static uint32_t saved_sp;

int rtos_generic_stack_read(struct target *t, const struct rtos_register_stacking *stacking,
		int64_t sp, struct rtos_reg **reg_list, int *num_regs)
{
	CHECK(stacking->num_output_registers == ARMV7M_NUM_CORE_REGS);
	CHECK(stacking->stack_growth_direction == -1);
	memcpy(saved_offsets, stacking->register_offsets, sizeof(saved_offsets));
	saved_size = stacking->stack_registers_size;
	saved_sp = sp;
	*reg_list = NULL;
	*num_regs = 0;
	return sp ? ERROR_OK : ERROR_FAIL;
}

static void setup(enum target_endianness endianness)
{
	const uint8_t signature[] = {
		'm', 'a', 'i', 'n', 0, 44, 0, 0x38,
		4, 4, 96, 8, 12, 16, 20, 28, 32, 36, 37, 38, 39, 40,
		0, 0, 0, 0, 36, 4, 1, 0, 4, 0, 0, 0, 0, 0, 0,
		12, 0, 16, 64, 72, 76, 0,
	};

	memset(memory, 0, sizeof(memory));
	memset(&armv7m, 0, sizeof(armv7m));
	memset(&target, 0, sizeof(target));
	memset(&rtos, 0, sizeof(rtos));
	reads = 0;
	read_fault = 0;
	armv7m.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m.arm.arch = ARM_ARCH_V7M;
	target.type = &target_type;
	target.arch_info = &armv7m.arm;
	target.endianness = endianness;
	target.rtos = &rtos;
	target.state = TARGET_HALTED;
	rtos.type = &rt7_rtos;
	rtos.target = &target;
	CHECK(rt7_rtos.create(&target) == ERROR_OK);
	CHECK(rt7_rtos.get_symbol_list_to_lookup(&rtos.symbols) == ERROR_OK);
	CHECK(!strcmp(rtos.symbols[0].symbol_name, "ch_system"));
	CHECK(!strcmp(rtos.symbols[1].symbol_name, "ch_debug"));
	CHECK(!rtos.symbols[2].symbol_name);
	rtos.symbols[0].address = SYSTEM;
	rtos.symbols[1].address = SIGNATURE;
	memcpy(memory + SIGNATURE, signature, sizeof(signature));
	if (endianness == TARGET_BIG_ENDIAN) {
		memory[SIGNATURE + 6] = 0x38;
		memory[SIGNATURE + 7] = 0;
	}
	put_u32(SYSTEM + 4, INSTANCE);
	put_u32(INSTANCE + 12, MAIN_THREAD);
	put_u32(REGISTRY, MAIN_THREAD + 16);
	put_u32(REGISTRY + 4, WORKER_THREAD + 16);
	put_u32(MAIN_THREAD + 16, WORKER_THREAD + 16);
	put_u32(MAIN_THREAD + 20, REGISTRY);
	put_u32(WORKER_THREAD + 16, REGISTRY);
	put_u32(WORKER_THREAD + 20, MAIN_THREAD + 16);
	put_u32(MAIN_THREAD + 28, SIGNATURE);
	put_u32(WORKER_THREAD + 28, 0);
	put_u32(WORKER_THREAD + 12, STACK);
	memory[MAIN_THREAD + 36] = 1;
	memory[WORKER_THREAD + 36] = 8;
}

static void teardown(void)
{
	rtos_free_threadlist(&rtos);
	free(rtos.symbols);
}

static void check_threads(void)
{
	CHECK(rt7_rtos.detect_rtos(&target));
	CHECK(rt7_rtos.update_threads(&rtos) == ERROR_OK);
	CHECK(rtos.thread_count == 2);
	CHECK(rtos.current_thread == MAIN_THREAD);
	CHECK(rtos.thread_details[0].threadid == MAIN_THREAD);
	CHECK(rtos.thread_details[1].threadid == WORKER_THREAD);
	CHECK(!strcmp(rtos.thread_details[0].thread_name_str, "main"));
	CHECK(!strcmp(rtos.thread_details[1].thread_name_str, "No Name"));
	CHECK(!strcmp(rtos.thread_details[1].extra_info_str, "State: SLEEPING"));
}

static void check_stack(unsigned int size, enum arm_arch arch)
{
	struct rtos_reg *regs;
	int count;

	memory[SIGNATURE + 26] = size;
	armv7m.arm.arch = arch;
	CHECK(rt7_rtos.get_thread_reg_list(&rtos, WORKER_THREAD, &regs, &count) == ERROR_OK);
	CHECK(saved_sp == STACK);
	CHECK(saved_size == size);
	for (unsigned int i = 0; i < ARMV7M_NUM_CORE_REGS; i++) {
		CHECK(saved_offsets[i].number == i);
		CHECK(saved_offsets[i].width_bits == 32);
	}
	CHECK(saved_offsets[ARMV7M_R0].offset == -1);
	CHECK(saved_offsets[ARMV7M_R12].offset == -1);
	CHECK(saved_offsets[ARMV7M_R13].offset == -2);
	CHECK(saved_offsets[ARMV7M_R14].offset == -1);
	CHECK(saved_offsets[ARMV7M_PC].offset == (int)size - 4);
	CHECK(saved_offsets[ARMV7M_XPSR].offset == -1);
}

static void test_registry(void)
{
	for (unsigned int big = 0; big < 2; big++) {
		setup(big ? TARGET_BIG_ENDIAN : TARGET_LITTLE_ENDIAN);
		check_threads();
		/* Repeated updates must free the old names and details. */
		check_threads();
		/* A name at the end of readable memory must not be over-read. */
		memcpy(memory + sizeof(memory) - 4, "end", 4);
		put_u32(WORKER_THREAD + 28, sizeof(memory) - 4);
		memory[WORKER_THREAD + 36] = 255;
		CHECK(rt7_rtos.update_threads(&rtos) == ERROR_OK);
		CHECK(!strcmp(rtos.thread_details[1].thread_name_str, "end"));
		CHECK(!strcmp(rtos.thread_details[1].extra_info_str, "State: Unknown"));
		/* Initialization and reset expose no stale thread list. */
		put_u32(SYSTEM + 4, 0);
		CHECK(rt7_rtos.update_threads(&rtos) == ERROR_OK);
		CHECK(rtos.thread_count == 1 && rtos.current_thread == 1);
		CHECK(!strcmp(rtos.thread_details[0].thread_name_str, "Current Execution"));
		put_u32(SYSTEM + 4, INSTANCE);
		put_u32(INSTANCE + 12, 0);
		CHECK(rt7_rtos.update_threads(&rtos) == ERROR_OK);
		CHECK(rtos.thread_count == 1 && rtos.thread_details[0].threadid == 1);
		teardown();
	}
}

static void test_corruption(void)
{
	const uint32_t bad_nodes[] = { 0, 3, 4, WORKER_THREAD + 16, 0xfffffff0 };

	for (unsigned int i = 0; i < ARRAY_SIZE(bad_nodes); i++) {
		setup(TARGET_LITTLE_ENDIAN);
		check_threads();
		put_u32(WORKER_THREAD + 16, bad_nodes[i]);
		CHECK(rt7_rtos.update_threads(&rtos) != ERROR_OK);
		CHECK(!rtos.thread_details && !rtos.thread_count && !rtos.current_thread);
		teardown();
	}
	setup(TARGET_LITTLE_ENDIAN);
	put_u32(REGISTRY + 4, MAIN_THREAD + 16);
	CHECK(rt7_rtos.update_threads(&rtos) != ERROR_OK);
	CHECK(!rtos.thread_details);
	put_u32(REGISTRY + 4, WORKER_THREAD + 16);
	put_u32(INSTANCE + 12, 0x700);
	CHECK(rt7_rtos.update_threads(&rtos) != ERROR_OK);
	put_u32(INSTANCE + 12, MAIN_THREAD);
	read_fault = WORKER_THREAD + 36;
	CHECK(rt7_rtos.update_threads(&rtos) != ERROR_OK);
	CHECK(!rtos.thread_details);
	teardown();
}

static void test_current_thread(void)
{
	setup(TARGET_LITTLE_ENDIAN);
	check_threads();
	/* GDB already selected main, but the worker owns the live registers. */
	rtos.current_threadid = MAIN_THREAD;
	put_u32(INSTANCE + 12, WORKER_THREAD);
	CHECK(rt7_rtos.update_threads(&rtos) == ERROR_OK);
	CHECK(rtos.thread_details[0].threadid == WORKER_THREAD);
	CHECK(rtos.thread_details[1].threadid == MAIN_THREAD);
	CHECK(rtos.current_thread == WORKER_THREAD);
	CHECK(rtos.current_threadid == MAIN_THREAD);
	/* A selected thread which no longer exists must not be retained. */
	rtos.current_threadid = 0x700;
	CHECK(rt7_rtos.update_threads(&rtos) == ERROR_OK);
	CHECK(rtos.current_threadid == -1);
	teardown();
}

static void test_signature(void)
{
	const unsigned int fields[] = { 0, 4, 5, 7, 8, 10, 14, 28 };
	const uint8_t invalid[] = { 'x', 1, 22, 0x40, 8, 20, 24, 0 };

	for (unsigned int i = 0; i < ARRAY_SIZE(fields); i++) {
		setup(TARGET_LITTLE_ENDIAN);
		memory[SIGNATURE + fields[i]] = invalid[i];
		CHECK(!rt7_rtos.detect_rtos(&target));
		CHECK(rt7_rtos.update_threads(&rtos) != ERROR_OK);
		teardown();
	}
	setup(TARGET_LITTLE_ENDIAN);
	memory[SIGNATURE + 5] = 60;
	check_threads();
	rtos.symbols[0].address = 0;
	CHECK(!rt7_rtos.detect_rtos(&target));
	rtos.symbols[0].address = SYSTEM;
	memory[SIGNATURE + 31] = 12; /* SMP registry. */
	CHECK(rt7_rtos.update_threads(&rtos) != ERROR_OK);
	teardown();
}

static void test_contexts(void)
{
	struct rtos_reg *regs;
	int count;

	setup(TARGET_LITTLE_ENDIAN);
	check_threads();
	check_stack(36, ARM_ARCH_V7M);
	CHECK(saved_offsets[ARMV7M_R4].offset == 0);
	CHECK(saved_offsets[ARMV7M_R11].offset == 28);
	check_stack(100, ARM_ARCH_V7M); /* S16-S31 precede R4. */
	CHECK(saved_offsets[ARMV7M_R4].offset == 64);
	check_stack(132, ARM_ARCH_V7M); /* Four switched MPU regions and FPU. */
	CHECK(saved_offsets[ARMV7M_R4].offset == 96);
	check_stack(104, ARM_ARCH_V8M); /* Stack limit and FPU. */
	CHECK(saved_offsets[ARMV7M_R4].offset == 68);
	check_stack(40, ARM_ARCH_V8M); /* Stack limit, no FPU. */
	CHECK(saved_offsets[ARMV7M_R4].offset == 4);
	check_stack(36, ARM_ARCH_V6M);
	CHECK(saved_offsets[ARMV7M_R4].offset == 16);
	CHECK(saved_offsets[ARMV7M_R7].offset == 28);
	CHECK(saved_offsets[ARMV7M_R8].offset == 0);
	CHECK(saved_offsets[ARMV7M_R11].offset == 12);
	CHECK(rt7_rtos.get_thread_reg_list(&rtos, 0x700, &regs, &count) != ERROR_OK);
	put_u32(WORKER_THREAD + 12, 0);
	CHECK(rt7_rtos.get_thread_reg_list(&rtos, WORKER_THREAD, &regs, &count) != ERROR_OK);
	put_u32(WORKER_THREAD + 12, STACK);
	memory[SIGNATURE + 26] = 35;
	CHECK(rt7_rtos.get_thread_reg_list(&rtos, WORKER_THREAD, &regs, &count) != ERROR_OK);
	teardown();
}

static void test_syscall_layout(void)
{
	setup(TARGET_LITTLE_ENDIAN);
	/* ARMv7-M with PORT_USE_SYSCALL: port_context grows by two words,
	 * shifting the registry node and all following fields.
	 */
	memory[SIGNATURE + 10] = 104;
	memory[SIGNATURE + 13] = 24;
	memory[SIGNATURE + 14] = 28;
	memory[SIGNATURE + 15] = 36;
	memory[SIGNATURE + 17] = 44;
	memset(memory + MAIN_THREAD, 0, 104);
	memset(memory + WORKER_THREAD, 0, 104);
	put_u32(REGISTRY, MAIN_THREAD + 24);
	put_u32(REGISTRY + 4, WORKER_THREAD + 24);
	put_u32(MAIN_THREAD + 24, WORKER_THREAD + 24);
	put_u32(MAIN_THREAD + 28, REGISTRY);
	put_u32(WORKER_THREAD + 24, REGISTRY);
	put_u32(WORKER_THREAD + 28, MAIN_THREAD + 24);
	put_u32(MAIN_THREAD + 36, SIGNATURE);
	put_u32(WORKER_THREAD + 12, STACK);
	memory[MAIN_THREAD + 44] = 1;
	memory[WORKER_THREAD + 44] = 8;
	check_threads();
	check_stack(100, ARM_ARCH_V7M);
	CHECK(saved_offsets[ARMV7M_R4].offset == 64);
	teardown();
}

static void setup_alt(bool fpu, bool syscall, unsigned int regions,
		bool extended, bool aligned)
{
	unsigned int intctx_size = 40 + (fpu ? 64 : 0) + (syscall ? 4 : 0);
	unsigned int node = 12 + 4 + intctx_size + regions * 8 + (syscall ? 12 : 0);
	unsigned int name = node + 12;
	unsigned int state = node + 20;

	/* SP, BASEPRI, R4-R11, optional CONTROL, EXC_RETURN, optional S16-S31,
	 * switched MPU regions, optional syscall pointers, then registry node.
	 */
	memory[SIGNATURE + 10] = node + 64;
	memory[SIGNATURE + 13] = node;
	memory[SIGNATURE + 14] = node + 4;
	memory[SIGNATURE + 15] = name;
	memory[SIGNATURE + 17] = state;
	memory[SIGNATURE + 26] = intctx_size;
	memset(memory + MAIN_THREAD, 0, 256);
	memset(memory + WORKER_THREAD, 0, 256);
	put_u32(REGISTRY, MAIN_THREAD + node);
	put_u32(REGISTRY + 4, WORKER_THREAD + node);
	put_u32(MAIN_THREAD + node, WORKER_THREAD + node);
	put_u32(MAIN_THREAD + node + 4, REGISTRY);
	put_u32(WORKER_THREAD + node, REGISTRY);
	put_u32(WORKER_THREAD + node + 4, MAIN_THREAD + node);
	put_u32(MAIN_THREAD + name, SIGNATURE);
	memory[MAIN_THREAD + state] = 1;
	memory[WORKER_THREAD + state] = 8;
	put_u32(WORKER_THREAD + 12, STACK);
	put_u32(WORKER_THREAD + 16, 0x20); /* BASEPRI is not R4. */
	for (unsigned int i = 4; i < 12; i++)
		put_u32(WORKER_THREAD + 20 + (i - 4) * 4, 0x8a123400 + i);
	if (syscall)
		put_u32(WORKER_THREAD + 52, 3); /* CONTROL is not EXC_RETURN. */
	put_u32(WORKER_THREAD + (syscall ? 56 : 52), extended ? 0xffffffed : 0xfffffffd);
	for (unsigned int i = 0; i < 4; i++)
		put_u32(STACK + i * 4, 0x8a123400 + i);
	put_u32(STACK + 16, 0x8a12340c);
	put_u32(STACK + 20, 0x08012345);
	put_u32(STACK + 24, 0x08002346);
	put_u32(STACK + 28, aligned ? 0x61000200 : 0x61000000);
}

static void check_alt_registers(bool extended, bool aligned)
{
	struct rtos_reg *regs;
	int count;
	uint32_t expected;

	CHECK(rt7_rtos.get_thread_reg_list(&rtos, WORKER_THREAD, &regs, &count) == ERROR_OK);
	CHECK(count == ARMV7M_NUM_CORE_REGS);
	for (unsigned int i = 0; i < ARMV7M_NUM_CORE_REGS; i++) {
		CHECK(regs[i].number == i && regs[i].size == 32);
		if (i <= ARMV7M_R12)
			expected = 0x8a123400 + i;
		else if (i == ARMV7M_R13)
			expected = STACK + (extended ? 104 : 32) + (aligned ? 4 : 0);
		else if (i == ARMV7M_R14)
			expected = 0x08012345;
		else if (i == ARMV7M_PC)
			expected = 0x08002346;
		else
			expected = aligned ? 0x61000200 : 0x61000000;
		/* Check the bytes sent to GDB, including reconstructed SP. */
		for (unsigned int j = 0; j < 4; j++) {
			unsigned int shift = target.endianness == TARGET_BIG_ENDIAN ? 3 - j : j;

			CHECK(regs[i].value[j] == ((expected >> (shift * 8)) & 0xff));
		}
	}
	free(regs);
}

static void test_alt_contexts(void)
{
	/* Cross all optional context fields, both byte orders, both frame
	 * sizes and the hardware alignment word. FPU builds can have basic
	 * frames too, notably for threads which have not started yet.
	 */
	for (unsigned int options = 0; options < 32; options++) {
		bool big = options & 1;
		bool fpu = options & 2;
		bool syscall = options & 4;
		bool extended = options & 8;
		bool aligned = options & 16;

		if (extended && !fpu)
			continue;
		for (unsigned int regions = 0; regions <= 4; regions++) {
			setup(big ? TARGET_BIG_ENDIAN : TARGET_LITTLE_ENDIAN);
			setup_alt(fpu, syscall, regions, extended, aligned);
			check_threads();
			check_alt_registers(extended, aligned);
			teardown();
		}
	}
}

static void check_context_failure(void)
{
	struct rtos_reg *regs = (struct rtos_reg *)&rtos;
	int count = -1;

	CHECK(rt7_rtos.get_thread_reg_list(&rtos, WORKER_THREAD, &regs, &count) != ERROR_OK);
	CHECK(!regs && !count);
}

static void test_alt_invalid_contexts(void)
{
	const uint32_t bad_sp[] = { 0, STACK + 1, sizeof(memory) - 16, 0xfffffff0 };
	const uint32_t bad_return[] = { 0, 0xfffffff1, 0xfffffff9, 0xffffffe1, 0xffffffed };
	const unsigned int bad_size[] = { 0, 36, 41, 48, 100, 112 };
	const unsigned int bad_node[] = { 8, 52, 57, 60, 92 };

	setup(TARGET_LITTLE_ENDIAN);
	setup_alt(false, false, 0, false, false);
	check_threads();
	for (unsigned int i = 0; i < ARRAY_SIZE(bad_sp); i++) {
		put_u32(WORKER_THREAD + 12, bad_sp[i]);
		check_context_failure();
	}
	put_u32(WORKER_THREAD + 12, STACK);
	for (unsigned int i = 0; i < ARRAY_SIZE(bad_return); i++) {
		put_u32(WORKER_THREAD + 52, bad_return[i]);
		check_context_failure();
	}
	put_u32(WORKER_THREAD + 52, 0xfffffffd);
	for (unsigned int i = 0; i < ARRAY_SIZE(bad_size); i++) {
		memory[SIGNATURE + 26] = bad_size[i];
		check_context_failure();
	}
	memory[SIGNATURE + 26] = 40;
	for (unsigned int i = 0; i < ARRAY_SIZE(bad_node); i++) {
		memory[SIGNATURE + 13] = bad_node[i];
		memory[SIGNATURE + 14] = bad_node[i] + 4;
		check_context_failure();
	}
	memory[SIGNATURE + 13] = 56;
	memory[SIGNATURE + 14] = 60;
	read_fault = WORKER_THREAD + 52;
	check_context_failure();
	read_fault = STACK + 28;
	check_context_failure();
	read_fault = 0;
	/* An embedded context on ARMv8-M is not the ARMv7-M ALT port. */
	armv7m.arm.arch = ARM_ARCH_V8M;
	check_context_failure();
	armv7m.arm.arch = ARM_ARCH_V6M;
	check_context_failure();
	armv7m.arm.arch = ARM_ARCH_V7M;
	check_alt_registers(false, false);
	/* Do not read past a basic frame even in an FPU-enabled build. */
	setup_alt(true, true, 4, false, true);
	check_threads();
	read_fault = STACK + 32;
	check_alt_registers(false, true);
	teardown();
}

int main(void)
{
	test_registry();
	test_corruption();
	test_current_thread();
	test_signature();
	test_contexts();
	test_syscall_layout();
	test_alt_contexts();
	test_alt_invalid_contexts();
	return EXIT_SUCCESS;
}
