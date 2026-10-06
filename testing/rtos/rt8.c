// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos/rtos.h"
#include "helper/log.h"
#include "target/armv7m.h"
#include "target/register.h"
#include "target/target_type.h"
#include "target/smp.h"
#include "server/gdb_server.h"

/* Independent byte-level RT8 fixtures. No driver layout types are imported. */
#define CHECK(expr) do { \
	if (!(expr)) { \
		fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
		exit(EXIT_FAILURE); \
	} \
} while (0)

enum {
	SIGNATURE = 0x100, SYSTEM = 0x300, INSTANCE = 0x400,
	MAIN_THREAD = 0x800, WORKER_THREAD = 0xc00, STACK = 0x2200,
	SECOND_INSTANCE = 0x1000, SECOND_THREAD = 0x1400, SHARED_REGISTRY = SYSTEM + 48,
	REGISTRY = INSTANCE + 512, NODE = 320, NONE = 0xffff,
};

static uint8_t memory[0x4000];
static unsigned int reads;
static target_addr_t read_fault;
static uint32_t fpccr, fpcar;
static struct target_type target_type = { .name = "cortex_m" };
static struct armv7m_common armv7m;
static struct target target;
struct target *all_targets = &target;
static struct rtos *rt8_os;
static struct target secondary;
static struct armv7m_common secondary_arm;
static struct reg_cache caches[2];
static struct reg secondary_regs[ARMV7M_NUM_CORE_REGS];
static uint8_t secondary_values[ARMV7M_NUM_CORE_REGS][4];
static uint8_t secondary_msp[4];
static struct reg secondary_msp_reg = {
	.size = 32, .value = secondary_msp, .exist = true, .valid = true,
};
static uint32_t secondary_fpccr, secondary_fpcar;
static struct list_head smp_targets;
static struct target_list members[2];
static struct gdb_service gdb_service;
static struct service service = { .priv = &gdb_service };
static struct connection connection = { .service = &service };
static char reply[GDB_BUFFER_SIZE + 1];
int debug_level = LOG_LVL_ERROR;

/* Only RT8 is used; the real RTOS framework still references other drivers. */
#define EMPTY_RTOS(symbol) const struct rtos_type symbol = { .name = #symbol }
EMPTY_RTOS(rt7_rtos);
EMPTY_RTOS(chibios_rtos);
EMPTY_RTOS(chromium_ec_rtos);
EMPTY_RTOS(ecos_rtos);
EMPTY_RTOS(embkernel_rtos);
EMPTY_RTOS(freertos_rtos);
EMPTY_RTOS(linux_rtos);
EMPTY_RTOS(mqx_rtos);
EMPTY_RTOS(nuttx_rtos);
EMPTY_RTOS(riot_rtos);
EMPTY_RTOS(rtkernel_rtos);
EMPTY_RTOS(threadx_rtos);
EMPTY_RTOS(ucos_iii_rtos);
EMPTY_RTOS(zephyr_rtos);
EMPTY_RTOS(hwthread_rtos);

void command_print(struct command_invocation *cmd, const char *format, ...)
{
}

int gdb_put_packet(struct connection *conn, const char *buffer, int len)
{
	CHECK(len >= 0 && (size_t)len < sizeof(reply));
	memcpy(reply, buffer, len);
	reply[len] = '\0';
	return ERROR_OK;
}
static struct reg live_regs[ARMV7M_NUM_CORE_REGS];
static uint8_t live_values[ARMV7M_NUM_CORE_REGS][4];
static uint8_t msp_value[4];
static struct reg msp_reg = { .size = 32, .value = msp_value, .exist = true, .valid = true };
static struct reg other_reg = { .size = 32, .exist = true };
static unsigned int cases;

static unsigned int writes;
static int write_result;
static struct reg *written_reg;

static int set_register(struct reg *reg, uint8_t *value)
{
	writes++;
	written_reg = reg;
	if (write_result != ERROR_OK)
		return write_result;
	memcpy(reg->value, value, DIV_ROUND_UP(reg->size, 8));
	return ERROR_OK;
}

static const struct reg_arch_type reg_type = { .set = set_register };

static void put_u32(uint32_t address, uint32_t value)
{
	CHECK(address <= sizeof(memory) - 4);
	for (unsigned int i = 0; i < 4; i++) {
		unsigned int shift = target.endianness == TARGET_BIG_ENDIAN ? 3 - i : i;
		memory[address + i] = value >> (shift * 8);
	}
}

static void put_u16(uint32_t address, uint16_t value)
{
	CHECK(address <= sizeof(memory) - 2);
	memory[address] = target.endianness == TARGET_BIG_ENDIAN ? value >> 8 : value;
	memory[address + 1] = target.endianness == TARGET_BIG_ENDIAN ? value : value >> 8;
}

const char *target_type_name(const struct target *t)
{
	return t->type->name;
}

int target_read_buffer(struct target *t, target_addr_t address, uint32_t size, uint8_t *buffer)
{
	CHECK(++reads < 100000);
	if (address == 0xe000ef34 && size == 4) {
		target_buffer_set_u32(t, buffer, t == &secondary ? secondary_fpccr : fpccr);
		return ERROR_OK;
	}
	if (address == 0xe000ef38 && size == 4) {
		target_buffer_set_u32(t, buffer, t == &secondary ? secondary_fpcar : fpcar);
		return ERROR_OK;
	}
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

int target_get_gdb_reg_list(struct target *t, struct reg ***list, int *count,
		enum target_register_class reg_class)
{
	*count = ARMV7M_NUM_CORE_REGS;
	*list = calloc(*count, sizeof(**list));
	CHECK(*list);
	for (int i = 0; i < *count; i++)
		(*list)[i] = t == &secondary ? &secondary_regs[i] : &live_regs[i];
	return ERROR_OK;
}

struct reg *register_get_by_number(struct reg_cache *cache, uint32_t number, bool all)
{
	if (number < ARMV7M_NUM_CORE_REGS)
		return cache == &caches[1] ? &secondary_regs[number] : &live_regs[number];
	if (number == ARMV7M_MSP)
		return cache == &caches[1] ? &secondary_msp_reg : &msp_reg;
	if (number >= ARMV7M_LAST_REG)
		return NULL;
	other_reg.size = number >= ARMV7M_D0 && number <= ARMV7M_D15 ? 64 : 32;
	return &other_reg;
}

static void teardown(void)
{
	rtos_destroy(&target);
	rtos_destroy(&secondary);
	rt8_os = NULL;
}

static unsigned int setup(unsigned int port, bool fpu, bool control, bool mpu,
		bool extended, bool aligned, bool big)
{
	bool split = port == 4 || port == 6;
	unsigned int prefix = (fpu ? 68 : 0) + (port == 5 ? 4 : 0);
	unsigned int integer_size = (port == 4 ? 40 : 44) + (control ? 4 : 0);
	unsigned int inner_size = split ? integer_size + (fpu ? 64 : 0) : prefix + 36;
	unsigned int ctx_size = split ? 4 + inner_size + (mpu ? 4 : 0) : 4;
	const uint16_t fields[] = {
		16, 1, 0, 4, NONE, NONE,
		640, 400, 12, 0, 16, 512, NONE,
		384, ctx_size, inner_size, NODE, 300, 336, 8, 340, 341, 342,
		NONE, NONE, 348, 352, 12,
	};
	const uint16_t payload[] = {
		(fpu ? 1 : 0) | (mpu ? 2 : 0), 0, split ? 4 : NONE,
		port <= 2 ? 16 : split ? 4 : prefix,
		port <= 2 ? 0 : split ? 20 : prefix + 16,
		split ? NONE : prefix + 32,
		split ? (port == 4 ? integer_size - 4 : 36) : NONE,
		control ? (port == 4 ? 36 : 44) : NONE,
		split ? 0 : NONE,
		port == 6 ? 40 : port == 5 ? 0 : NONE,
		fpu ? (split ? integer_size : port == 5 ? 4 : 0) : NONE,
		fpu && !split ? prefix - 4 : NONE,
		fpu ? 104 : 32, 0, 16, 20, 24, 28,
		fpu ? 32 : NONE, fpu ? 96 : NONE,
		fpu ? (split ? 2 : 1) : 0,
		mpu ? 4 + inner_size : NONE, mpu ? 4 : 0, mpu ? 8 : 0,
		mpu ? 0 : NONE, mpu ? 4 : NONE,
	};

	memset(memory, 0, sizeof(memory));
	memset(&target, 0, sizeof(target));
	memset(&secondary, 0, sizeof(secondary));
	target.next = &secondary;
	INIT_LIST_HEAD(&smp_targets);
	target.smp_targets = &smp_targets;
	secondary.smp_targets = &smp_targets;
	target.reg_cache = &caches[0];
	secondary_fpccr = 0;
	secondary_fpcar = 0;
	gdb_service.target = &target;
	memset(&armv7m, 0, sizeof(armv7m));
	reads = 0;
	writes = 0;
	write_result = ERROR_OK;
	written_reg = NULL;
	read_fault = 0;
	fpccr = fpcar = 0;
	armv7m.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m.arm.arch = port <= 2 ? ARM_ARCH_V6M : port <= 4 ? ARM_ARCH_V7M : ARM_ARCH_V8M;
	target.type = &target_type;
	target.arch_info = &armv7m.arm;
	target.endianness = big ? TARGET_BIG_ENDIAN : TARGET_LITTLE_ENDIAN;
	target.state = TARGET_HALTED;
	target_set_examined(&target);
	CHECK(rtos_create(NULL, &target, "chibios-rt8") == ERROR_OK);
	rt8_os = target.rtos;
	CHECK(rt8_rtos.get_symbol_list_to_lookup(&rt8_os->symbols) == ERROR_OK);
	rt8_os->symbols[0].address = SYSTEM;
	rt8_os->symbols[1].address = SIGNATURE;
	memcpy(memory + SIGNATURE, "main", 5);
	memory[SIGNATURE + 5] = 126;
	put_u16(SIGNATURE + 6, 8 << 11);
	memory[SIGNATURE + 9] = 0xaa;
	memory[SIGNATURE + 11] = 2;
	memory[SIGNATURE + 12] = 2;
	memory[SIGNATURE + 13] = port;
	put_u16(SIGNATURE + 14, 74);
	put_u16(SIGNATURE + 16, 52);
	for (unsigned int i = 0; i < ARRAY_SIZE(fields); i++)
		put_u16(SIGNATURE + 18 + 2 * i, fields[i]);
	for (unsigned int i = 0; i < ARRAY_SIZE(payload); i++)
		put_u16(SIGNATURE + 74 + 2 * i, payload[i]);
	memory[SYSTEM] = 2;
	put_u32(SYSTEM + 4, INSTANCE);
	put_u32(INSTANCE + 12, MAIN_THREAD);
	put_u32(REGISTRY, MAIN_THREAD + NODE);
	put_u32(REGISTRY + 4, WORKER_THREAD + NODE);
	put_u32(MAIN_THREAD + NODE, WORKER_THREAD + NODE);
	put_u32(MAIN_THREAD + NODE + 4, REGISTRY);
	put_u32(WORKER_THREAD + NODE, REGISTRY);
	put_u32(WORKER_THREAD + NODE + 4, MAIN_THREAD + NODE);
	put_u32(MAIN_THREAD + 300, INSTANCE);
	put_u32(WORKER_THREAD + 300, INSTANCE);
	put_u32(MAIN_THREAD + 336, SIGNATURE);
	put_u32(MAIN_THREAD + 8, 128);
	put_u32(WORKER_THREAD + 8, 64);
	memory[MAIN_THREAD + 340] = 1;
	memory[WORKER_THREAD + 340] = 8;
	put_u32(WORKER_THREAD + 348, 0x2000);
	put_u32(WORKER_THREAD + 352, 0x2800);
	put_u32(WORKER_THREAD + 12, STACK);
	unsigned int inner = split ? WORKER_THREAD + 16 : STACK;
	for (unsigned int i = 0; i < 4; i++) {
		put_u32(inner + payload[3] + 4 * i, 0x10000004 + i);
		put_u32(inner + payload[4] + 4 * i, 0x10000008 + i);
	}
	if (split) {
		put_u32(inner, 0x80);
		put_u32(inner + payload[6], (port == 6 ? 0xffffffbc : 0xfffffffd) - (extended ? 16 : 0));
		for (unsigned int i = 0; i < 4; i++)
			put_u32(STACK + 4 * i, 0x10000000 + i);
		put_u32(STACK + 16, 0x1000000c);
		put_u32(STACK + 20, 0x08005679);
		put_u32(STACK + 24, 0x08001234);
		put_u32(STACK + 28, 0x01000000 | (aligned ? 512 : 0));
		if (control)
			put_u32(inner + payload[7], 3);
	} else {
		put_u32(inner + payload[5], 0x08001235);
	}
	if (port >= 5)
		put_u32(inner + payload[9], 0x2000);
	if (fpu) {
		for (unsigned int i = 0; i < 16; i++)
			put_u32(inner + payload[10] + 4 * i, 0x3f000010 + i);
		if (split) {
			for (unsigned int i = 0; i < 16; i++)
				put_u32(STACK + 32 + 4 * i, 0x3f000000 + i);
			put_u32(STACK + 96, 0x00400000);
		} else {
			put_u32(inner + payload[11], 0x00400000);
		}
	}
	for (unsigned int i = 0; i < ARMV7M_NUM_CORE_REGS; i++) {
		target_buffer_set_u32(&target, live_values[i], 0xa0000000 + i);
		live_regs[i] = (struct reg) {
			.number = i, .size = 32, .value = live_values[i], .exist = true, .valid = true,
			.type = &reg_type,
		};
	}
	return split ? (extended ? 104U : 32U) + (aligned ? 4U : 0U) : inner_size;
}

static void check_threads(void)
{
	CHECK(rt8_rtos.detect_rtos(&target));
	CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
	CHECK(rt8_os->thread_count == 2 && rt8_os->current_thread == MAIN_THREAD);
	CHECK(rt8_os->thread_details[0].threadid == MAIN_THREAD);
	CHECK(rt8_os->thread_details[1].threadid == WORKER_THREAD);
	CHECK(!strcmp(rt8_os->thread_details[0].thread_name_str, "main"));
	CHECK(!strcmp(rt8_os->thread_details[1].extra_info_str, "State: SLEEPING, Priority: 64, Core: 0"));
}

static uint64_t get_register(unsigned int number, unsigned int expected_bits)
{
	uint8_t *value;
	uint32_t size;
	CHECK(rt8_rtos.get_thread_reg_value(rt8_os, WORKER_THREAD, number, &size, &value) == ERROR_OK);
	CHECK(size == expected_bits);
	uint64_t result = 0;
	unsigned int bytes = DIV_ROUND_UP(size, 8);
	for (unsigned int i = 0; i < bytes; i++) {
		unsigned int shift = target.endianness == TARGET_BIG_ENDIAN ? bytes - 1 - i : i;
		result |= (uint64_t)value[i] << (8 * shift);
	}
	free(value);
	return result;
}

static void unavailable(unsigned int number)
{
	uint8_t *value = NULL;
	uint32_t size;
	CHECK(rt8_rtos.get_thread_reg_value(rt8_os, WORKER_THREAD, number, &size, &value) == ERROR_OK);
	CHECK(!value && size == (number >= ARMV7M_D0 && number <= ARMV7M_D15 ? 64U : 32U));
}

static void test_contexts(void)
{
	for (unsigned int port = 1; port <= 6; port++) {
		bool split = port == 4 || port == 6;
		for (unsigned int bits = 0; bits < 64; bits++) {
			bool fpu = bits & 1, control = bits & 2, mpu = bits & 4;
			bool extended = bits & 8, aligned = bits & 16, big = bits & 32;
			if ((fpu && port <= 2) || ((control || mpu || extended || aligned) && !split) ||
					(extended && !fpu))
				continue;
			unsigned int frame_size = setup(port, fpu, control, mpu, extended, aligned, big);
			check_threads();
			struct rtos_reg *regs;
			int count;
			CHECK(rt8_rtos.get_thread_reg_list(rt8_os, WORKER_THREAD, &regs, &count) == ERROR_OK);
			CHECK(count == ARMV7M_NUM_CORE_REGS);
			for (unsigned int i = 4; i <= 11; i++)
				CHECK(target_buffer_get_u32(&target, regs[i].value) == 0x10000000 + i);
			CHECK(target_buffer_get_u32(&target, regs[ARMV7M_R13].value) == STACK + frame_size);
			CHECK(target_buffer_get_u32(&target, regs[ARMV7M_PC].value) == (split ? 0x08001234U : 0x08001235U));
			if (split) {
				CHECK(target_buffer_get_u32(&target, regs[ARMV7M_R0].value) == 0x10000000);
				CHECK(target_buffer_get_u32(&target, regs[ARMV7M_R12].value) == 0x1000000c);
				CHECK(target_buffer_get_u32(&target, regs[ARMV7M_R14].value) == 0x08005679);
			}
			free(regs);
			CHECK(get_register(ARMV7M_PSP, 32) == STACK + frame_size);
			if (control)
				CHECK(get_register(ARMV7M_CONTROL, 3) == 3);
			if (split)
				CHECK(get_register(ARMV7M_BASEPRI, 8) == 0x80);
			if (port >= 5)
				CHECK(get_register(ARMV8M_PSPLIM, 32) == 0x2000);
			if (fpu && (!split || extended)) {
				CHECK(get_register(ARMV7M_D8, 64) == UINT64_C(0x3f0000113f000010));
				CHECK(get_register(ARMV7M_D15, 64) == UINT64_C(0x3f00001f3f00001e));
				CHECK(get_register(ARMV7M_FPSCR, 32) == 0x00400000);
			} else {
				unavailable(ARMV7M_D8);
			}
			if (extended) {
				CHECK(get_register(ARMV7M_D0, 64) == UINT64_C(0x3f0000013f000000));
				fpccr = 1;
				fpcar = STACK + 32;
				unavailable(ARMV7M_D0);
				unavailable(ARMV7M_FPSCR);
				fpcar = STACK + 0x100;
				CHECK(get_register(ARMV7M_D0, 64) == UINT64_C(0x3f0000013f000000));
			} else {
				unavailable(ARMV7M_D0);
			}
			target_buffer_set_u32(&target, msp_value, 0x3000);
			CHECK(get_register(ARMV7M_MSP, 32) == 0x3000);
			unavailable(ARMV7M_PRIMASK);
			CHECK(rt8_rtos.get_thread_reg_list(rt8_os, MAIN_THREAD, &regs, &count) == ERROR_OK);
			CHECK(target_buffer_get_u32(&target, regs[4].value) == 0xa0000004);
			free(regs);
			teardown();
			cases++;
		}
	}
}

static void test_registry(void)
{
	for (unsigned int big = 0; big < 2; big++) {
		setup(4, true, true, true, false, false, big);
		check_threads();
		rt8_os->current_threadid = WORKER_THREAD;
		check_threads();
		CHECK(rt8_os->current_threadid == WORKER_THREAD);
		put_u32(INSTANCE + 12, WORKER_THREAD);
		CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
		CHECK(rt8_os->thread_details[0].threadid == WORKER_THREAD);
		CHECK(rt8_os->current_thread == WORKER_THREAD);
		put_u32(INSTANCE + 12, MAIN_THREAD);
		memcpy(memory + sizeof(memory) - 4, "end", 4);
		put_u32(WORKER_THREAD + 336, sizeof(memory) - 4);
		CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
		CHECK(!strcmp(rt8_os->thread_details[1].thread_name_str, "end"));
		/* Configurations can vary enum and state widths. */
		memory[SIGNATURE + 11] = 10;
		put_u32(SYSTEM, 3);
		memory[SIGNATURE + 10] = 2;
		put_u32(WORKER_THREAD + 340, 8);
		CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
		CHECK(strstr(rt8_os->thread_details[1].extra_info_str, "SLEEPING"));
		put_u32(SYSTEM, 0);
		CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
		CHECK(rt8_os->thread_count == 1 && rt8_os->current_thread == 1);
		CHECK(!strcmp(rt8_os->thread_details[0].thread_name_str, "Current Execution"));
		teardown();
		cases++;
	}
}

static void test_bad_layouts(void)
{
	const struct { unsigned int offset, width, value; } mutations[] = {
		{0, 1, 'x'}, {4, 1, 1}, {5, 1, 73}, {6, 2, 7 << 11}, {6, 2, 9 << 11},
		{8, 1, 0x80}, {9, 1, 0xab}, {11, 1, 0x12}, {12, 1, 3}, {13, 1, 0}, {13, 1, 7},
		{14, 2, NONE}, {14, 2, 73}, {14, 2, 80}, {16, 2, 51}, {16, 2, 65535},
		{18, 2, 4}, {20, 2, 0}, {20, 2, 65}, {22, 2, NONE}, {24, 2, 15},
		{26, 2, 0}, {30, 2, 512}, {32, 2, NONE}, {34, 2, 638}, {40, 2, NONE},
		{44, 2, 320}, {46, 2, 2}, {48, 2, 8}, {50, 2, NONE}, {52, 2, 382},
		{54, 2, NONE}, {56, 2, 383}, {58, 2, 384}, {68, 2, NONE}, {72, 2, NONE},
		{74, 2, 4}, {76, 2, NONE}, {78, 2, NONE}, {80, 2, 65530}, {84, 2, 0},
		{86, 2, NONE}, {98, 2, 31}, {100, 2, 20}, {108, 2, NONE}, {114, 2, 3},
		{116, 2, 0}, {118, 2, 1}, {120, 2, 8},
	};
	for (unsigned int i = 0; i < ARRAY_SIZE(mutations); i++) {
		setup(4, false, false, false, false, false, i & 1);
		if (mutations[i].width == 1)
			memory[SIGNATURE + mutations[i].offset] = mutations[i].value;
		else
			put_u16(SIGNATURE + mutations[i].offset, mutations[i].value);
		CHECK(!rt8_rtos.detect_rtos(&target));
		CHECK(rt8_rtos.update_threads(rt8_os) != ERROR_OK);
		CHECK(rt8_os->thread_count == 0);
		teardown();
		cases++;
	}
}

static void test_faults(void)
{
	for (unsigned int fault = 0; fault < 16; fault++) {
		setup(6, true, true, true, true, false, false);
		check_threads();
		switch (fault) {
		case 0: put_u32(REGISTRY, 0); break;
		case 1: put_u32(WORKER_THREAD + NODE + 4, REGISTRY); break;
		case 2: put_u32(REGISTRY + 4, MAIN_THREAD + NODE); break;
		case 3: put_u32(WORKER_THREAD + NODE, MAIN_THREAD + NODE); break;
		case 4: put_u32(INSTANCE + 12, 0x1000); break;
		case 5: put_u32(WORKER_THREAD + 300, 0); break;
		case 6: put_u32(REGISTRY, 0xfffffffc); break;
		case 7: read_fault = WORKER_THREAD + 340; break;
		case 8: read_fault = SIGNATURE + 100; break;
		case 9: target.smp = true; break;
		case 10: target.state = TARGET_RUNNING; break;
		case 11: put_u32(SYSTEM + 4, 0xfffffffc); break;
		case 12: memory[SYSTEM] = 4; break;
		case 13: put_u32(WORKER_THREAD + 336, 0xffffffff); break;
		case 14: put_u16(SIGNATURE + 20, 2); put_u32(SYSTEM + 8, INSTANCE); break;
		case 15:
			memory[SIGNATURE + 8] = 1;
			put_u16(SIGNATURE + 26, 8);
			put_u16(SIGNATURE + 40, NONE);
			break;
		}
		CHECK(rt8_rtos.update_threads(rt8_os) != ERROR_OK);
		CHECK(!rt8_os->thread_count && !rt8_os->thread_details);
		teardown();
		cases++;
	}
	for (unsigned int fault = 0; fault < 9; fault++) {
		setup(6, false, true, true, false, false, false);
		check_threads();
		switch (fault) {
		case 0: put_u32(WORKER_THREAD + 12, 0); break;
		case 1: put_u32(WORKER_THREAD + 12, 0x27fc); break;
		case 2: put_u32(WORKER_THREAD + 12, STACK + 1); break;
		case 3: put_u32(WORKER_THREAD + 16 + 36, 0xfffffff9); break;
		case 4: put_u32(WORKER_THREAD + 16 + 36, 0xffffffac); break;
		case 5: put_u32(STACK + 28, 0); break;
		case 6: read_fault = STACK + 24; break;
		case 7: put_u32(WORKER_THREAD + 348, STACK + 4); break;
		case 8: armv7m.arm.arch = ARM_ARCH_V7M; break;
		}
		struct rtos_reg *regs = NULL;
		int count;
		CHECK(rt8_rtos.get_thread_reg_list(rt8_os, WORKER_THREAD, &regs, &count) != ERROR_OK);
		CHECK(!regs && !count);
		teardown();
		cases++;
	}
}

static void setup_smp(unsigned int port, bool shared, bool big)
{
	setup(port, port > 2, port == 4 || port == 6, false, port == 4 || port == 6, false, big);
	secondary = target;
	secondary.next = NULL;
	secondary_arm = armv7m;
	secondary.arch_info = &secondary_arm.arm;
	secondary.rtos = NULL;
	secondary.reg_cache = &caches[1];
	target.coreid = 3;
	secondary.coreid = 7;
	CHECK(rtos_create(NULL, &secondary, "chibios-rt8") == ERROR_OK);
	/* List order deliberately differs from the ChibiOS instance order. */
	members[0].target = &secondary;
	members[1].target = &target;
	list_add_tail(&members[0].lh, &smp_targets);
	list_add_tail(&members[1].lh, &smp_targets);
	target.smp = true;
	secondary.smp = true;
	CHECK(rtos_smp_init(&target) == ERROR_OK);
	CHECK(target.rtos == secondary.rtos && rt8_os->target == &target);
	CHECK(!rt8_rtos.detect_rtos(&target)); /* Explicit selection for SMP. */
	for (unsigned int i = 0; i < ARMV7M_NUM_CORE_REGS; i++) {
		target_buffer_set_u32(&secondary, secondary_values[i], 0xb0000000 + i);
		secondary_regs[i] = (struct reg) {
			.number = i, .size = 32, .value = secondary_values[i], .exist = true, .valid = true,
			.type = &reg_type,
		};
	}
	target_buffer_set_u32(&target, msp_value, 0x3100);
	target_buffer_set_u32(&secondary, secondary_msp, 0x3200);
	put_u16(SIGNATURE + 18, 80); /* system size */
	put_u16(SIGNATURE + 20, 8);  /* sparse instance slots */
	put_u32(SYSTEM + 4, 0);
	put_u32(SYSTEM + 4 + 3 * 4, INSTANCE);
	put_u32(SYSTEM + 4 + 7 * 4, SECOND_INSTANCE);
	put_u32(INSTANCE + 400, 3);
	put_u32(SECOND_INSTANCE + 400, 7);
	put_u32(SECOND_INSTANCE + 12, SECOND_THREAD);
	memcpy(memory + SECOND_THREAD, memory + MAIN_THREAD, 384);
	put_u32(SECOND_THREAD + 300, SECOND_INSTANCE);
	put_u32(WORKER_THREAD + 300, SECOND_INSTANCE);
	if (shared) {
		memory[SIGNATURE + 8] = 1;
		put_u16(SIGNATURE + 26, 48);
		put_u16(SIGNATURE + 40, NONE);
		put_u32(SHARED_REGISTRY, MAIN_THREAD + NODE);
		put_u32(SHARED_REGISTRY + 4, SECOND_THREAD + NODE);
		put_u32(MAIN_THREAD + NODE + 4, SHARED_REGISTRY);
		put_u32(WORKER_THREAD + NODE, SECOND_THREAD + NODE);
		put_u32(SECOND_THREAD + NODE + 4, WORKER_THREAD + NODE);
		put_u32(SECOND_THREAD + NODE, SHARED_REGISTRY);
	} else {
		put_u32(REGISTRY + 4, MAIN_THREAD + NODE);
		put_u32(MAIN_THREAD + NODE, REGISTRY);
		unsigned int head = SECOND_INSTANCE + 512;
		put_u32(head, WORKER_THREAD + NODE);
		put_u32(head + 4, SECOND_THREAD + NODE);
		put_u32(WORKER_THREAD + NODE + 4, head);
		put_u32(WORKER_THREAD + NODE, SECOND_THREAD + NODE);
		put_u32(SECOND_THREAD + NODE + 4, WORKER_THREAD + NODE);
		put_u32(SECOND_THREAD + NODE, head);
	}
}

static void packet(const char *text)
{
	CHECK(gdb_thread_packet(&connection, text, strlen(text)) == ERROR_OK);
}

static void test_smp(void)
{
	for (unsigned int port = 1; port <= 6; port++) {
		for (unsigned int flags = 0; flags < 4; flags++) {
			setup_smp(port, flags & 1, flags & 2);
			packet("Hg0"); /* Attachment precedes symbol negotiation. */
			CHECK(!strcmp(reply, "OK") && gdb_service.target == &target);
			CHECK(!rtos_needs_fake_step(&target, 0));
			/* Symbol negotiation and thread packets use the real RTOS core. */
			packet("qSymbol::");
			CHECK(!strcmp(reply, "qSymbol:63685f73797374656d"));
			packet("qSymbol:300:63685f73797374656d");
			CHECK(!strcmp(reply, "qSymbol:63685f6465627567"));
			packet("qSymbol:100:63685f6465627567");
			CHECK(!strcmp(reply, "OK"));
			CHECK(rt8_os->thread_count == 3 && rt8_os->current_thread == MAIN_THREAD);
			packet("qfThreadInfo");
			CHECK(strstr(reply, "0000000000000800") && strstr(reply, "0000000000001400"));
			packet("Hg1400");
			CHECK(!strcmp(reply, "OK") && gdb_service.target == &secondary);
			CHECK(rt8_os->current_threadid == SECOND_THREAD);
			CHECK(rtos_get_gdb_reg(&connection, ARMV7M_R4) == ERROR_OK);
			CHECK(!strcmp(reply, flags & 2 ? "b0000004" : "040000b0"));
			CHECK(rtos_get_gdb_reg(&connection, ARMV7M_MSP) == ERROR_OK);
			CHECK(!strcmp(reply, flags & 2 ? "00003200" : "00320000"));
			CHECK(rtos_get_gdb_reg_list(&connection) == ERROR_OK);
			CHECK(!strncmp(reply, flags & 2 ? "b0000000" : "000000b0", 8));
			uint8_t new_value[4] = { 1, 2, 3, 4 };
			CHECK(rtos_set_reg(&connection, ARMV7M_R4, new_value) == ERROR_OK);
			CHECK(writes == 1 && written_reg == &secondary_regs[ARMV7M_R4]);
			write_result = ERROR_TARGET_FAILURE;
			CHECK(rtos_set_reg(&connection, ARMV7M_R4, new_value) == ERROR_TARGET_FAILURE);
			write_result = ERROR_OK;
			CHECK(!rtos_needs_fake_step(&target, SECOND_THREAD));
			CHECK(!rtos_needs_fake_step(&secondary, MAIN_THREAD));
			CHECK(rtos_needs_fake_step(&target, WORKER_THREAD));
			CHECK(rtos_swbp_target(&secondary, 0x2000, 2, BKPT_SOFT) == &target);
			packet("Hg800");
			CHECK(gdb_service.target == &target);
			CHECK(rtos_get_gdb_reg(&connection, ARMV7M_R4) == ERROR_OK);
			CHECK(!strcmp(reply, flags & 2 ? "a0000004" : "040000a0"));
			CHECK(rtos_set_reg(&connection, ARMV7M_R4, new_value) == ERROR_OK);
			CHECK(writes == 3 && written_reg == &live_regs[ARMV7M_R4]);
			packet("Hgc00");
			CHECK(rtos_set_reg(&connection, ARMV7M_R4, new_value) == ERROR_FAIL);
			CHECK(writes == 3);
			CHECK(rtos_get_gdb_reg(&connection, ARMV7M_R4) == ERROR_OK);
			CHECK(!strcmp(reply, flags & 2 ? "10000004" : "04000010"));
			CHECK(get_register(ARMV7M_MSP, 32) == 0x3200);
			if (port == 4 || port == 6) {
				CHECK(get_register(ARMV7M_D0, 64) == UINT64_C(0x3f0000013f000000));
				secondary_fpccr = 1;
				secondary_fpcar = STACK + 32;
				unavailable(ARMV7M_D0);
				unavailable(ARMV7M_FPSCR);
				secondary_fpccr = 0;
				/* Lazy state on a different core must also be respected. */
				fpccr = 1;
				fpcar = STACK + 32;
				unavailable(ARMV7M_D0);
			}
			packet("Hgdead");
			CHECK(!strcmp(reply, "E01") && rt8_os->current_threadid == WORKER_THREAD);
			target.debug_reason = DBG_REASON_DBGRQ;
			secondary.debug_reason = DBG_REASON_BREAKPOINT;
			CHECK(rtos_update_threads(&target) == ERROR_OK);
			CHECK(rt8_os->current_thread == SECOND_THREAD);
			packet("qC");
			CHECK(!strcmp(reply, "QC0000000000001400"));
			struct target *cpu = NULL;
			CHECK(rt8_os->gdb_target_for_threadid(&connection, SECOND_THREAD, &cpu) == ERROR_OK);
			CHECK(cpu == &secondary);
			target.debug_reason = DBG_REASON_SINGLESTEP;
			CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
			CHECK(rt8_os->current_thread == MAIN_THREAD);
			/* Equal stop reasons prefer the selected live thread. */
			secondary.debug_reason = DBG_REASON_SINGLESTEP;
			packet("Hg1400");
			CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
			CHECK(rt8_os->current_thread == SECOND_THREAD);
			packet("Hg0");
			CHECK(rt8_os->current_threadid == SECOND_THREAD);
			/* A running peer must prevent even the initial memory read. */
			secondary.state = TARGET_RUNNING;
			unsigned int old_reads = reads;
			CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_TARGET_NOT_HALTED);
			CHECK(reads == old_reads && rt8_os->thread_count == 0);
			CHECK(rtos_needs_fake_step(&target, SECOND_THREAD));
			CHECK(rt8_os->gdb_target_for_threadid(&connection, SECOND_THREAD, &cpu) == ERROR_OK);
			CHECK(cpu == &secondary); /* Ctrl-C still has a usable target. */
			secondary.state = TARGET_HALTED;
			CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
			/* Reset drops the old selection and uses the boot placeholder. */
			memory[SYSTEM] = 0;
			CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
			CHECK(rt8_os->thread_count == 1 && rt8_os->current_thread == 1);
			packet("Hg1");
			CHECK(gdb_service.target == &target);
			CHECK(!rtos_needs_fake_step(&target, 1));
			CHECK(rtos_get_gdb_reg_list(&connection) == ERROR_OK);
			teardown();
			CHECK(!target.rtos && !secondary.rtos);
			cases++;
		}
	}
}

static void test_smp_faults(void)
{
	for (unsigned int fault = 0; fault < 13; fault++) {
		setup_smp(6, true, false);
		switch (fault) {
		case 0:
			secondary.coreid = target.coreid;
			break;
		case 1:
			secondary.coreid = 8;
			break;
		case 2:
			secondary.endianness = TARGET_BIG_ENDIAN;
			break;
		case 3:
			secondary.examined = false;
			break;
		case 4:
			secondary.state = TARGET_UNAVAILABLE;
			break;
		case 5:
			secondary_arm.arm.arch = ARM_ARCH_V7M;
			break;
		case 6:
			put_u32(SECOND_INSTANCE + 400, 3);
			break;
		case 7:
			put_u32(SECOND_INSTANCE + 12, MAIN_THREAD);
			break;
		case 8:
			put_u32(WORKER_THREAD + 300, 0x1800);
			break;
		case 9:
			put_u32(SYSTEM + 4 + 7 * 4, 0);
			break;
		case 10:
			put_u32(SECOND_INSTANCE + 12, 0);
			break;
		case 11:
			read_fault = SECOND_INSTANCE + 400;
			break;
		case 12:
			secondary_arm.fp_feature = 1;
			break;
		}
		CHECK(rt8_rtos.update_threads(rt8_os) != ERROR_OK);
		CHECK(!rt8_os->thread_count);
		struct target *cpu = NULL;
		CHECK(rt8_os->gdb_target_for_threadid(&connection, SECOND_THREAD, &cpu) != ERROR_OK);
		CHECK(cpu); /* GDB stop-reply callers may ignore the error. */
		teardown();
		cases++;
	}
	/* An unpublished secondary instance is valid while its core is halted. */
	setup_smp(3, true, false);
	put_u32(SYSTEM + 4 + 7 * 4, 0);
	put_u32(MAIN_THREAD + NODE, SHARED_REGISTRY);
	put_u32(SHARED_REGISTRY + 4, MAIN_THREAD + NODE);
	CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
	CHECK(rt8_os->thread_count == 1 && rt8_os->current_thread == MAIN_THREAD);
	/* Published before current is set is also a valid startup state. */
	put_u32(SYSTEM + 4 + 7 * 4, SECOND_INSTANCE);
	put_u32(SECOND_INSTANCE + 12, 0);
	CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
	CHECK(rt8_os->thread_count == 1);
	teardown();
	cases++;
	/* Destroying/reconfiguring any alias frees a shared RTOS only once. */
	setup_smp(3, true, false);
	CHECK(rt8_rtos.update_threads(rt8_os) == ERROR_OK);
	/* Aliases must also be detached after the SMP group was dismantled. */
	target.smp = false;
	secondary.smp = false;
	rtos_destroy(&secondary);
	CHECK(!target.rtos && !secondary.rtos);
	teardown();
	cases++;
}

int main(void)
{
	test_contexts();
	test_registry();
	test_bad_layouts();
	test_faults();
	test_smp();
	test_smp_faults();
	printf("PASS: %u RT8 context, registry and rejection cases\n", cases);
	return EXIT_SUCCESS;
}
