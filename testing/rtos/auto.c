// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos/rtos.h"
#include "helper/binarybuffer.h"
#include "helper/log.h"
#include "server/gdb_server.h"

/* Exercise the real qSymbol state machine and driver order with controlled
 * probe results. Individual drivers' memory decoders are tested separately.
 */
#define CHECK(expr) do { \
	if (!(expr)) { \
		fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
		exit(EXIT_FAILURE); \
	} \
} while (0)

static struct target target;
static struct gdb_service gdb_service = { .target = &target };
static struct service service = { .priv = &gdb_service };
static struct connection connection = { .service = &service };
static char reply[GDB_BUFFER_SIZE + 1];
static const struct rtos_type *accepted;
static const struct rtos_type *probed[16];
static unsigned int probe_count;
static unsigned int created;
static unsigned int updated;
static int register_result;
int debug_level = LOG_LVL_ERROR;

static bool probe(struct target *t)
{
	CHECK(probe_count < ARRAY_SIZE(probed));
	probed[probe_count++] = t->rtos->type;
	return t->rtos->type == accepted;
}

static int create(struct target *t)
{
	CHECK(t->rtos->type == accepted);
	created++;
	return ERROR_OK;
}

static int update(struct rtos *rtos)
{
	CHECK(rtos->type == accepted);
	updated++;
	return ERROR_OK;
}

static int get_registers(struct rtos *rtos, int64_t thread,
		struct rtos_reg **registers, int *count)
{
	CHECK(thread == 2);
	if (register_result != ERROR_OK)
		return register_result;
	*registers = calloc(1, sizeof(**registers));
	CHECK(*registers);
	*count = 1;
	(*registers)->size = 32;
	(*registers)->value[0] = 0x12;
	return ERROR_OK;
}

static int get_register(struct rtos *rtos, int64_t thread,
		uint32_t number, uint32_t *size, uint8_t **value)
{
	CHECK(thread == 2 && number == 17);
	*size = 32;
	*value = NULL;
	return ERROR_OK;
}

static int chibios_symbols(struct symbol_table_elem **symbols)
{
	const struct symbol_table_elem list[] = {
		{ "ch_system", 0, false }, { "ch_debug", 0, false }, { NULL, 0, false },
	};

	*symbols = malloc(sizeof(list));
	CHECK(*symbols);
	memcpy(*symbols, list, sizeof(list));
	return ERROR_OK;
}

static int legacy_symbols(struct symbol_table_elem **symbols)
{
	const struct symbol_table_elem list[] = {
		{ "rlist", 0, true }, { "ch", 0, true },
		{ "ch_debug", 0, false }, { NULL, 0, false },
	};

	*symbols = malloc(sizeof(list));
	CHECK(*symbols);
	memcpy(*symbols, list, sizeof(list));
	return ERROR_OK;
}

static int empty_symbols(struct symbol_table_elem **symbols)
{
	*symbols = calloc(1, sizeof(**symbols));
	CHECK(*symbols);
	return ERROR_OK;
}

const struct rtos_type rt8_rtos = {
	.name = "chibios-rt8", .detect_rtos = probe, .create = create,
	.update_threads = update, .get_symbol_list_to_lookup = chibios_symbols,
	.get_thread_reg_list = get_registers,
	.get_thread_reg_value = get_register,
};
const struct rtos_type rt7_rtos = {
	.name = "chibios-rt7", .detect_rtos = probe, .create = create,
	.update_threads = update, .get_symbol_list_to_lookup = chibios_symbols,
};
const struct rtos_type chibios_rtos = {
	.name = "chibios", .detect_rtos = probe, .create = create,
	.update_threads = update, .get_symbol_list_to_lookup = legacy_symbols,
};

#define EMPTY_RTOS(symbol) const struct rtos_type symbol = { \
	.name = #symbol, .detect_rtos = probe, .create = create, \
	.update_threads = update, .get_symbol_list_to_lookup = empty_symbols, \
}

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

void log_printf_lf(enum log_levels level, const char *file, unsigned int line,
		const char *function, const char *format, ...)
{
}

void command_print(struct command_invocation *cmd, const char *format, ...)
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

int target_read_buffer(struct target *t, target_addr_t address, uint32_t size, uint8_t *buffer)
{
	CHECK(false); /* The qSymbol state machine does not read target memory. */
	return ERROR_FAIL;
}

int gdb_put_packet(struct connection *conn, const char *buffer, int len)
{
	CHECK(len >= 0 && (size_t)len < sizeof(reply));
	memcpy(reply, buffer, len);
	reply[len] = '\0';
	return ERROR_OK;
}

static void expect_symbol(const char *name)
{
	char expected[128] = "qSymbol:";

	hexify(expected + 8, (const uint8_t *)name, strlen(name), sizeof(expected) - 8);
	CHECK(!strcmp(reply, expected));
}

static void answer(const char *name, bool found)
{
	char packet[128];
	int prefix = snprintf(packet, sizeof(packet), "qSymbol:%s:", found ? "100" : "");

	hexify(packet + prefix, (const uint8_t *)name, strlen(name), sizeof(packet) - prefix);
	CHECK(rtos_thread_packet(&connection, packet, strlen(packet)) == ERROR_OK);
}

static void start(const struct rtos_type *accept, const char *mode)
{
	accepted = accept;
	probe_count = 0;
	created = 0;
	updated = 0;
	CHECK(rtos_create(NULL, &target, mode) == ERROR_OK);
	CHECK(rtos_thread_packet(&connection, "qSymbol::", 9) == ERROR_OK);
	expect_symbol("ch_system");
}

static void finish_legacy(void)
{
	expect_symbol("rlist");
	answer("rlist", false);
	expect_symbol("rlist.lto_priv.0");
	answer("rlist.lto_priv.0", false);
	expect_symbol("ch");
	answer("ch", true);
	expect_symbol("ch_debug");
	answer("ch_debug", true);
	CHECK(!strcmp(reply, "OK"));
}

int main(void)
{
	/* RT8 must be first, including when its symbols have LTO suffixes. */
	start(&rt8_rtos, "auto");
	answer("ch_system", true);
	expect_symbol("ch_debug");
	answer("ch_debug", false);
	expect_symbol("ch_debug.lto_priv.0");
	answer("ch_debug.lto_priv.0", true);
	CHECK(!strcmp(reply, "OK") && !target.rtos_auto_detect);
	CHECK(probe_count == 1 && probed[0] == &rt8_rtos);
	CHECK(created == 1 && updated == 1);

	/* RT7 follows RT8 when the kernel version is incompatible. */
	start(&rt7_rtos, "auto");
	answer("ch_system", true);
	answer("ch_debug", true);
	expect_symbol("ch_system");
	answer("ch_system", true);
	answer("ch_debug", true);
	CHECK(!strcmp(reply, "OK") && !target.rtos_auto_detect);
	CHECK(probe_count == 2 && probed[0] == &rt8_rtos && probed[1] == &rt7_rtos);
	CHECK(created == 1 && updated == 1);

	/* Missing RT8/RT7 symbols must still allow legacy detection. */
	start(&chibios_rtos, "auto");
	answer("ch_system", false);
	expect_symbol("ch_system.lto_priv.0");
	answer("ch_system.lto_priv.0", false);
	expect_symbol("ch_system");
	answer("ch_system", false);
	expect_symbol("ch_system.lto_priv.0");
	answer("ch_system.lto_priv.0", false);
	finish_legacy();
	CHECK(probe_count == 1 && probed[0] == &chibios_rtos);
	CHECK(created == 1 && updated == 1);

	/* Found symbols with an incompatible kernel must also fall through. */
	start(&chibios_rtos, "auto");
	answer("ch_system", true);
	answer("ch_debug", true);
	answer("ch_system", true);
	answer("ch_debug", true);
	finish_legacy();
	CHECK(probe_count == 3 && probed[0] == &rt8_rtos &&
		probed[1] == &rt7_rtos && probed[2] == &chibios_rtos);
	CHECK(target.rtos->type == &chibios_rtos && !target.rtos_auto_detect);
	CHECK(created == 1 && updated == 1);

	/* Continue past multiple rejections, including symbol-less drivers. */
	start(&ecos_rtos, "auto");
	answer("ch_system", true);
	answer("ch_debug", true);
	answer("ch_system", true);
	answer("ch_debug", true);
	finish_legacy();
	CHECK(probe_count == 5 && target.rtos->type == &ecos_rtos);
	CHECK(!target.rtos_auto_detect && created == 1 && updated == 1);

	/* Exhaustion ends the exchange without creating a rejected driver. */
	start(NULL, "auto");
	answer("ch_system", true);
	answer("ch_debug", true);
	answer("ch_system", true);
	answer("ch_debug", true);
	finish_legacy();
	CHECK(target.rtos->type == &hwthread_rtos && target.rtos_auto_detect);
	CHECK(!created && !updated);

	/* Explicit selection does not invoke the scan. */
	start(&rt7_rtos, "chibios-rt7");
	answer("ch_system", true);
	answer("ch_debug", true);
	CHECK(!strcmp(reply, "OK") && !probe_count);
	CHECK(created == 1 && updated == 1);
	start(&rt8_rtos, "chibios-rt8");
	answer("ch_system", true);
	answer("ch_debug", true);
	CHECK(!strcmp(reply, "OK") && !probe_count);
	CHECK(created == 1 && updated == 1);
	rtos_destroy(&target);

	/* Only an unhandled read may fall back to the CPU's live registers. */
	start(&rt8_rtos, "chibios-rt8");
	target.rtos->current_thread = 1;
	target.rtos->current_threadid = 1;
	CHECK(rtos_get_gdb_reg_list(&connection) == ERROR_NOT_IMPLEMENTED);
	target.rtos->current_threadid = 2;
	register_result = ERROR_TARGET_FAILURE;
	CHECK(rtos_get_gdb_reg_list(&connection) == ERROR_TARGET_FAILURE);
	register_result = ERROR_OK;
	CHECK(rtos_get_gdb_reg_list(&connection) == ERROR_OK);
	CHECK(!strcmp(reply, "12000000"));
	CHECK(rtos_get_gdb_reg(&connection, 17) == ERROR_OK);
	CHECK(!strcmp(reply, "xxxxxxxx"));
	rtos_destroy(&target);
	return EXIT_SUCCESS;
}
