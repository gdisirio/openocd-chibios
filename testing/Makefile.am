# SPDX-License-Identifier: GPL-2.0-or-later

SUBDIRS = tcl_commands
DIST_SUBDIRS = tcl_commands

check_PROGRAMS = rt7-test rt8-test rtos-auto-test
TESTS = $(check_PROGRAMS)
rt7_test_SOURCES = rtos/rt7.c $(top_srcdir)/src/rtos/rt7.c \
	$(top_srcdir)/src/rtos/rtos_chibios_stackings_alt.c
rt7_test_CPPFLAGS = -I$(top_srcdir)/src -I$(top_builddir)
rt7_test_CFLAGS = $(GCC_WARNINGS)
if INTERNAL_JIMTCL
rt7_test_CPPFLAGS += -I$(top_srcdir)/jimtcl -I$(top_builddir)/jimtcl
else
rt7_test_CPPFLAGS += $(JIMTCL_CFLAGS)
endif

rt8_test_SOURCES = rtos/rt8.c $(top_srcdir)/src/rtos/rt8.c \
	$(top_srcdir)/src/rtos/rtos.c $(top_srcdir)/src/helper/binarybuffer.c
rt8_test_CPPFLAGS = $(rt7_test_CPPFLAGS)
rt8_test_CFLAGS = $(GCC_WARNINGS)

rtos_auto_test_SOURCES = rtos/auto.c $(top_srcdir)/src/rtos/rtos.c \
	$(top_srcdir)/src/helper/binarybuffer.c
rtos_auto_test_CPPFLAGS = $(rt7_test_CPPFLAGS)
rtos_auto_test_CFLAGS = $(GCC_WARNINGS)
