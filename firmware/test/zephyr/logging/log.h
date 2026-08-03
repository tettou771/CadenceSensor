/*
 * Host stub for Zephyr's logging API. Log lines go to stderr so they do not
 * mix with the test's own report on stdout.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HOSTTEST_ZEPHYR_LOG_H
#define HOSTTEST_ZEPHYR_LOG_H

#include <stdio.h>

#define LOG_LEVEL_ERR 1
#define LOG_LEVEL_WRN 2
#define LOG_LEVEL_INF 3
#define LOG_LEVEL_DBG 4

#define LOG_MODULE_REGISTER(...) struct log_stub_unused_##__LINE__ { int x; }

extern int host_log_quiet;

#define LOG_AT(pfx, fmt, ...)                                                 \
	do {                                                                  \
		if (!host_log_quiet) {                                        \
			fprintf(stderr, pfx fmt "\n", ##__VA_ARGS__);          \
		}                                                             \
	} while (0)

#define LOG_ERR(fmt, ...) LOG_AT("[err] ", fmt, ##__VA_ARGS__)
#define LOG_WRN(fmt, ...) LOG_AT("[wrn] ", fmt, ##__VA_ARGS__)
#define LOG_INF(fmt, ...) LOG_AT("[inf] ", fmt, ##__VA_ARGS__)
#define LOG_DBG(fmt, ...) LOG_AT("[dbg] ", fmt, ##__VA_ARGS__)

#endif /* HOSTTEST_ZEPHYR_LOG_H */
