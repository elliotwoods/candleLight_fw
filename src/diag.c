/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Elliot Woods
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdbool.h>
#include <string.h>

#include "compiler.h"
#include "config.h"
#include "diag.h"
#include "hal_include.h"

struct diag_persistent {
	uint32_t magic;
	uint32_t version;
	uint32_t boot_count;
	uint32_t watchdog_resets;
	uint32_t fault_resets;
	uint32_t last_fault_kind;
	uint32_t last_fault_pc;
	uint32_t last_fault_lr;
	uint32_t last_fault_xpsr;
	uint32_t last_fault_uptime_ms;
	uint32_t last_fault_boot;
	uint32_t check; /* ~magic ^ boot_count: catches random RAM after power-on */
};

static struct diag_persistent persistent __attribute__((section(".noinit")));
static uint32_t reset_flags;
static uint32_t bus_off_count;

static uint32_t diag_check(const struct diag_persistent *p)
{
	return ~p->magic ^ p->boot_count ^ p->fault_resets ^ p->watchdog_resets;
}

void diag_boot(void)
{
	if (!IS_ENABLED(CONFIG_DIAG))
		return;

	reset_flags = RCC->CSR >> 24;
	RCC->CSR |= RCC_CSR_RMVF;

	const bool power_on = reset_flags & (RCC_CSR_PORRSTF >> 24);
	if (power_on || persistent.magic != DIAG_MAGIC ||
		persistent.version != DIAG_VERSION || persistent.check != diag_check(&persistent)) {
		memset(&persistent, 0, sizeof(persistent));
		persistent.magic = DIAG_MAGIC;
		persistent.version = DIAG_VERSION;
	}

	persistent.boot_count++;
	if (reset_flags & (RCC_CSR_IWDGRSTF >> 24))
		persistent.watchdog_resets++;
	persistent.check = diag_check(&persistent);
}

void diag_record_fault(uint32_t kind, uint32_t pc, uint32_t lr, uint32_t xpsr)
{
	if (!IS_ENABLED(CONFIG_DIAG))
		return;

	persistent.fault_resets++;
	persistent.last_fault_kind = kind;
	persistent.last_fault_pc = pc;
	persistent.last_fault_lr = lr;
	persistent.last_fault_xpsr = xpsr;
	persistent.last_fault_uptime_ms = HAL_GetTick();
	persistent.last_fault_boot = persistent.boot_count;
	persistent.check = diag_check(&persistent);
}

void diag_count_bus_off(void)
{
	bus_off_count++;
}

void diag_fill_report(struct diag_report *r, uint32_t from_host_dropped,
					  uint32_t can_rx_dropped, uint32_t pool_free, uint32_t pool_size)
{
	*r = (struct diag_report){
		.magic = DIAG_MAGIC,
		.version = DIAG_VERSION,
		.boot_count = persistent.boot_count,
		.reset_flags = reset_flags,
		.watchdog_resets = persistent.watchdog_resets,
		.fault_resets = persistent.fault_resets,
		.last_fault_kind = persistent.last_fault_kind,
		.last_fault_pc = persistent.last_fault_pc,
		.last_fault_lr = persistent.last_fault_lr,
		.last_fault_xpsr = persistent.last_fault_xpsr,
		.last_fault_uptime_ms = persistent.last_fault_uptime_ms,
		.last_fault_boot = persistent.last_fault_boot,
		.uptime_ms = HAL_GetTick(),
		.from_host_dropped = from_host_dropped,
		.can_rx_dropped = can_rx_dropped,
		.pool_free = pool_free,
		.pool_size = pool_size,
		.bus_off_count = bus_off_count,
	};
}

void diag_clear_fault(void)
{
	persistent.last_fault_kind = DIAG_FAULT_NONE;
	persistent.last_fault_pc = 0;
	persistent.last_fault_lr = 0;
	persistent.last_fault_xpsr = 0;
	persistent.last_fault_uptime_ms = 0;
	persistent.last_fault_boot = 0;
	persistent.check = diag_check(&persistent);
}
