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

#pragma once

#include <stdint.h>

/*
 * Diagnostics that survive a reset (CONFIG_DIAG): why the device last reset,
 * where the last fault happened, and counters of frames dropped because the
 * host did not keep up. Read with the vendor request GS_USB_BREQ_X_GET_DIAG.
 *
 * The record lives in .noinit RAM, which the start-up code does not clear;
 * a power-on reset (or a changed layout) starts it afresh.
 */

#define GS_USB_BREQ_X_GET_DIAG		 0xC0 /* device -> host: struct diag_report */
#define GS_USB_BREQ_X_GET_CLEAR_DIAG 0xC1 /* as GET_DIAG, then forget the last fault */
#define GS_USB_BREQ_X_DEBUG_FAULT	 0xC2 /* CONFIG_DEBUG_FAULT_REQUEST: wValue 0 assert, 1 hang, 2 HardFault */

#define DIAG_MAGIC	 0x47414944u /* "DIAG" */
#define DIAG_VERSION 1u

enum diag_fault_kind {
	DIAG_FAULT_NONE = 0,
	DIAG_FAULT_HARDFAULT = 1,
	DIAG_FAULT_ASSERT = 2,
	DIAG_FAULT_NMI = 3,
};

/* All fields little-endian uint32_t; the host reads them by index. */
struct diag_report {
	uint32_t magic;
	uint32_t version;
	uint32_t boot_count;          /* boots since power-on (1 = first) */
	uint32_t reset_flags;         /* RCC_CSR[31:24] of this boot */
	uint32_t watchdog_resets;     /* since power-on */
	uint32_t fault_resets;        /* since power-on */
	uint32_t last_fault_kind;     /* enum diag_fault_kind */
	uint32_t last_fault_pc;
	uint32_t last_fault_lr;
	uint32_t last_fault_xpsr;
	uint32_t last_fault_uptime_ms;
	uint32_t last_fault_boot;     /* boot_count when it happened */
	uint32_t uptime_ms;           /* now */
	uint32_t from_host_dropped;   /* this boot */
	uint32_t can_rx_dropped;      /* this boot */
	uint32_t pool_free;           /* now */
	uint32_t pool_size;
	uint32_t bus_off_count;       /* this boot */
};

/* Call once early in main(), after HAL_Init(). */
void diag_boot(void);

/* Records a fault before the reset. */
void diag_record_fault(uint32_t kind, uint32_t pc, uint32_t lr, uint32_t xpsr);

void diag_count_bus_off(void);

/* Fills the report (persistent part plus live counters) for a GET_DIAG. */
void diag_fill_report(struct diag_report *report, uint32_t from_host_dropped,
					  uint32_t can_rx_dropped, uint32_t pool_free, uint32_t pool_size);

void diag_clear_fault(void);
