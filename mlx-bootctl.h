/*
 * Copyright (c) 2017, Mellanox Technologies, Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of Mellanox nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BLUEFIELD_SVC_H__
#define __BLUEFIELD_SVC_H__

/* BlueField-specific SMC function IDs */

/*
 * Request that the on-chip watchdog be enabled, or disabled, after
 * the next chip soft reset. This call does not affect the current
 * status of the on-chip watchdog. If non-zero, the argument
 * specifies the watchdog interval in seconds. If zero, the watchdog
 * will not be enabled after the next soft reset. Non-zero errors are
 * returned as documented below.
 */
#define MLNX_SET_POST_RESET_WDOG	0x82000000

/*
 * Query the status which has been requested for the on-chip watchdog
 * after the next chip soft reset. Returns the interval as set by
 * MLNX_SET_POST_RESET_WDOG.
 */
#define MLNX_GET_POST_RESET_WDOG	0x82000001

/*
 * Request that a specific boot action be taken at the next soft
 * reset. By default, the boot action is set by external chip pins,
 * which are sampled on hard reset. Note that the boot action
 * requested by this call will persist on subsequent resets unless
 * this service, or the MLNX_SET_SECOND_RESET_ACTION service, is
 * invoked. See below for the available MLNX_BOOT_xxx parameter
 * values. Non-zero errors are returned as documented below.
 */
#define MLNX_SET_RESET_ACTION		0x82000002

/*
 * Return the specific boot action which will be taken at the next
 * soft reset. Returns the reset action (see below for the parameter
 * values for MLNX_SET_RESET_ACTION).
 */
#define MLNX_GET_RESET_ACTION		0x82000003

/*
 * Request that a specific boot action be taken at the soft reset
 * after the next soft reset. For a specified valid boot mode, the
 * effect of this call is identical to that of invoking
 * MLNX_SET_RESET_ACTION after the next chip soft reset; in
 * particular, after that reset, the action for the now next reset can
 * be queried with MLNX_GET_RESET_ACTION and modified with
 * MLNX_SET_RESET_ACTION. You may also specify the parameter as
 * MLNX_BOOT_NONE, which is equivalent to specifying that no call to
 * MLNX_SET_RESET_ACTION be taken after the next chip soft reset.
 * This call does not affect the action to be taken at the next soft
 * reset. Non-zero errors are returned as documented below.
 */
#define MLNX_SET_SECOND_RESET_ACTION	0x82000004

/*
 * Return the specific boot action which will be taken at the soft
 * reset after the next soft reset; this will be one of the valid
 * actions for MLNX_SET_SECOND_RESET_ACTION.
 */
#define MLNX_GET_SECOND_RESET_ACTION	0x82000005

/*
 * Return the fuse status of the current chip. The caller should specify
 * with the second argument if the state of the lifecycle fuses or the
 * version of secure boot fuse keys left should be returned.
 */
#define MLNX_GET_TBB_FUSE_STATUS	0x82000006

/*
 * Reset eMMC by programming the RST_N register.
 */
#define MLNX_SET_EMMC_RST_N		0x82000007

#define MLNX_GET_DIMM_INFO		0x82000008

/*
 * Read from or write to register. Used for accessing registers in secure blocks
 * from the non-secure world, without changing the security level of the whole
 * block. Only registers defined by a verification table may be accessed.
 *
 * MLNX_READ_REG_32/64:
 * Parameters:
 * x1 Verification table number, see MLNX_REGS_XXXX for valid values
 * x2 Address to access
 *
 * Returns:
 * r0 Status, one of:
 *   - 0 if read succeeded
 *   - SMCCC_OUT_OF_RANGE if verification table number is outside valid range
 *   - SMCCC_ACCESS_VIOLATION if the address is not in the selected
 *     verification table
 * r1 The value read from the register. If r0 is nonzero, r1 will be 0.
 *
 * MLNX_WRITE_REG_32/64:
 * Parameters:
 * x1 Verification table number
 * x2 Value to write
 * x3 Address to access
 *
 * Returns:
 * r0 Status, one of:
 *   - 0 if read succeeded
 *   - SMCCC_OUT_OF_RANGE if verification table number is outside valid range
 *   - SMCCC_ACCESS_VIOLATION if the address is not in the selected
 *     verification table
 */
#define MLNX_WRITE_REG_32	0x82000009
#define MLNX_READ_REG_32	0x8200000A
#define MLNX_WRITE_REG_64	0x8200000B
#define MLNX_READ_REG_64	0x8200000C

/* SMC function IDs for SiP Service queries */
#define MLNX_SIP_SVC_CALL_COUNT		0x8200ff00
#define MLNX_SIP_SVC_UID		0x8200ff01
#define MLNX_SIP_SVC_VERSION		0x8200ff03

/* ARM Standard Service Calls version numbers */
#define MLNX_SVC_VERSION_MAJOR		0x0
#define MLNX_SVC_VERSION_MINOR		0x3

/* Number of svc calls defined. */
#define MLNX_NUM_SVC_CALLS 16

/* Valid reset actions for MLNX_SET_RESET_ACTION. */
#define MLNX_BOOT_EXTERNAL	0 /* Do not boot from eMMC */
#define MLNX_BOOT_EMMC		1 /* Boot from primary eMMC boot partition */
#define MLNX_BOOT_SWAP_EMMC	2 /* Swap eMMC boot partitions and reboot */
#define MLNX_BOOT_EMMC_LEGACY	3 /* Boot from primary eMMC in legacy mode */

/* Valid arguments for requesting the fuse status. */
#define MLNX_FUSE_STATUS_LIFECYCLE	0 /* Return the lifecycle status. */
#define MLNX_FUSE_STATUS_KEYS		1 /* Return secure boot key status */

/* Additional parameter value to disable the MLNX_SET_SECOND_RESET_ACTION. */
#define MLNX_BOOT_NONE		0x7fffffff /* Don't change next boot action */

/* Parameter value for reg_read/write32 and reg_read/write64. */
enum {
	MLNX_REGS_PERF = 0,
	MLNX_REGS_L3C
};

/* Error values (non-zero). */
#define SMCCC_INVALID_PARAMETERS	-2
#define SMCCC_OUT_OF_RANGE		-3
#define SMCCC_ACCESS_VIOLATION		-4

#endif /* __BLUEFIELD_SVC_H__ */
