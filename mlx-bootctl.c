// SPDX-License-Identifier: GPL-2.0
/*
 *  Mellanox boot control driver
 *  This driver provides a sysfs interface for systems management
 *  software to manage reset-time actions.
 *
 *  Copyright (C) 2017 Mellanox Technologies.  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License v2.0 as published by
 *  the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/acpi.h>
#include <linux/arm-smccc.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "mlx-bootctl.h"

#define DRIVER_NAME		"mlx-bootctl"
#define DRIVER_VERSION		"1.2"
#define DRIVER_DESCRIPTION	"Mellanox boot control driver"

#define SB_MODE_SECURE_MASK	0x03
#define SB_MODE_TEST_MASK	0x0c

#define SB_KEY_NUM		4

struct boot_name {
	int value;
	const char name[12];
};

static struct boot_name boot_names[] = {
	{ MLNX_BOOT_EXTERNAL,		"external"	},
	{ MLNX_BOOT_EMMC,		"emmc"		},
	{ MLNX_BOOT_SWAP_EMMC,		"swap_emmc"	},
	{ MLNX_BOOT_EMMC_LEGACY,	"emmc_legacy"	},
	{ MLNX_BOOT_NONE,		"none"		},
	{ -1,				""		}
};

static char lifecycle_states[][16] = {
	[0] = "Production",
	[1] = "GA Secured",
	[2] = "GA Non-Secured",
	[3] = "RMA",
};

/* The SMC calls in question are atomic, so we don't have to lock here. */
static int smc_call1(unsigned int smc_op, int smc_arg)
{
	struct arm_smccc_res res;

	arm_smccc_smc(smc_op, smc_arg, 0, 0, 0, 0, 0, 0, &res);

	return res.a0;
}

/* Syntactic sugar to avoid having to specify an unused argument. */
#define smc_call0(smc_op) smc_call1(smc_op, 0)

static int reset_action_to_val(const char *action, size_t len)
{
	struct boot_name *bn;

	/* Accept string either with or without a newline terminator */
	if (action[len-1] == '\n')
		--len;

	for (bn = boot_names; bn->value >= 0; ++bn)
		if (strncmp(bn->name, action, len) == 0)
			break;

	return bn->value;
}

static const char *reset_action_to_string(int action)
{
	struct boot_name *bn;

	for (bn = boot_names; bn->value >= 0; ++bn)
		if (bn->value == action)
			break;

	return bn->name;
}

static ssize_t post_reset_wdog_show(struct device_driver *drv,
				    char *buf)
{
	return sprintf(buf, "%d\n", smc_call0(MLNX_GET_POST_RESET_WDOG));
}

static ssize_t post_reset_wdog_store(struct device_driver *drv,
				     const char *buf, size_t count)
{
	int err;
	unsigned long watchdog;

	err = kstrtoul(buf, 10, &watchdog);
	if (err)
		return err;

	if (smc_call1(MLNX_SET_POST_RESET_WDOG, watchdog) < 0)
		return -EINVAL;

	return count;
}

static ssize_t reset_action_show(struct device_driver *drv,
				 char *buf)
{
	return sprintf(buf, "%s\n", reset_action_to_string(
			       smc_call0(MLNX_GET_RESET_ACTION)));
}

static ssize_t reset_action_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	int action = reset_action_to_val(buf, count);

	if (action < 0 || action == MLNX_BOOT_NONE)
		return -EINVAL;

	if (smc_call1(MLNX_SET_RESET_ACTION, action) < 0)
		return -EINVAL;

	return count;
}

static ssize_t second_reset_action_show(struct device_driver *drv,
					char *buf)
{
	return sprintf(buf, "%s\n", reset_action_to_string(
			       smc_call0(MLNX_GET_SECOND_RESET_ACTION)));
}

static ssize_t second_reset_action_store(struct device_driver *drv,
					 const char *buf, size_t count)
{
	int action = reset_action_to_val(buf, count);

	if (action < 0)
		return -EINVAL;

	if (smc_call1(MLNX_SET_SECOND_RESET_ACTION, action) < 0)
		return -EINVAL;

	return count;
}

static ssize_t lifecycle_state_show(struct device_driver *drv,
				    char *buf)
{
	int lc_state = smc_call1(MLNX_GET_TBB_FUSE_STATUS,
				 MLNX_FUSE_STATUS_LIFECYCLE);

	if (lc_state < 0)
		return -EINVAL;

	lc_state &= (SB_MODE_TEST_MASK | SB_MODE_SECURE_MASK);
	/*
	 * If the test bits are set, we specify that the current state may be
	 * due to using the test bits.
	 */
	if ((lc_state & SB_MODE_TEST_MASK) != 0) {

		lc_state &= SB_MODE_SECURE_MASK;

		return sprintf(buf, "%s(test)\n", lifecycle_states[lc_state]);
	}

	return sprintf(buf, "%s\n", lifecycle_states[lc_state]);
}

static ssize_t secure_boot_fuse_state_show(struct device_driver *drv,
					   char *buf)
{
	int key;
	int buf_len = 0;
	int upper_key_used = 0;
	int sb_key_state = smc_call1(MLNX_GET_TBB_FUSE_STATUS,
				     MLNX_FUSE_STATUS_KEYS);

	if (sb_key_state < 0)
		return -EINVAL;

	for (key = SB_KEY_NUM - 1; key >= 0; key--) {
		int burnt = ((sb_key_state & (1 << key)) != 0);
		int valid = ((sb_key_state & (1 << (key + SB_KEY_NUM))) != 0);

		buf_len += sprintf(buf + buf_len, "Ver%d:", key);
		if (upper_key_used) {
			if (burnt) {
				if (valid)
					buf_len += sprintf(buf + buf_len,
							  "Used");
				else
					buf_len += sprintf(buf + buf_len,
							  "Wasted");
			} else {
				if (valid)
					buf_len += sprintf(buf + buf_len,
							  "Invalid");
				else
					buf_len += sprintf(buf + buf_len,
							  "Skipped");
			}
		} else {
			if (burnt) {
				if (valid) {
					upper_key_used = 1;
					buf_len += sprintf(buf + buf_len,
							  "In use");
				} else
					buf_len += sprintf(buf + buf_len,
							  "Burn incomplete");
			} else {
				if (valid)
					buf_len += sprintf(buf + buf_len,
							  "Invalid");
				else
					buf_len += sprintf(buf + buf_len,
							  "Free");
			}
		}
		buf_len += sprintf(buf + buf_len, "\n");
	}

	return buf_len;
}

#define MBC_DRV_ATTR(_name) DRIVER_ATTR_RW(_name)

static MBC_DRV_ATTR(post_reset_wdog);
static MBC_DRV_ATTR(reset_action);
static MBC_DRV_ATTR(second_reset_action);
static DRIVER_ATTR_RO(lifecycle_state);
static DRIVER_ATTR_RO(secure_boot_fuse_state);

static struct attribute *mbc_dev_attrs[] = {
	&driver_attr_post_reset_wdog.attr,
	&driver_attr_reset_action.attr,
	&driver_attr_second_reset_action.attr,
	&driver_attr_lifecycle_state.attr,
	&driver_attr_secure_boot_fuse_state.attr,
	NULL
};

static struct attribute_group mbc_attr_group = {
	.attrs = mbc_dev_attrs
};

static const struct attribute_group *mbc_attr_groups[] = {
	&mbc_attr_group,
	NULL
};

static const struct of_device_id mbc_dt_ids[] = {
	{.compatible = "mellanox,bootctl"},
	{},
};

MODULE_DEVICE_TABLE(of, mbc_dt_ids);

static const struct acpi_device_id mbc_acpi_ids[] = {
	{"MLNXBF04", 0},
	{},
};

MODULE_DEVICE_TABLE(acpi, mbc_acpi_ids);

static int mbc_probe(struct platform_device *pdev)
{
	struct arm_smccc_res res;

	/*
	 * Ensure we have the UUID we expect for this service.
	 * Note that the functionality we want is present in the first
	 * released version of this service, so we don't check the version.
	 */
	arm_smccc_smc(MLNX_SIP_SVC_UID, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != 0x89c036b4 || res.a1 != 0x11e6e7d7 ||
	    res.a2 != 0x1a009787 || res.a3 != 0xc4bf00ca)
		return -ENODEV;

	/*
	 * When watchdog is used, it sets the boot mode to MLNX_BOOT_SWAP_EMMC
	 * in case of boot failures. However it doesn't clear the state if there
	 * is no failure. Restore the default boot mode here to avoid any
	 * unnecessary boot partition swapping.
	 */
	if (smc_call1(MLNX_SET_RESET_ACTION, MLNX_BOOT_EMMC) < 0)
		pr_err("Unable to reset the EMMC boot mode\n");

	pr_info("%s (version %s)\n", DRIVER_DESCRIPTION, DRIVER_VERSION);

	return 0;
}

static int mbc_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mbc_driver = {
	.probe = mbc_probe,
	.remove = mbc_remove,
	.driver = {
		.name = DRIVER_NAME,
		.groups = mbc_attr_groups,
		.of_match_table = mbc_dt_ids,
		.acpi_match_table = ACPI_PTR(mbc_acpi_ids),
	}
};

module_platform_driver(mbc_driver);

MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR("Mellanox Technologies");
MODULE_LICENSE("GPL");
