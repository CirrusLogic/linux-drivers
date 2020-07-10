// SPDX-License-Identifier: GPL-2.0
/*
 * Allowlist of PCI drivers that are allowed to bind to external devices
 */

#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/pci.h>
#include "pci.h"

/*
 * Parameter to disable allowlist (thus allow all drivers to connect
 * to any external PCI devices).
 */
static bool trust_external_pci_devices;
core_param(trust_external_pci_devices, trust_external_pci_devices, bool, 0444);

/* Driver allowlist */
struct allowlist_entry {
	const char *drvr_name;
	struct list_head node;
};

static LIST_HEAD(allowlist);
static DECLARE_RWSEM(allowlist_sem);

#define TRUNCATED	"...<truncated>\n"

static ssize_t drivers_allowlist_show(struct bus_type *bus, char *buf)
{
	size_t count = 0;
	struct allowlist_entry *entry;

	down_read(&allowlist_sem);
	list_for_each_entry(entry, &allowlist, node) {
		if (count + strlen(entry->drvr_name) + sizeof(TRUNCATED) <
		    PAGE_SIZE) {
			count += snprintf(buf + count, PAGE_SIZE - count,
					  "%s\n", entry->drvr_name);
		} else {
			count += snprintf(buf + count, PAGE_SIZE - count,
					  TRUNCATED);
			break;
		}
	}
	up_read(&allowlist_sem);
	return count;
}

static ssize_t drivers_allowlist_store(struct bus_type *bus, const char *buf,
				       size_t count)
{
	struct allowlist_entry *entry;
	ssize_t ret = count;
	unsigned int i;
	char *drv;

	if (!count)
		return -EINVAL;

	drv = kstrndup(buf, count, GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	/* Remove any trailing white spaces */
	strim(drv);
	if (!*drv) {
		ret = -EINVAL;
		goto out_kfree;
	}

	/* Driver names cannot have special characters */
	for (i = 0; i < strlen(drv); i++)
		if (!isalnum(drv[i]) && drv[i] != '_') {
			ret = -EINVAL;
			goto out_kfree;
		}

	down_write(&allowlist_sem);

	/* Lookup in the allowlist */
	list_for_each_entry(entry, &allowlist, node)
		if (!strcmp(drv, entry->drvr_name)) {
			ret = -EEXIST;
			goto out_release_sem;
		}

	/* Add a driver to the allowlist */
	entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry) {
		ret = -ENOMEM;
		goto out_release_sem;
	}
	entry->drvr_name = drv;
	list_add_tail(&entry->node, &allowlist);
	up_write(&allowlist_sem);
	return ret;

out_release_sem:
	up_write(&allowlist_sem);
out_kfree:
	kfree(drv);
	return ret;
}
static BUS_ATTR_RW(drivers_allowlist);

static int __init pci_drivers_allowlist_init(void)
{
	if (trust_external_pci_devices)
		return 0;

	return bus_create_file(&pci_bus_type, &bus_attr_drivers_allowlist);
}
late_initcall(pci_drivers_allowlist_init);

static bool pci_driver_is_allowed(const char *name)
{
	struct allowlist_entry *entry;

	down_read(&allowlist_sem);
	list_for_each_entry(entry, &allowlist, node) {
		if (!strcmp(name, entry->drvr_name)) {
			up_read(&allowlist_sem);
			return true;
		}
	}
	up_read(&allowlist_sem);
	return false;
}

bool pci_drv_allowed_for_untrusted_devs(struct device_driver *drv)
{
	if (trust_external_pci_devices || pci_driver_is_allowed(drv->name))
		return true;

	return false;
}
