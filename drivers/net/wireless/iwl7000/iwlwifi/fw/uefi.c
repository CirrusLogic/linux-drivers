// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright(c) 2021 Intel Corporation
 */

#include "iwl-drv.h"
#include "pnvm.h"
#include "iwl-prph.h"
#include "iwl-io.h"

#include "fw/uefi.h"
#include <linux/efi.h>

/*
 * This is known to be broken on v4.19 and to work on v5.4.  Until we
 * figure out why this is the case and how to make it work, simply
 * disable the feature in old kernels.
 */
#if LINUX_VERSION_IS_GEQ(5,4,0)
#define IWL_EFI_VAR_GUID EFI_GUID(0x92daaf2f, 0xc02b, 0x455b,	\
				  0xb2, 0xec, 0xf5, 0xa3,	\
				  0x59, 0x4f, 0x4a, 0xea)

void *iwl_uefi_get_pnvm(struct iwl_trans *trans, size_t *len)
{
	struct efivar_entry *pnvm_efivar;
	void *data;
	unsigned long package_size;
	int err;

	*len = 0;

	pnvm_efivar = kzalloc(sizeof(*pnvm_efivar), GFP_KERNEL);
	if (!pnvm_efivar)
		return ERR_PTR(-ENOMEM);

	memcpy(&pnvm_efivar->var.VariableName, IWL_UEFI_OEM_PNVM_NAME,
	       sizeof(IWL_UEFI_OEM_PNVM_NAME));
	pnvm_efivar->var.VendorGuid = IWL_EFI_VAR_GUID;

	/*
	 * TODO: we hardcode a maximum length here, because reading
	 * from the UEFI is not working.  To implement this properly,
	 * we have to call efivar_entry_size().
	 */
	package_size = IWL_HARDCODED_PNVM_SIZE;

	data = kmalloc(package_size, GFP_KERNEL);
	if (!data) {
		data = ERR_PTR(-ENOMEM);
		goto out;
	}

	err = efivar_entry_get(pnvm_efivar, NULL, &package_size, data);
	if (err) {
		IWL_DEBUG_FW(trans,
			     "PNVM UEFI variable not found %d (len %zd)\n",
			     err, package_size);
		data = ERR_PTR(err);
		goto out;
	}

	IWL_DEBUG_FW(trans, "Read PNVM from UEFI with size %zd\n", package_size);
	*len = package_size;

out:
	kfree(pnvm_efivar);

	return data;
}

void *iwl_uefi_get_reduced_power(struct iwl_trans *trans, size_t *len)
{
	struct efivar_entry *pnvm_efivar;
	void *data;
	unsigned long package_size;
	int err;

	pnvm_efivar = kzalloc(sizeof(*pnvm_efivar), GFP_KERNEL);
	if (!pnvm_efivar)
		return ERR_PTR(-ENOMEM);

	memcpy(&pnvm_efivar->var.VariableName, IWL_UEFI_REDUCED_POWER_NAME,
	       sizeof(IWL_UEFI_REDUCED_POWER_NAME));
	pnvm_efivar->var.VendorGuid = IWL_EFI_VAR_GUID;

	/*
	 * TODO: we hardcode a maximum length here, because reading
	 * from the UEFI is not working.  To implement this properly,
	 * we have to call efivar_entry_size().
	 */
	package_size = IWL_HARDCODED_REDUCE_POWER_SIZE;

	data = kmalloc(package_size, GFP_KERNEL);
	if (!data) {
		data = ERR_PTR(-ENOMEM);
		*len = 0;
		goto out;
	}

	err = efivar_entry_get(pnvm_efivar, NULL, &package_size, data);
	if (err) {
		IWL_DEBUG_FW(trans,
			     "Reduced Power UEFI variable not found %d (len %zd)\n",
			     err, package_size);
		data = ERR_PTR(err);
		goto out;
	}

	IWL_DEBUG_FW(trans, "Read reduced power from UEFI with size %zd\n",
		     package_size);
	*len = package_size;

out:
	kfree(pnvm_efivar);

	return data;
}
#endif /* >= 5.4 */
