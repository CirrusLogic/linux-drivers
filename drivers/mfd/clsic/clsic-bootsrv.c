/*
 * clsic-bootsrv.c -- CLSIC Bootloader Service
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/crc32.h>
#include <linux/firmware.h>

#include <linux/mfd/clsic/core.h>
#include <linux/mfd/clsic/message.h>
#include <linux/mfd/clsic/irq.h>
#include <linux/mfd/clsic/bootsrv.h>
#include <linux/mfd/clsic/syssrv.h>

/*
 * The way this bootloader service works is that it has two entry points
 *
 * The first is the handler that the messaging layer calls when it receives
 * notifications from the bootloader - we would expect a notification if the
 * device is reset with the fw_update bits set or if the device encounters
 * corrupted flash. The result of this call will be the overall driver state
 * transitioning to one of the bootloader states and the driver maintenance
 * thread being scheduled, this will then call the next entry point, the state
 * handler.
 *
 * The second entry point is the state handler; this is called by the driver
 * maintenance thread when it encounters the driver state within the bootloader
 * range. This handler provides a response to a notification, for instance if
 * the device requests a customer key then the handler will send it.
 *
 * It is expected that the bootloader will send a series of notifications, in
 * the form "give me X ... (driver satisfies request by sending X) ... give me
 * Y ... (driver satisfies request by sending Y)"
 *
 * Normally the bootloader cycle will finish when the MAB file is successfully
 * sent to the device; the response to that message indicates whether the
 * device should then be reset or whether it is ready for use.
 */

/*
 * All the files follow the same pattern: "clsic-<devid>/<devid>-???.bin"
 *	"clsic-"	= 6
 *	<devid>		= 8
 *	"/"		= 1
 *	<devid>		= 8
 *	"-???.bin"	= 8
 *	"\0"		= 1
 */
#define CLSIC_BOOTSRV_FILENAME_MAX_LEN	(6 + 8 + 1 + 8 + 8 + 1)
static const char CLSIC_FWFILE_MAB[] = "clsic-%s/%s-mab.bin";
static const char CLSIC_FWFILE_CPK[] = "clsic-%s/%s-cpk.bin";
static const char CLSIC_FWFILE_FWU[] = "clsic-%s/%s-fwu.bin";

/* Constants describing datafile structures and constants */
#define SIZEOF_PADDING_IN_BYTES		76
#define SIZEOF_PADDING2_IN_BYTES	12

struct clsic_fwheader {
	uint8_t padding[SIZEOF_PADDING_IN_BYTES];
	uint32_t magic;
	uint32_t type;
	uint8_t padding2[SIZEOF_PADDING2_IN_BYTES];
	uint32_t version;
} PACKED;

#define CLSIC_FWMAGIC               0x42554c43UL
#define CLSIC_FWTYPE_FWU            0x20555746UL
#define CLSIC_FWTYPE_CPK            0x204b5043UL
#define CLSIC_FWTYPE_MAB            0x2042414dUL

/* Strings used for describing firmware types */
static const char CLSIC_FWU[] = "FWU";
static const char CLSIC_CPK[] = "CPK";
static const char CLSIC_MAB[] = "MAB";

/*
 * Utility function to convert between an integer file type and a three letter
 * string representation for use in messages.
 */
static const char *clsic_fwtype2string(uint32_t type)
{
	/* recognised firmware types */
	switch (type) {
	case CLSIC_FWTYPE_FWU:
		return CLSIC_FWU;
	case CLSIC_FWTYPE_CPK:
		return CLSIC_CPK;
	case CLSIC_FWTYPE_MAB:
		return CLSIC_MAB;
	default:
		return NULL;
	}
}

static inline const char *clsic_bootsrv_err_to_string(uint8_t err)
{
	switch (err) {
	case CLSIC_ERR_NONE:
		return "Success";
	case CLSIC_ERR_BL_AUTH_FAILED:
		return "Authentication failed";
	case CLSIC_ERR_BL_INVAL_VERSION:
		return "Invalid version";
	case CLSIC_ERR_BL_FLASH_WRITE_FAILED:
		return "Flash write failed";
	case CLSIC_ERR_BL_ARB_CHECK_FAILED:
		return "ARB check failed";
	case CLSIC_ERR_BL_CLUB_TOO_LARGE:
		return "CLUB tool large";
	case CLSIC_ERR_BL_IMG_NAME_CLASH:
		return "Image name clash";
	case CLSIC_ERR_BL_CAB_NOT_1ST_IN_MAB:
		return "CAB not 1st in MAB";
	case CLSIC_ERR_BL_TOO_MANY_IMGS:
		return "Too many images";
	case CLSIC_ERR_BL_NO_MIN_SET_IN_MAB:
		return "Too few images";
	case CLSIC_ERR_BL_FLASH_ERASE_FAILED:
		return "Flash erase failed";
	case CLSIC_ERR_BL_FLASH_READ_FAILED:
		return "Flash read failed";
	case CLSIC_ERR_BL_NBS2_NOT_1ST_IN_CAB:
		return "NBS2 not 1st in CAB";
	case CLSIC_ERR_BL_OSAPP_NOT_2ND_IN_CAB:
		return "OSAPP not 2nd in CAB";
	default:
		return "Unknown";
	}
}

/*
 * Utility function, check that the magic number and the file type in the given
 * header are valid. This function doesn't reopen the firmware file - the
 * filename is just used for the logged message.
 */
static const int clsic_bootsrv_fwheader_check(struct clsic *clsic,
					      const char *filename,
					      struct clsic_fwheader *hdr)
{
	int ret;

	/* Perform a basic sanity check on magic and type */
	if (hdr->magic != CLSIC_FWMAGIC) {
		clsic_err(clsic, "Firmware file %s wrong magic 0x%x\n",
			  filename, hdr->magic);
		ret = -EINVAL;
	} else if (clsic_fwtype2string(hdr->type) == NULL) {
		clsic_err(clsic, "Firmware file %s unknown type 0x%x\n",
			  filename, hdr->type);
		ret = -EINVAL;
	} else {
		ret = 0;
	}

	return ret;
}

/*
 * For a given filename, safely read from the file to populate header
 * structure.  The header can then be used to check it is the expected kind of
 * firmware file and the version of the file.
 */
static const int clsic_bootsrv_fwfile_info(struct clsic *clsic,
					   const char *filename_template,
					   struct clsic_fwheader *hdr)
{
	const struct firmware *firmware;
	int ret;
	char filename[CLSIC_BOOTSRV_FILENAME_MAX_LEN];

	snprintf(filename, CLSIC_BOOTSRV_FILENAME_MAX_LEN,
		 filename_template, clsic_devid_to_string(clsic->devid),
		 clsic_devid_to_string(clsic->devid));

	ret = request_firmware(&firmware, filename, clsic->dev);
	if (ret != 0) {
		clsic_info(clsic, "request_firmware %s failed %d\n",
			   filename, ret);
		return ret;
	}

	/*
	 * This driver has a minimal file header structure that contains only
	 * what it needs, if the file is smaller than that it can't be a real
	 * firmware file.
	 */
	if (firmware->size < sizeof(struct clsic_fwheader)) {
		clsic_info(clsic, "Firmware file %s too small %d\n",
			   filename, firmware->size);
		ret = -EINVAL;
		goto release_exit;
	}

	memcpy(hdr, firmware->data, sizeof(struct clsic_fwheader));

	/* Finally sanity check the file's magic numbers */
	ret = clsic_bootsrv_fwheader_check(clsic, filename, hdr);

release_exit:
	release_firmware(firmware);
	return ret;
}

/*
 * For a given firmware filename, safely interrogate the header and return the
 * version within.
 *
 * To prevent an overlap of ranges in this function if an error is encountered
 * the version returned is 0. This should mean that if the device has valid
 * firmware then the firmware update process will not be started if an error is
 * encountered.
 *
 * Traditionally the top bit is used to indicate returned value is an error
 * codes but that bit is used in the major version.
 */
static const uint32_t clsic_bootsrv_file_version(struct clsic *clsic,
						 const char *filename)
{
	struct clsic_fwheader hdr;
	int ret;

	ret = clsic_bootsrv_fwfile_info(clsic, filename, &hdr);
	if (ret != 0)
		return 0;

	clsic_dbg(clsic, "%s: %s 0x%x (%d.%d.%d)\n",
		  filename, clsic_fwtype2string(hdr.type), hdr.version,
		  (hdr.version & CLSIC_SVCVER_MAJ_MASK) >>
		  CLSIC_SVCVER_MAJ_SHIFT,
		  (hdr.version & CLSIC_SVCVER_MIN_MASK) >>
		  CLSIC_SVCVER_MIN_SHIFT,
		  (hdr.version & CLSIC_SVCVER_BLD_MASK) >>
		  CLSIC_SVCVER_BLD_SHIFT);

	return hdr.version;
}

/* Structure containing the Boot Service instance data */
struct clsic_bootsrv_struct {
	struct clsic *clsic;

	struct clsic_service *srv;
	u32 fw_crc;
};

/*
 * Transmits the contents of the given filename as bulk data payload to the
 * bootloader with the given message id.
 *
 * Performs basic sanity check on the file header to make sure it is valid and
 * matches the expected type.
 */
static int clsic_bootsrv_sendfile(struct clsic *clsic,
				  const char *filename_template,
				  uint32_t type, uint32_t msgid,
				  union clsic_bl_msg *msg_rsp)
{
	const struct firmware *firmware;
	int ret;
	union t_clsic_generic_message msg_cmd;
	struct clsic_fwheader *hdr;
	uint8_t err;
	u32 fw_crc;
	struct clsic_bootsrv_struct *bootsrv =
		clsic->service_handlers[CLSIC_SRV_INST_BLD]->data;
	char filename[CLSIC_BOOTSRV_FILENAME_MAX_LEN];

	if (bootsrv == NULL) {
		clsic_err(clsic, "No bootldr service data\n");
		return -EINVAL;
	}

	snprintf(filename, CLSIC_BOOTSRV_FILENAME_MAX_LEN,
		 filename_template, clsic_devid_to_string(clsic->devid),
		 clsic_devid_to_string(clsic->devid));

	ret = request_firmware(&firmware, filename, clsic->dev);
	if (ret != 0) {
		clsic_err(clsic,
			  "request_firmware failed '%s' = %d (check files)\n",
			  filename, ret);
		clsic->blrequest = CLSIC_BL_IDLE;
		clsic_device_error(clsic, CLSIC_DEVICE_ERROR_LOCKNOTHELD);
		return ret;
	}

	clsic_dbg(clsic, "%s len: %d\n", filename, firmware->size);

	if (firmware->size < sizeof(struct clsic_fwheader)) {
		clsic_err(clsic, "Firmware file '%s' too small %d\n",
			  filename, firmware->size);
		ret = -EINVAL;
		goto release_exit;
	}

	hdr = (struct clsic_fwheader *)firmware->data;

	/* Sanity check the file's magic numbers */
	if (clsic_bootsrv_fwheader_check(clsic, filename, hdr) != 0) {
		ret = -EINVAL;
		goto release_exit;
	}

	if (hdr->type != type) {
		clsic_err(clsic,
			  "Wrong file type in '%s': expected 0x%x, file 0x%x\n",
			  filename, type, hdr->type);
		ret = -EINVAL;
		goto release_exit;
	}

	if ((hdr->type == CLSIC_FWTYPE_MAB) && clsic->volatile_memory) {
		fw_crc = crc32(0, firmware->data, firmware->size);
		if (fw_crc != bootsrv->fw_crc) {
			clsic_info(clsic, "mab changed : expected %x got %x\n",
				bootsrv->fw_crc, fw_crc);
			if (clsic->service_states == CLSIC_ENUMERATED)
				clsic->service_states =
				   CLSIC_REENUMERATION_REQUIRED;
			bootsrv->fw_crc = fw_crc;
		}
	}

	/* Finally send the file as the bulk data payload of the given msgid */
	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   CLSIC_SRV_INST_BLD, msgid);
	msg_cmd.bulk_cmd.hdr.bulk_sz = firmware->size;

	ret = clsic_send_msg_sync(clsic, &msg_cmd,
				  (union t_clsic_generic_message *)msg_rsp,
				  firmware->data, firmware->size,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	if (ret != 0) {
		clsic_info(clsic, "Failed to send: %d\n", ret);
		ret = -EIO;
	} else if (msg_rsp->rsp_set_mab.hdr.err != 0) {
		err = msg_rsp->rsp_set_mab.hdr.err;
		clsic_info(clsic, "Response error_code 0x%x : '%s'\n",
			   err, clsic_bootsrv_err_to_string(err));
		ret = -EIO;
	}

release_exit:
	/*
	 * Any failures to send files to the bootloader are considered fatal.
	 */
	if (ret != 0) {
		clsic->blrequest = CLSIC_BL_IDLE;
		clsic_device_error(clsic, CLSIC_DEVICE_ERROR_LOCKNOTHELD);
	}

	release_firmware(firmware);
	return ret;
}

/*
 * Called by the messaging layer in response to receiving a notification
 * message
 */
static int clsic_bootsrv_msghandler(struct clsic *clsic,
				    struct clsic_service *handler,
				    struct clsic_message *msg)
{
	uint8_t msgid = clsic_get_messageid(msg);
	int ret = CLSIC_HANDLED;

	/*
	 * Most of the notifications result in the driver sending a file to the
	 * bootloader service in the maintenance thread context.
	 *
	 * This function cannot send the response message directly because this
	 * context is used to progress all notifications; as sending files uses
	 * bulk messaging and that involves a system service notification if we
	 * blocked this context the messaging layer would deadlock.
	 */
	switch (msgid) {
	case CLSIC_BL_MSG_N_REQ_FWU:
		clsic_dbg(clsic, "Request FWU bundle\n");
		clsic->blrequest = CLSIC_BL_FWU;
		break;
	case CLSIC_BL_MSG_N_REQ_CPK:
		clsic_dbg(clsic, "Request CPK bundle\n");
		clsic->blrequest = CLSIC_BL_CPK;
		break;
	case CLSIC_BL_MSG_N_REQ_MAB:
		clsic_dbg(clsic, "Request MAB bundle\n");
		clsic->blrequest = CLSIC_BL_MAB;
		break;
	case CLSIC_BL_MSG_N_NO_BOOTABLE_COMP:
	case CLSIC_BL_MSG_N_FAILED_FLASH_AUTH:
	case CLSIC_BL_MSG_N_FLASH_CORRUPTED:
		clsic_err(clsic, "CSLIC boot fail: %d : %s %d %d\n",
			  msgid, clsic_state_to_string(clsic->state),
			  clsic->blrequest, clsic->service_states);
		clsic_state_set(clsic, CLSIC_STATE_HALTED,
				CLSIC_STATE_CHANGE_LOCKNOTHELD);
		break;
	default:
		clsic_dump_message(clsic, msg, "clsic_bootsrv_msghandler");
		ret = CLSIC_UNHANDLED;
	}
	if (clsic->blrequest != CLSIC_BL_IDLE) {
		/*
		 * If a bootloader message bas been received then purge the
		 * message queues as this will interrupt any messages currently
		 * in the system that may be blocked - the corresponding
		 * bootloader message will be sent when the maintenance handler
		 * runs.
		 */
		clsic_purge_message_queues(clsic);
		schedule_work(&clsic->maintenance_handler);
	}
	return ret;
}

/*
 * Called by the maintenance thread to progress bootloader states
 *
 * The majority of the states in the handler are for sending files to the
 * bootloader after receiving a notification.
 */
int clsic_bootsrv_state_handler(struct clsic *clsic)
{
	int ret = 0;
	union clsic_bl_msg msg_rsp;
	enum clsic_blrequests saved_request = clsic->blrequest;

	clsic->blrequest = CLSIC_BL_EXPECTED;

	switch (saved_request) {
	case CLSIC_BL_FWU:
		ret = clsic_bootsrv_sendfile(clsic,
					     CLSIC_FWFILE_FWU,
					     CLSIC_FWTYPE_FWU,
					     CLSIC_BL_MSG_CR_SET_FWU,
					     &msg_rsp);
		break;
	case CLSIC_BL_CPK:
		ret = clsic_bootsrv_sendfile(clsic,
					     CLSIC_FWFILE_CPK,
					     CLSIC_FWTYPE_CPK,
					     CLSIC_BL_MSG_CR_SET_CPK,
					     &msg_rsp);
		break;
	case CLSIC_BL_MAB:
		ret = clsic_bootsrv_sendfile(clsic,
					     CLSIC_FWFILE_MAB,
					     CLSIC_FWTYPE_MAB,
					     CLSIC_BL_MSG_CR_SET_MAB,
					     &msg_rsp);
		if (ret == 0) {
			/*
			 * Successfully downloading the MAB is normally the end
			 * of the bootloader exchange.
			 */
			clsic->blrequest = CLSIC_BL_IDLE;
			if (!(msg_rsp.rsp_set_mab.flags &
			      CLSIC_BL_RESET_NOT_REQUIRED)) {
				/* This device needs to be software reset */
				clsic_irq_disable(clsic);
				clsic_soft_reset(clsic);
				clsic_irq_enable(clsic);
			}
			clsic_state_set(clsic, CLSIC_STATE_RESUMING,
					CLSIC_STATE_CHANGE_LOCKNOTHELD);

			schedule_work(&clsic->maintenance_handler);
		}
		break;
	case CLSIC_BL_UPDATE:
		mutex_lock(&clsic->message_lock);
		/* If services have been discovered, reenumerate them */
		if (clsic->service_states == CLSIC_ENUMERATED)
			clsic->service_states = CLSIC_REENUMERATION_REQUIRED;
		clsic_fwupdate_reset(clsic);
		mutex_unlock(&clsic->message_lock);
		break;
	case CLSIC_BL_EXPECTED:
		break;
	default:
		clsic_err(clsic, "Unrecognised: %d\n", saved_request);
	}
	return ret;
}

static ssize_t clsic_show_file_fw_version(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct clsic *clsic = dev_get_drvdata(dev);
	uint32_t file_version = clsic_bootsrv_file_version(clsic,
							   CLSIC_FWFILE_MAB);

	return snprintf(buf, PAGE_SIZE, "%d.%d.%d\n",
		       (file_version & CLSIC_SVCVER_MAJ_MASK) >>
		       CLSIC_SVCVER_MAJ_SHIFT,
		       (file_version & CLSIC_SVCVER_MIN_MASK) >>
		       CLSIC_SVCVER_MIN_SHIFT,
		       (file_version & CLSIC_SVCVER_BLD_MASK) >>
		       CLSIC_SVCVER_BLD_SHIFT);
}
static DEVICE_ATTR(file_fw_version, 0444, clsic_show_file_fw_version, NULL);

static ssize_t clsic_store_device_fw_version(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct clsic *clsic = dev_get_drvdata(dev);

	if (!strncmp(buf, "update", strlen("update"))) {
		/* Debug control prevents device state changes */
		if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED)
			return -EPERM;

		if (clsic->volatile_memory)
			return -EPERM;

		/*
		 * If the device previously failed clear that state so it can
		 * be powered on (pm resume is prevented in the HALTED state).
		 */
		if (clsic->state == CLSIC_STATE_HALTED)
			clsic_state_set(clsic, CLSIC_STATE_OFF,
					CLSIC_STATE_CHANGE_LOCKNOTHELD);

		clsic->blrequest = CLSIC_BL_UPDATE;

		pm_runtime_suspend(clsic->dev);
		clsic_pm_wake(clsic);

		/*
		 * The bootloader state will be progressed in the maintenance
		 * thread
		 */
		schedule_work(&clsic->maintenance_handler);
	}

	return count;
}

static ssize_t clsic_show_device_fw_version(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct clsic *clsic = dev_get_drvdata(dev);
	uint32_t device_version =
		clsic->service_handlers[CLSIC_SRV_INST_SYS]->service_version;

	return snprintf(buf, PAGE_SIZE, "%d.%d.%d\n",
		       (device_version & CLSIC_SVCVER_MAJ_MASK) >>
		       CLSIC_SVCVER_MAJ_SHIFT,
		       (device_version & CLSIC_SVCVER_MIN_MASK) >>
		       CLSIC_SVCVER_MIN_SHIFT,
		       (device_version & CLSIC_SVCVER_BLD_MASK) >>
		       CLSIC_SVCVER_BLD_SHIFT);
}
static DEVICE_ATTR(device_fw_version, 0644, clsic_show_device_fw_version,
		   clsic_store_device_fw_version);

static void clsic_bootsrv_service_stop(struct clsic *clsic,
				      struct clsic_service *handler)
{
	device_remove_file(clsic->dev, &dev_attr_device_fw_version);
	device_remove_file(clsic->dev, &dev_attr_file_fw_version);
	kfree(handler->data);
}

int clsic_bootsrv_service_start(struct clsic *clsic,
				struct clsic_service *handler)
{
	struct clsic_bootsrv_struct *bootsrv;

	if (handler->data != NULL)
		return 0;

	bootsrv = kzalloc(sizeof(struct clsic_bootsrv_struct), GFP_KERNEL);
	if (bootsrv == NULL)
		return -ENOMEM;

	bootsrv->clsic = clsic;
	bootsrv->srv = handler;

	handler->callback = &clsic_bootsrv_msghandler;
	handler->stop = &clsic_bootsrv_service_stop;
	handler->data = bootsrv;

	device_create_file(clsic->dev, &dev_attr_device_fw_version);
	device_create_file(clsic->dev, &dev_attr_file_fw_version);

	return 0;
}
