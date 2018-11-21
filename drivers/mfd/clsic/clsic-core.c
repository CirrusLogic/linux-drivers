/*
 * clsic-core.c -- CLSIC core driver initialisation
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include <linux/mfd/clsic/core.h>
#include "clsic-trace.h"
#include <linux/mfd/clsic/message.h>
#include <linux/mfd/clsic/irq.h>
#include <linux/mfd/clsic/bootsrv.h>
#include <linux/mfd/clsic/syssrv.h>
#include <linux/mfd/clsic/rassrv.h>

static void clsic_init_sysfs(struct clsic *clsic);
static void clsic_deinit_sysfs(struct clsic *clsic);
static int clsic_pm_service_transition(struct clsic *clsic, int pm_event);

#ifdef CONFIG_OF
const struct of_device_id clsic_of_match[] = {
	{ .compatible = "cirrus,clsic" },
	{},
};
EXPORT_SYMBOL_GPL(clsic_of_match);
#endif

static const char * const clsic_core_supplies[] = {
	"VDD_A",
	"VDD_IO1",
};

static bool clsic_bootonload = true;
module_param(clsic_bootonload, bool, 0000);
MODULE_PARM_DESC(clsic_bootonload,
		 "Whether to boot the device when the module is loaded");

/* CLSIC_PM_AUTOSUSPEND_MS is used as the idle time for runtime autosuspend */
#define CLSIC_PM_AUTOSUSPEND_MS		(15 * 1000)

static atomic_t clsic_instances_count = ATOMIC_INIT(0);

static void clsic_enable_hard_reset(struct clsic *clsic)
{
	if (clsic->reset_gpio)
		gpiod_set_value_cansleep(clsic->reset_gpio, 0);
}

static void clsic_disable_hard_reset(struct clsic *clsic)
{
	if (clsic->reset_gpio) {
		gpiod_set_value_cansleep(clsic->reset_gpio, 1);
		usleep_range(1000, 2000);
	}
}

/*
 * NOTE: These are quite large timeouts whilst we are in development
 */
#define CLSIC_BOOT_POLL_MICROSECONDS    5000
#define CLSIC_BOOT_TIMEOUT_MICROSECONDS 2000000

static bool clsic_wait_for_boot_done(struct clsic *clsic)
{
	unsigned int val;
	int ret;

	ret = regmap_read_poll_timeout(clsic->regmap, TACNA_IRQ1_EINT_2, val,
				       (val & TACNA_BOOT_DONE_EINT1_MASK),
				       CLSIC_BOOT_POLL_MICROSECONDS,
				       CLSIC_BOOT_TIMEOUT_MICROSECONDS);
	if (ret) {
		clsic_err(clsic, "Failed to get BOOT_DONE: %d\n", ret);
		return false;
	}

	return true;
}

void clsic_soft_reset(struct clsic *clsic)
{
	regmap_write(clsic->regmap, TACNA_SFT_RESET, CLSIC_SOFTWARE_RESET_CODE);
	clsic_wait_for_boot_done(clsic);
}

static void clsic_hard_reset(struct clsic *clsic)
{
	clsic_enable_hard_reset(clsic);
	usleep_range(1000, 2000);
	clsic_disable_hard_reset(clsic);

	clsic_wait_for_boot_done(clsic);
}

int clsic_fwupdate_reset(struct clsic *clsic)
{
	int ret = 0;

	clsic->blrequest = CLSIC_BL_EXPECTED;

	ret = regmap_update_bits(clsic->regmap, CLSIC_FW_UPDATE_REG,
				 CLSIC_FW_UPDATE_BIT, CLSIC_FW_UPDATE_BIT);
	if (ret == 0) {
		clsic_irq_disable(clsic);
		clsic_soft_reset(clsic);
		clsic_irq_enable(clsic);
	}

	return ret;
}

static bool clsic_supported_devid(struct clsic *clsic)
{
	int ret = 0;
	unsigned int revid;
	unsigned int fabid;
	unsigned int relid;
	unsigned int otpid;

	/*
	 * When devid is 0 read from the device and print the other IDs to aid
	 * investigations.
	 */
	if (clsic->devid == 0) {
		ret = regmap_read(clsic->regmap, TACNA_DEVID, &clsic->devid);
		if (ret)
			clsic_warn(clsic, "Failed to read ID register: %d\n",
				   ret);

		regmap_read(clsic->regmap, TACNA_REVID, &revid);
		revid &= (TACNA_AREVID_MASK | TACNA_MTLREVID_MASK);
		regmap_read(clsic->regmap, TACNA_FABID, &fabid);
		fabid &= TACNA_FABID_MASK;
		regmap_read(clsic->regmap, TACNA_RELID, &relid);
		relid &= TACNA_RELID_MASK;
		regmap_read(clsic->regmap, TACNA_OTPID, &otpid);
		otpid &= TACNA_OTPID_MASK;

		clsic_info(clsic,
			   "DEVID 0x%x, REVID 0x%x, FABID 0x%x, RELID 0x%x, OTPID 0x%x\n",
			   clsic->devid, revid, fabid, relid, otpid);
	}

	switch (clsic->devid) {
	case CLSIC_SUPPORTED_ID_48AB50:
	case CLSIC_SUPPORTED_ID_EMULATED_CODEC:
	case CLSIC_SUPPORTED_ID_48AC40:
		return true;
	default:
		return false;
	}
}

static int clsic_shutdown_notifier_cb(struct notifier_block *this,
				      unsigned long code, void *data)
{
	struct clsic *clsic = container_of(this, struct clsic,
					   clsic_shutdown_notifier);

	pr_devel("%s() clsic %p code %ld\n", __func__, clsic, code);

	if ((code == SYS_DOWN) || (code == SYS_HALT))
		/* signal the device is shutting down - halt the CLSIC device */
		clsic_send_shutdown_cmd(clsic);

	return NOTIFY_DONE;
}

static int clsic_register_reboot_notifier(struct clsic *clsic)
{
	clsic->clsic_shutdown_notifier.notifier_call =
		&clsic_shutdown_notifier_cb;

	clsic->instance = atomic_inc_return(&clsic_instances_count);

	return register_reboot_notifier(&clsic->clsic_shutdown_notifier);
}

static int clsic_unregister_reboot_notifier(struct clsic *clsic)
{
	return unregister_reboot_notifier(&clsic->clsic_shutdown_notifier);
}

static int clsic_vdd_d_notify(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	struct clsic *clsic = container_of(nb, struct clsic,
					   vdd_d_notifier);

	dev_dbg(clsic->dev, "VDD_D notify %lx\n", action);

	if (action & REGULATOR_EVENT_DISABLE)
		clsic->vdd_d_powered_off = true;

	return NOTIFY_DONE;
}

static void clsic_regulators_deregister_disable(struct clsic *clsic)
{
	regulator_disable(clsic->vdd_d);
	regulator_bulk_disable(clsic->num_core_supplies, clsic->core_supplies);
	regulator_unregister_notifier(clsic->vdd_d, &clsic->vdd_d_notifier);
	regulator_put(clsic->vdd_d);
}

static int clsic_regulators_register_enable(struct clsic *clsic)
{
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(clsic_core_supplies); i++)
		clsic->core_supplies[i].supply = clsic_core_supplies[i];
	clsic->num_core_supplies = ARRAY_SIZE(clsic_core_supplies);

	ret = devm_regulator_bulk_get(clsic->dev, clsic->num_core_supplies,
				      clsic->core_supplies);
	if (ret) {
		clsic_err(clsic, "Failed to request core supplies: %d\n", ret);
		return ret;
	}

	clsic->vdd_d = regulator_get(clsic->dev, "VDD_D");
	if (IS_ERR(clsic->vdd_d)) {
		ret = PTR_ERR(clsic->vdd_d);
		clsic_err(clsic, "Failed to request VDD_D: %d\n", ret);

		/*
		 * since devres_* version is used to get core regulators
		 * no need for explicit put for them
		 */
		return ret;
	}

	clsic->vdd_d_powered_off = false;
	clsic->vdd_d_notifier.notifier_call = clsic_vdd_d_notify;
	ret = regulator_register_notifier(clsic->vdd_d,
					  &clsic->vdd_d_notifier);
	if (ret) {
		clsic_err(clsic, "Failed to register VDD_D notifier %d\n", ret);
		goto vdd_d_notifier_failed;
	}

	ret = regulator_bulk_enable(clsic->num_core_supplies,
				    clsic->core_supplies);
	if (ret) {
		clsic_err(clsic, "Failed to enable core supplies: %d\n", ret);
		goto core_enable_failed;
	}

	ret = regulator_enable(clsic->vdd_d);
	if (ret) {
		clsic_err(clsic, "Failed to enable VDD_D: %d\n", ret);
		goto vdd_enable_failed;
	}

	return 0;

vdd_enable_failed:
	regulator_bulk_disable(clsic->num_core_supplies, clsic->core_supplies);
core_enable_failed:
	regulator_unregister_notifier(clsic->vdd_d, &clsic->vdd_d_notifier);
vdd_d_notifier_failed:
	regulator_put(clsic->vdd_d);

	return ret;
}

void clsic_state_set(struct clsic *clsic,
		     const enum clsic_states new_state,
		     bool lock_held)
{
	if (!lock_held)
		mutex_lock(&clsic->message_lock);

	trace_clsic_statechange(clsic->state, new_state);
	clsic->state = new_state;

	if (!lock_held)
		mutex_unlock(&clsic->message_lock);
}

/*
 * All devices in the CLSIC family have bootloader and system services in fixed
 * positions.
 *
 * Preregister handlers for them so that if a notification arrives during boot
 * it can be suitably handled.
 */
static int clsic_services_init(struct clsic *clsic)
{
	int ret = 0;

	mutex_init(&clsic->service_lock);

	clsic->service_states = CLSIC_ENUMERATION_REQUIRED;

	ret = clsic_register_service_handler(clsic,
					     CLSIC_SRV_INST_SYS,
					     CLSIC_SRV_TYPE_SYS,
					     0, clsic_system_service_start);
	if (ret != 0)
		return ret;

	ret = clsic_register_service_handler(clsic,
					     CLSIC_SRV_INST_BLD,
					     CLSIC_SRV_INST_BLD,
					     0, clsic_bootsrv_service_start);
	if (ret != 0) {
		clsic->service_handlers[CLSIC_SRV_INST_SYS]->stop(clsic,
				   clsic->service_handlers[CLSIC_SRV_INST_SYS]);
		clsic_deregister_service_handler(clsic,
				   clsic->service_handlers[CLSIC_SRV_INST_SYS]);
		kfree(clsic->service_handlers[CLSIC_SRV_INST_SYS]);
	}
	return ret;
}

int clsic_dev_init(struct clsic *clsic)
{
	int ret = 0;

	clsic_info(clsic, "%p (bootonload: %d)\n", clsic, clsic_bootonload);

	dev_set_drvdata(clsic->dev, clsic);

	ret = clsic_regulators_register_enable(clsic);
	if (ret != 0) {
		clsic_err(clsic, "Regulator register failed=%d", ret);
		return ret;
	}

	clsic->reset_gpio = devm_gpiod_get(clsic->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(clsic->reset_gpio)) {
		ret = PTR_ERR(clsic->reset_gpio);
		clsic_err(clsic,
			  "DT property reset-gpio is missing or malformed %d\n",
			  ret);
		clsic->reset_gpio = NULL;
	}

	if (clsic->reset_gpio == NULL) {
		clsic_warn(clsic,
			   "Running without reset GPIO is not recommended\n");
		clsic_soft_reset(clsic);
	} else {
		clsic_hard_reset(clsic);
	}

	if (!clsic_supported_devid(clsic)) {
		clsic_err(clsic, "Unknown device ID: %x\n", clsic->devid);
		ret = -EINVAL;
		goto err_reset;
	}

	clsic->volatile_memory = of_property_read_bool(clsic->dev->of_node,
						       "volatile_memory");

	INIT_WORK(&clsic->maintenance_handler, clsic_maintenance);

	clsic_init_sysfs(clsic);

	clsic_init_debugfs(clsic);

	ret = clsic_setup_message_interface(clsic);
	if (ret != 0)
		goto messaging_failed;

	ret = clsic_register_reboot_notifier(clsic);
	if (ret != 0) {
		clsic_err(clsic, "Register reboot notifier ret=%d", ret);
		goto notifier_failed;
	}

	/* The irq starts disabled */
	ret = clsic_irq_init(clsic);
	if (ret != 0)
		goto irq_failed;

	ret = clsic_services_init(clsic);
	if (ret != 0)
		goto service_init_failed;

	init_completion(&clsic->pm_completion);
	pm_runtime_set_suspended(clsic->dev);
	pm_runtime_mark_last_busy(clsic->dev);
	pm_runtime_set_autosuspend_delay(clsic->dev, CLSIC_PM_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(clsic->dev);
	pm_runtime_enable(clsic->dev);

	if (clsic_bootonload)
		clsic_pm_wake(clsic);

	/*
	 * At this point the device is NOT fully setup - the device will block
	 * until runtime resume has completed (device state is resuming), the
	 * device reaches the "ON" state after enumeration has been completed
	 * by the maintenance thread.
	 *
	 * If a conclusive decision on failed/succeeded is required then the
	 * device state should be tested here until it reaches HALTED or ON.
	 */

	return 0;

	/* If errors are encountered, tidy up and deallocate as appropriate */
service_init_failed:
	clsic_irq_exit(clsic);
irq_failed:
notifier_failed:
	clsic_unregister_reboot_notifier(clsic);
	clsic_shutdown_message_interface(clsic);
messaging_failed:
	clsic_deinit_debugfs(clsic);
	clsic_deinit_sysfs(clsic);

err_reset:
	clsic_enable_hard_reset(clsic);

	return ret;
}
EXPORT_SYMBOL_GPL(clsic_dev_init);

/*
 * Called when the device has informed the system service of a panic or other
 * fatal error.
 */
void clsic_dev_panic(struct clsic *clsic, struct clsic_message *msg)
{
	int ret;

	trace_clsic_dev_panic(clsic->state);
	clsic_dump_message(clsic, msg, "Panic Notification");
	memcpy(&clsic->last_panic.msg, &msg->fsm, CLSIC_FIXED_MSG_SZ);

	ret = clsic_fifo_readbulk_payload(clsic, msg, (uint8_t *)
					  &clsic->last_panic.di,
					  sizeof(clsic->last_panic.di));

	clsic_info(clsic, "ret: %d version: %d encrypted: %d\n",
		   ret,
		   clsic->last_panic.di.version,
		   clsic->last_panic.di.encrypted);

	/* If the device panics don't attempt to recover it automatically */
	clsic_device_error(clsic, CLSIC_DEVICE_ERROR_LOCKNOTHELD);
}

/*
 * The driver maintenance thread used for progressing state - the kernel init
 * context can't be used as it would block kernel boot and the messaging thread
 * can't be used as that thread is required to progress messages.
 *
 * The main tasks that this thread progresses are service enumeration and
 * servicing the bootloader with any data it requires to start or upgrade the
 * device.
 *
 * This thread is responsible for releasing the pm_runtime resume() context
 * when the device is considered fully booted. It normally does that at the end
 * after the device has had any firmware download or enumeration demands
 * serviced, but it is also released when any error is encountered and when
 * debugcontrol is activated when the device is switched off.
 *
 * If the device is in one of the bootloader states then call the bootloader
 * service handler to progress the system booting. The bootloader end state
 * signals that the bootloader service has successfully downloaded software to
 * the device. This is separated out into a different logical state as at this
 * point some devices will be reset whilst on others the driver should attempt
 * service enumeration.
 */
void clsic_maintenance(struct work_struct *data)
{
	struct clsic *clsic = container_of(data, struct clsic,
					   maintenance_handler);

	clsic_info(clsic, "States: %s %d %d\n",
		   clsic_state_to_string(clsic->state),
		   clsic->blrequest, clsic->service_states);

	if (clsic->blrequest != CLSIC_BL_IDLE) {
		if (clsic_bootsrv_state_handler(clsic) != 0) {
			clsic_err(clsic,
				  "Bootloader operation failed (%s s: %d b: %d)\n",
				  clsic_state_to_string(clsic->state),
				  clsic->service_states,
				  clsic->blrequest);
			goto pm_complete_exit;
		}
		return;
	}

	if ((clsic->state != CLSIC_STATE_RESUMING) &&
	    (clsic->state != CLSIC_STATE_DEBUGCONTROL_REQUESTED))
		goto pm_complete_exit;

	switch (clsic->service_states) {
	case CLSIC_ENUMERATED:
		/* Nothing to do (typical power on resume) */
		break;
	case CLSIC_ENUMERATION_REQUIRED:
		/*
		 * Perform the first device enumeration - the pm_runtime resume
		 * context needs to be released first as enumeration is likely
		 * to start MFD children and attempt to take the pm_runtime
		 * lock.
		 */
		complete(&clsic->pm_completion);
		/* FALLTHROUGH */
	case CLSIC_REENUMERATION_REQUIRED:
		clsic_system_service_enumerate(clsic);
		break;
	case CLSIC_UNLOADING:
		/* Fast exit */
		clsic_state_set(clsic, CLSIC_STATE_ON,
				CLSIC_STATE_CHANGE_LOCKNOTHELD);

		goto pm_complete_exit;
	default:
		clsic_info(clsic, "Service state %d\n", clsic->service_states);
		goto pm_complete_exit;
	}

	if ((clsic->service_states == CLSIC_ENUMERATED) &&
	    (clsic_pm_service_transition(clsic, PM_EVENT_RESUME) == 0))
		clsic_state_set(clsic,
				CLSIC_STATE_ON,
				CLSIC_STATE_CHANGE_LOCKNOTHELD);

pm_complete_exit:
	/*
	 * Always make sure pm_runtime is released after enumeration, in the
	 * case of the first device enumeration this will already have been
	 * signalled - the number of times complete is called is not important.
	 */
	complete(&clsic->pm_completion);
}

int clsic_dev_exit(struct clsic *clsic)
{
	int i;

	if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED) {
		/* this put matches the one on grant so the module can exit */
		pm_runtime_mark_last_busy(clsic->dev);
		pm_runtime_put_autosuspend(clsic->dev);
		clsic_irq_enable(clsic);
	}

	/*
	 * Setting services state to unloading will allow the device to power
	 * on but prevent enumeration and reenumeration
	 */
	clsic->service_states = CLSIC_UNLOADING;

	/*
	 * Volatile devices may need to be resumed at this point in time, place
	 * a pm get on the device so the power is applied and then wait for the
	 * state to become ON or HALTED.
	 */
	pm_runtime_get_sync(clsic->dev);

	if (clsic_wait_for_state(clsic, CLSIC_STATE_ON,
				 CLSIC_WAIT_FOR_STATE_MAX_CYCLES,
				 CLSIC_WAIT_FOR_STATE_DELAY_MS))
		clsic_info(clsic, "Warning: state is %s\n",
			   clsic_state_to_string(clsic->state));

	pm_runtime_mark_last_busy(clsic->dev);
	pm_runtime_put_autosuspend(clsic->dev);

	cancel_work_sync(&clsic->maintenance_handler);

	/*
	 * If any of the services registered child devices this will call their
	 * remove callback. This is being done before shutting down the service
	 * handlers because child mfd drivers may require service functionality
	 * to shutdown cleanly, such as the register access service.
	 */
	mfd_remove_devices(clsic->dev);

	clsic_unregister_reboot_notifier(clsic);

	/*
	 * To safely shutdown the device this driver will need to transition
	 * the device's state machine to idle and then issue a shutdown
	 * command, after which device power can be removed.
	 *
	 * Give all the service handlers a chance to tidy themselves up, they
	 * can send more messages to the device to tidy the services up.  On
	 * return they are expected to have deregistered and released all their
	 * resources. When all services have been shutdown the device should be
	 * in an idle state and be ready to be shutdown.
	 *
	 * The ordering of shutdown is important, service instance 0 is the
	 * system service that is used in some bulk transfers as well as error
	 * handling and will issue the shutdown command.
	 *
	 * As that service should be done last, shut them down in reverse order.
	 */
	for (i = CLSIC_SERVICE_MAX; i >= 0; i--) {
		if (clsic->service_handlers[i] != NULL) {
			clsic_dbg(clsic, "Stopping %d: %pF\n",
				  i, clsic->service_handlers[i]->stop);
			/* a stop() callback on handlers is optional */
			if (clsic->service_handlers[i]->stop != NULL)
				clsic->service_handlers[i]->stop(clsic,
						    clsic->service_handlers[i]);

			clsic_deregister_service_handler(clsic,
						    clsic->service_handlers[i]);

			kfree(clsic->service_handlers[i]);
		}
	}

	/* Place the driver into suspend and give it a chance to get there */
	pm_runtime_suspend(clsic->dev);
	if (clsic_wait_for_state(clsic, CLSIC_STATE_OFF,
				 CLSIC_WAIT_FOR_STATE_MAX_CYCLES,
				 CLSIC_WAIT_FOR_STATE_DELAY_MS))
		clsic_info(clsic, "Warning: state is %s\n",
			   clsic_state_to_string(clsic->state));

	i = atomic_read(&clsic->dev->power.usage_count);
	if (i != 0)
		clsic_err(clsic, "Final power usage_count: %d\n", i);

	pm_runtime_set_suspended(clsic->dev);
	pm_runtime_disable(clsic->dev);

	clsic_irq_exit(clsic);

	clsic_deinit_debugfs(clsic);
	clsic_deinit_sysfs(clsic);

	clsic_shutdown_message_interface(clsic);

	clsic_regulators_deregister_disable(clsic);

	clsic_enable_hard_reset(clsic);

	return 0;
}
EXPORT_SYMBOL_GPL(clsic_dev_exit);

/* Register as a handler for a service ID */
int clsic_register_service_handler(struct clsic *clsic,
				   uint8_t service_instance,
				   uint16_t service_type,
				   uint32_t service_version,
				   int (*start)(struct clsic *clsic,
						struct clsic_service *handler))
{
	struct clsic_service *tmp_handler;
	int ret = 0;

	clsic_dbg(clsic, "%p %d: %pF\n", clsic, service_instance, start);

	if (service_instance > CLSIC_SERVICE_MAX) {
		clsic_err(clsic, "%p:%d out of range\n", start,
			  service_instance);
		return -EINVAL;
	}

	mutex_lock(&clsic->service_lock);
	if (clsic->service_handlers[service_instance] != NULL) {
		clsic_dbg(clsic, "%d pre-registered %p\n", service_instance,
			  start);

		/*
		 * Check the service type matches, if not call stop and
		 * repopulate as a new handler.
		 */
		tmp_handler = clsic->service_handlers[service_instance];
		if ((tmp_handler->service_instance != service_instance) ||
		    (tmp_handler->service_type != service_type)) {
			clsic_err(clsic,
				  "handler different: instance %d:%d type 0x%x:0x%x\n",
				  service_instance,
				  tmp_handler->service_instance,
				  service_type, tmp_handler->service_type);

			if (tmp_handler->stop != NULL)
				tmp_handler->stop(clsic, tmp_handler);

			tmp_handler->service_instance = service_instance;
			tmp_handler->service_type = service_type;
		}
	} else {
		tmp_handler = kzalloc(sizeof(*tmp_handler), GFP_KERNEL);
		if (tmp_handler == NULL) {
			ret = -ENOMEM;
			mutex_unlock(&clsic->service_lock);
			goto reterror;
		}

		tmp_handler->start = start;
		tmp_handler->service_instance = service_instance;
		tmp_handler->service_type = service_type;
		clsic->service_handlers[service_instance] = tmp_handler;
	}
	tmp_handler->service_version = service_version;
	mutex_unlock(&clsic->service_lock);

	if (start != NULL)
		ret = (start) (clsic, tmp_handler);

reterror:
	return ret;
}

/*
 * Deregister a service handler - this expects to be called with the same
 * structure that was originally registered
 */
int clsic_deregister_service_handler(struct clsic *clsic,
				     struct clsic_service *handler)
{
	int ret = 0;
	uint8_t servinst = handler->service_instance;

	clsic_dbg(clsic, "%p %d: %pF\n", clsic, servinst, handler->callback);

	if (servinst > CLSIC_SERVICE_MAX) {
		clsic_err(clsic, "%p:%d out of range\n", handler, servinst);
		return -EINVAL;
	}

	mutex_lock(&clsic->service_lock);

	if (clsic->service_handlers[servinst] == NULL) {
		clsic_err(clsic, "%d not registered %p\n", servinst, handler);
		ret = -EINVAL;
	} else if (clsic->service_handlers[servinst] != handler) {
		clsic_err(clsic, "%d not matched %p != %p\n", servinst,
			  handler, clsic->service_handlers[servinst]);
		ret = -EINVAL;
	} else {
		clsic->service_handlers[servinst] = NULL;
	}

	mutex_unlock(&clsic->service_lock);

	return ret;
}

#ifdef CONFIG_PM
static int clsic_pm_service_transition(struct clsic *clsic, int pm_event)
{
	int idx, ret = 0;
	struct clsic_service *tmp_srv;

	for (idx = 0; idx < CLSIC_SERVICE_COUNT; idx++) {
		tmp_srv = clsic->service_handlers[idx];
		if (tmp_srv && tmp_srv->pm_handler) {
			ret = tmp_srv->pm_handler(tmp_srv, pm_event);
			if (ret) {
				/* We fail PM call if any service fails */
				clsic_err(clsic,
					  "service %d, type 0x%x, event %d fail\n",
					  idx, tmp_srv->service_type, pm_event);
				break;
			}
		}
	}

	return ret;
}

/*
 * NOTE: These are quite large timeouts whilst we are in development
 */
#define CLSIC_BOOT_PROGRESS_POLL_MICROSECONDS    5000
#define CLSIC_BOOT_PROGRESS_TIMEOUT_MICROSECONDS 2000000

static int clsic_runtime_resume(struct device *dev)
{
	struct clsic *clsic = dev_get_drvdata(dev);
	bool force_reset = false;
	int ret;
	unsigned int val;

	/* if the driver has halted, don't switch back on */
	if (clsic->state == CLSIC_STATE_HALTED)
		return -EINVAL;

	trace_clsic_pm(RPM_RESUMING);

	clsic_state_set(clsic, CLSIC_STATE_RESUMING,
			CLSIC_STATE_CHANGE_LOCKNOTHELD);

	/*
	 * If VDD_D didn't power off we must force a reset so that the
	 * cache syncs correctly. If we have a hardware reset this must
	 * be done before powering up VDD_D. If not, we'll use a software
	 * reset after powering-up VDD_D
	 */
	if (!clsic->vdd_d_powered_off) {
		clsic_dbg(clsic, "VDD_D did not power off, forcing reset\n");
		force_reset = true;
	}

	ret = regulator_enable(clsic->vdd_d);
	if (ret) {
		clsic_err(clsic, "Failed to enable VDD_D: %d\n", ret);

		/*
		 * not using the clsic_device_error() helper function here as
		 * it makes pm_runtime calls and will get cyclic
		 */
		mutex_lock(&clsic->message_lock);
		clsic_state_set(clsic, CLSIC_STATE_HALTED,
				CLSIC_STATE_CHANGE_LOCKHELD);
		clsic_purge_message_queues(clsic);
		mutex_unlock(&clsic->message_lock);

		return ret;
	}

	if (force_reset) {
		if (clsic->reset_gpio)
			clsic_hard_reset(clsic);
		else
			clsic_soft_reset(clsic);
	}

	reinit_completion(&clsic->pm_completion);

	clsic_irq_enable(clsic);

	if (clsic->volatile_memory) {
		clsic_info(clsic, "Volatile resume\n");
		regmap_read_poll_timeout(clsic->regmap,
			CLSIC_FW_BOOT_PROGRESS, val,
			(val >= CLSIC_FW_BOOT_PROGRESS_READ_FW_UPDATE_BIT),
			CLSIC_BOOT_PROGRESS_POLL_MICROSECONDS,
			CLSIC_BOOT_PROGRESS_TIMEOUT_MICROSECONDS);

		clsic_fwupdate_reset(clsic);
	} else
		schedule_work(&clsic->maintenance_handler);

	/*
	 * Wait for the system to have fully initialised, including any
	 * firmware download and enumeration activity
	 */
	wait_for_completion(&clsic->pm_completion);

	trace_clsic_pm(RPM_ACTIVE);

	return ret;
}

static int clsic_runtime_suspend(struct device *dev)
{
	int ret = 0;
	struct clsic *clsic = dev_get_drvdata(dev);

	trace_clsic_pm(RPM_SUSPENDING);

	if (clsic->state != CLSIC_STATE_HALTED) {
		clsic_state_set(clsic, CLSIC_STATE_SUSPENDING,
				CLSIC_STATE_CHANGE_LOCKNOTHELD);

		/* suspend services */
		ret = clsic_pm_service_transition(clsic, PM_EVENT_SUSPEND);
		if (ret)
			return ret;
	}

	/*
	 * disable IRQ before removing VDD_D, balances with an enable in resume
	 */
	clsic_irq_disable(clsic);

	clsic->vdd_d_powered_off = false;
	regulator_disable(clsic->vdd_d);

	if (clsic->state != CLSIC_STATE_HALTED)
		clsic_state_set(clsic, CLSIC_STATE_OFF,
				CLSIC_STATE_CHANGE_LOCKNOTHELD);

	trace_clsic_pm(RPM_SUSPENDED);

	return 0;
}

const struct dev_pm_ops clsic_pm_ops = {
	SET_RUNTIME_PM_OPS(clsic_runtime_suspend,
			   clsic_runtime_resume,
			   NULL)
};
EXPORT_SYMBOL_GPL(clsic_pm_ops);

#endif

#ifdef CONFIG_DEBUG_FS

/*
 * This method is just so we can trigger the enumeration process that would
 * normally occur when the device raises a bootdone interrupt
 */
static int clsic_bootdone_write(void *data, u64 val)
{
	struct clsic *clsic = data;

	clsic_pm_wake(clsic);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(clsic_bootdone_fops, NULL,
			 clsic_bootdone_write, "%llu\n");

static ssize_t clsic_services_read_file(struct file *file,
					char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct clsic *clsic = file_inode(file)->i_private;
	char *buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	ssize_t len;
	ssize_t ret = 0;
	uint32_t tmp_version, version_maj, version_min, version_bld;
	int i;

	if (buf == NULL)
		return -ENOMEM;

	len = snprintf(buf + ret, PAGE_SIZE - ret,
		       "{\n\t\"format_version\" : 1,\n\t\t\"sysfw\" : {\n\t\t\t\"services\" : [");
	if (len >= 0)
		ret += len;
	if (ret > PAGE_SIZE)
		ret = PAGE_SIZE;

	if (mutex_lock_interruptible(&clsic->service_lock)) {
		kfree(buf);
		return -EINTR;
	}

	for (i = 0; i <= CLSIC_SERVICE_MAX; i++) {
		if (clsic->service_handlers[i] == NULL)
			break;

		tmp_version =
			clsic->service_handlers[i]->service_version;
		version_maj = (tmp_version & CLSIC_SVCVER_MAJ_MASK) >>
			CLSIC_SVCVER_MAJ_SHIFT;
		version_min = (tmp_version & CLSIC_SVCVER_MIN_MASK) >>
			CLSIC_SVCVER_MIN_SHIFT;
		version_bld = (tmp_version & CLSIC_SVCVER_BLD_MASK) >>
			CLSIC_SVCVER_BLD_SHIFT;

		len = snprintf(buf + ret, PAGE_SIZE - ret,
			       "%s\t\t\t\t{ \"instance\" : %d, \"type\" : %d, \"version\" : \"%d.%d.%d\", \"handler\" : \"%pF\" }",
			       (i == 0) ? "\n" : ",\n", i,
			       clsic->service_handlers[i]->service_type,
			       version_maj, version_min, version_bld,
			       clsic->service_handlers[i]->callback);
		if (len >= 0)
			ret += len;
		if (ret > PAGE_SIZE) {
			ret = PAGE_SIZE;
			break;
		}
	}
	len = snprintf(buf + ret, PAGE_SIZE - ret, "\n\t\t\t]\n\t\t}\n}\n");
	if (len >= 0)
		ret += len;
	if (ret > PAGE_SIZE)
		ret = PAGE_SIZE;


	mutex_unlock(&clsic->service_lock);
	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);

	return ret;
}

static const struct file_operations clsic_services_fops = {
	.read = &clsic_services_read_file,
	.llseek = &default_llseek,
};

static ssize_t clsic_state_panic_file(struct file *file,
				      char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct clsic *clsic = file_inode(file)->i_private;
	ssize_t ret = 0;

	ret = simple_read_from_buffer(user_buf, count, ppos,
				      &clsic->last_panic,
				      sizeof(clsic->last_panic));

	return ret;
}

static const struct file_operations clsic_panic_fops = {
	.read = &clsic_state_panic_file,
	.llseek = &default_llseek,
};

#define CLSIC_DEBUGINFO_FILENAME_MAX 50
static const char * const CLSIC_DEBUGINFO_FILENAME_FORMAT =
					       "debug_info-si%d_cat%d_pg%d.bin";

/*
 * The debuginfo read callback obtains the service instance, category and page
 * information from the filename.
 *
 * This is then used to form the message to obtain the required page.
 *
 * The buffer shared back to userspace contains the received message and the
 * associated payload.
 */
static ssize_t clsic_debuginfo_read(struct file *file,
				    char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct clsic *clsic = file_inode(file)->i_private;
	struct dentry *dentry = file_dentry(file);
	ssize_t len = 0;
	union clsic_sys_msg msg_cmd;
	union clsic_sys_msg *msg_rsp;
	char *buf;
	int ret;
	uint8_t service_instance;
	uint16_t tmp_category, tmp_page;

	if (sscanf(dentry->d_name.name, CLSIC_DEBUGINFO_FILENAME_FORMAT,
		   &service_instance, &tmp_category, &tmp_page) != 3)
		return -EINVAL;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	msg_rsp = (union clsic_sys_msg *) buf;

	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   service_instance,
			   CLSIC_GBL_MSG_CR_GET_DEBUG_INFO);

	msg_cmd.cmd_get_debug_info.category = tmp_category;
	msg_cmd.cmd_get_debug_info.page = tmp_page;

	ret = clsic_send_msg_sync_pm(clsic,
				     (union t_clsic_generic_message *) &msg_cmd,
				     (union t_clsic_generic_message *) buf,
				     CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				     buf + CLSIC_FIXED_MSG_SZ,
				     PAGE_SIZE - CLSIC_FIXED_MSG_SZ);

	/*
	 * Check for failures - must have a successful message exchange and the
	 * response must be bulk with no error
	 */
	if (!ret &&
	    (clsic_get_bulk_bit(msg_rsp->blkrsp_get_debug_info.hdr.sbc) == 1) &&
	    (msg_rsp->blkrsp_get_debug_info.hdr.err == 0)) {
		len = simple_read_from_buffer(user_buf, count, ppos, buf,
				    CLSIC_FIXED_MSG_SZ +
				    msg_rsp->blkrsp_get_debug_info.hdr.bulk_sz);
	}

	kfree(buf);
	return len;
}

static const struct file_operations clsic_debuginfo_fops = {
	.read = &clsic_debuginfo_read,
	.llseek = &default_llseek,
};

static void clsic_service_populate_debuginfo(struct clsic *clsic,
					     uint8_t service_instance)
{
	char tmp_filename[CLSIC_DEBUGINFO_FILENAME_MAX];
	uint16_t max_category, tmp_category, tmp_page;
	union clsic_sys_msg msg_cmd;
	union clsic_sys_msg msg_rsp;

	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   service_instance,
			   CLSIC_GBL_MSG_CR_GET_DI_CATEGORY_COUNT);

	if (clsic_send_msg_sync_pm(clsic,
				   (union t_clsic_generic_message *) &msg_cmd,
				   (union t_clsic_generic_message *) &msg_rsp,
				   CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				   CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN))
		return;

	max_category = msg_rsp.rsp_get_di_category_count.category_count;

	for (tmp_category = 0; tmp_category < max_category; tmp_category++) {
		clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
				   service_instance,
				   CLSIC_GBL_MSG_CR_GET_DI_PAGE_COUNT);
		msg_cmd.cmd_get_di_page_count.category = tmp_category;
		if (clsic_send_msg_sync_pm(clsic,
				     (union t_clsic_generic_message *) &msg_cmd,
				     (union t_clsic_generic_message *) &msg_rsp,
				     CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				     CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN))
			return;

		clsic_dbg(clsic, "service %d category %d tmp_page count %d\n",
			  service_instance,
			  msg_rsp.rsp_get_di_page_count.category,
			  msg_rsp.rsp_get_di_page_count.page_count);

		for (tmp_page = 0;
		     tmp_page < msg_rsp.rsp_get_di_page_count.page_count;
		     tmp_page++) {
			snprintf(tmp_filename, sizeof(tmp_filename),
				 CLSIC_DEBUGINFO_FILENAME_FORMAT,
				 service_instance, tmp_category, tmp_page);

			debugfs_create_file(tmp_filename, 0440,
					    clsic->debugfs_debuginfo,
					    clsic, &clsic_debuginfo_fops);
		}
	}
}

/*
 * The read function is a simple indicator as to whether there is a refresh of
 * the debug info pages ongoing.
 */
static int clsic_refresh_debuginfo_read(void *data, u64 *val)
{
	struct clsic *clsic = data;

	if (clsic->debuginfo_refreshing)
		*val = 1;
	else
		*val = 0;

	return 0;
}

/*
 * The write function is simply a trigger to refresh the debuginfo debugfs
 * directory, the actual work can't be performed directly because the context
 * is using debugfs to write to the file and the refresh operation deletes and
 * creates entries that requires debugfs to be idle.
 */
static int clsic_refresh_debuginfo_write(void *data, u64 val)
{
	struct clsic *clsic = data;

	clsic->debuginfo_refreshing = true;
	schedule_work(&clsic->refresh_debuginfo);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(clsic_refresh_debuginfo_fops,
			 clsic_refresh_debuginfo_read,
			 clsic_refresh_debuginfo_write, "%llu\n");

static void clsic_refresh_debuginfo(struct work_struct *data)
{
	struct clsic *clsic = container_of(data, struct clsic,
					   refresh_debuginfo);
	int i;

	debugfs_remove_recursive(clsic->debugfs_debuginfo);
	clsic->debugfs_debuginfo = debugfs_create_dir("debuginfo",
						      clsic->debugfs_root);

	/*
	 * For each of the discovered service instances, populate the debug
	 * info, as there are no holes in the service map stop when a NULL
	 * service_handler is encountered as there will be no more services to
	 * update.
	 */
	for (i = 0; i < CLSIC_SERVICE_COUNT; i++) {
		if (clsic->service_handlers[i] == NULL)
			break;

		if (clsic->service_handlers[i]->supports_debuginfo)
			clsic_service_populate_debuginfo(clsic, i);
	}

	clsic->debuginfo_refreshing = false;
}

/* 13 as the name will be at most "clsic-nnn" + \0 */
#define CLSIC_DEBUGFS_DIRNAME_MAX		13
void clsic_init_debugfs(struct clsic *clsic)
{
	char dirname[CLSIC_DEBUGFS_DIRNAME_MAX];

	if (clsic->instance == 0)
		strlcpy(dirname, "clsic", sizeof(dirname));
	else
		snprintf(dirname, sizeof(dirname), "clsic-%d", clsic->instance);

	clsic->debugfs_root = debugfs_create_dir(dirname, NULL);
	if (clsic->debugfs_root == NULL) {
		clsic_err(clsic, "Failed to create debugfs dir\n");
		return;
	}

	debugfs_create_file("bootdone", 0220, clsic->debugfs_root, clsic,
			    &clsic_bootdone_fops);

	debugfs_create_file("services", 0444, clsic->debugfs_root, clsic,
			    &clsic_services_fops);

	debugfs_create_file("last_panic", 0440, clsic->debugfs_root, clsic,
			    &clsic_panic_fops);

	clsic->debugfs_debuginfo = debugfs_create_dir("debuginfo",
						      clsic->debugfs_root);

	INIT_WORK(&clsic->refresh_debuginfo, clsic_refresh_debuginfo);
	clsic->debuginfo_refreshing = false;
	debugfs_create_file("refresh_debuginfo", 0200, clsic->debugfs_root,
			    clsic, &clsic_refresh_debuginfo_fops);
}

void clsic_deinit_debugfs(struct clsic *clsic)
{
	debugfs_remove_recursive(clsic->debugfs_root);

	cancel_work_sync(&clsic->refresh_debuginfo);
	clsic->debugfs_root = NULL;
}

#else /* ifdef CONFIG_DEBUG_FS */

void clsic_init_debugfs(struct clsic *clsic)
{
}

void clsic_deinit_debugfs(struct clsic *clsic)
{
}

#endif

static ssize_t clsic_store_state(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct clsic *clsic = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset"))) {
		/* Debug control prevents device state changes */
		if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED)
			return -EPERM;

		clsic_info(clsic, "Reset requested\n");

		if (clsic->state == CLSIC_STATE_HALTED) {
			clsic_info(clsic, "Released from HALTED state\n");
			clsic_state_set(clsic, CLSIC_STATE_OFF,
					CLSIC_STATE_CHANGE_LOCKNOTHELD);
		}

		pm_runtime_suspend(clsic->dev);

		if (clsic_wait_for_state(clsic, CLSIC_STATE_OFF,
				 CLSIC_WAIT_FOR_STATE_MAX_CYCLES,
				 CLSIC_WAIT_FOR_STATE_DELAY_MS))
			clsic_info(clsic, "Warning: state is %s\n",
				   clsic_state_to_string(clsic->state));

		mutex_lock(&clsic->message_lock);
		clsic->service_states = CLSIC_REENUMERATION_REQUIRED;
		clsic_purge_message_queues(clsic);
		mutex_unlock(&clsic->message_lock);

		clsic_pm_wake(clsic);
	} else if (!strncmp(buf, "msgprochold", strlen("msgprochold"))) {
		/*
		 * Mark the messaging processor as used by the system service.
		 *
		 * This is useful when debugging as the normal power management
		 * will switch off the device when it is deemed idle.
		 *
		 * This is not reference counted.
		 */
		clsic_info(clsic,
			   "Marking messaging processor as used by system service.\n");
		clsic_msgproc_use(clsic, CLSIC_SRV_INST_SYS);
	} else if (!strncmp(buf, "msgprocrelease", strlen("msgprocrelease"))) {
		/*
		 * Clear the mark of the messaging processor being used by the
		 * system service
		 *
		 * This will allow normal power management to resume.
		 *
		 * This is not reference counted.
		 */
		clsic_info(clsic,
			   "Clearing messaging processor as used by system service mark.\n");
		clsic_msgproc_release(clsic, CLSIC_SRV_INST_SYS);
	}

	return count;
}

static ssize_t clsic_show_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct clsic *clsic = dev_get_drvdata(dev);
	enum clsic_blrequests saved_blrequest;

	mutex_lock(&clsic->message_lock);
	saved_blrequest = clsic->blrequest;
	mutex_unlock(&clsic->message_lock);

	if (saved_blrequest != CLSIC_BL_IDLE) {
		switch (saved_blrequest) {
		case CLSIC_BL_EXPECTED:
			return snprintf(buf, PAGE_SIZE, "BOOTLOADER_EXPECTED");
		case CLSIC_BL_UPDATE:
			return snprintf(buf, PAGE_SIZE, "BOOTLOADER_UPDATE");
		case CLSIC_BL_FWU:
			return snprintf(buf, PAGE_SIZE, "BOOTLOADER_FWU");
		case CLSIC_BL_CPK:
			return snprintf(buf, PAGE_SIZE, "BOOTLOADER_CPK");
		case CLSIC_BL_MAB:
			return snprintf(buf, PAGE_SIZE, "BOOTLOADER_MAB");
		case CLSIC_BL_IDLE: /* required for a switch werror */
			return snprintf(buf, PAGE_SIZE, "BOOTLOADER_IDLE");
		}
	}

	return snprintf(buf, PAGE_SIZE, "%s\n",
			clsic_state_to_string(clsic->state));
}
static DEVICE_ATTR(state, 0644,
		   clsic_show_state, clsic_store_state);

static ssize_t clsic_show_devid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct clsic *clsic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", clsic->devid);
}
static DEVICE_ATTR(devid, 0444, clsic_show_devid, NULL);

static void clsic_init_sysfs(struct clsic *clsic)
{
	device_create_file(clsic->dev, &dev_attr_devid);
	device_create_file(clsic->dev, &dev_attr_state);
}

static void clsic_deinit_sysfs(struct clsic *clsic)
{
	device_remove_file(clsic->dev, &dev_attr_devid);
	device_remove_file(clsic->dev, &dev_attr_state);
}

MODULE_DESCRIPTION("CLSIC MFD");
MODULE_AUTHOR("Simon Trimmer <simont@opensource.cirrus.com>");
MODULE_LICENSE("GPL v2");
