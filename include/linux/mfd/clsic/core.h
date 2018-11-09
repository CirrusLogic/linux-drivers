/*
 * core.h -- CLSIC core definitions
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CIRRUS_CLSIC_CORE_H
#define CIRRUS_CLSIC_CORE_H
#include <linux/mfd/core.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/pm_runtime.h>

#ifndef PACKED
#define PACKED __packed
#endif
#include <linux/mfd/clsic/clsicmessagedefines.h>

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/kconfig.h>
#include <linux/regmap.h>
#include <linux/notifier.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <linux/mfd/clsic/registers.h>

#define clsic_dbg(_clsic, fmt, ...) \
	dev_dbg(_clsic->dev, "%s() " fmt, __func__, ##__VA_ARGS__)
#define clsic_info(_clsic, fmt, ...) \
	dev_info(_clsic->dev, "%s() " fmt, __func__, ##__VA_ARGS__)
#define clsic_warn(_clsic, fmt, ...) \
	dev_warn(_clsic->dev, "%s() " fmt, __func__, ##__VA_ARGS__)
#define clsic_err(_clsic, fmt, ...) \
	dev_err(_clsic->dev, "%s() " fmt, __func__, ##__VA_ARGS__)

extern const struct of_device_id clsic_of_match[];
extern const struct dev_pm_ops clsic_pm_ops;

#define CLSIC_SUPPORTED_ID_48AB50		0x48AB50
#define CLSIC_SUPPORTED_ID_EMULATED_CODEC	0xF48AB50
#define CLSIC_SUPPORTED_ID_48AC40		0x48AC40

#define CLSIC_SERVICE_TYPE_DEBUG_EMU		0x1E

#define CLSIC_SERVICE_COUNT			32	/* 0 to 31 */
#define CLSIC_SERVICE_MAX			(CLSIC_SERVICE_COUNT - 1)
#define CLSIC_SERVICE_RESERVED			CLSIC_SERVICE_COUNT

#define CLSIC_MAX_CORE_SUPPLIES			2

enum clsic_states {
	CLSIC_STATE_OFF = 0,
	CLSIC_STATE_RESUMING,
	CLSIC_STATE_ON,
	CLSIC_STATE_SUSPENDING,
	CLSIC_STATE_DEBUGCONTROL_REQUESTED,
	CLSIC_STATE_DEBUGCONTROL_GRANTED,
	CLSIC_STATE_HALTED,
};

static inline const char *clsic_state_to_string(enum clsic_states state)
{
	switch (state) {
	case CLSIC_STATE_OFF:
		return "OFF";
	case CLSIC_STATE_RESUMING:
		return "RESUMING";
	case CLSIC_STATE_ON:
		return "ON";
	case CLSIC_STATE_SUSPENDING:
		return "SUSPENDING";
	case CLSIC_STATE_DEBUGCONTROL_REQUESTED:
		return "DEBUGCONTROL_REQUESTED";
	case CLSIC_STATE_DEBUGCONTROL_GRANTED:
		return "DEBUGCONTROL_GRANTED";
	case CLSIC_STATE_HALTED:
		return "HALTED";
	default:
		return "UNKNOWN";
	}
}

struct clsic_panic {
	uint8_t msg[CLSIC_FIXED_MSG_SZ];
	struct clsic_debug_info di;
};

#ifdef CONFIG_DEBUG_FS
enum clsic_simirq_state {
	CLSIC_SIMIRQ_STATE_DEASSERTED = 0,
	CLSIC_SIMIRQ_STATE_ASSERTED = 1
};
#endif

enum clsic_blrequests {
	CLSIC_BL_IDLE = 0,
	CLSIC_BL_UPDATE,
	CLSIC_BL_EXPECTED,
	CLSIC_BL_FWU,
	CLSIC_BL_CPK,
	CLSIC_BL_MAB,
};

/*
 * These states represent the power state and general availability of the
 * messaging processor in the device, if a message has been sent or received
 * the processor is presumed ON (and is a candidate for being shutdown based on
 * idle time).
 */
enum clsic_msgproc_states {
	CLSIC_MSGPROC_OFF = 0,
	CLSIC_MSGPROC_ON,
};

enum clsic_service_states {
	CLSIC_ENUMERATION_REQUIRED = 0,
	CLSIC_REENUMERATION_REQUIRED,
	CLSIC_ENUMERATED,
	CLSIC_UNLOADING,
};

struct clsic {
	struct regmap *regmap;

	struct device *dev;
	uint32_t devid;

	/*
	 * Devices that have volatile memory need to have their firmware
	 * downloaded on every reset or power on.
	 */
	bool volatile_memory;

	int irq;

	uint8_t instance; /* instance number */
	enum clsic_states state;
	enum clsic_msgproc_states msgproc_state;

	enum clsic_blrequests blrequest;

	struct notifier_block clsic_shutdown_notifier;

	/*
	 * Location of the FIFO TX register
	 *
	 * Set to one of:
	 * CLSIC_SCP_TX_SPI
	 * CLSIC_SCP_TX_SLIMBUS
	 * CLSIC_SCP_TX_SOUNDWIRE
	 */
	uint32_t fifo_tx;

	/*
	 * This handler takes over the booting and enumeration of the system
	 * and servicing requests from the device bootloader.
	 *
	 * It has a brief lifespan and uses the shared workqueue
	 */
	struct work_struct maintenance_handler;

	/* The message layer has it's own workqueue as it is long lived */
	struct workqueue_struct *message_worker_queue;
	struct work_struct message_work;
	struct timer_list workerthread_timer;

	/*
	 * Number of times the worker thread had nothing to do on this message.
	 * This value is updated AFTER the timer runs and is measured in
	 * seconds.
	 */
	uint8_t timeout_counter;

	/*
	 * Informational counters indicating how many messages have been sent
	 * and received on the message bus
	 */
	uint32_t messages_sent;
	uint32_t messages_received;

	/* To be held whilst manipulating message queues */
	struct mutex message_lock;

	/*
	 * To be held whilst manipulating services and calling service handler
	 */
	struct mutex service_lock;

	/* Slab cache of messages */
	struct kmem_cache *message_cache;

	/*
	 * Single pointer to the message currently blocking the bus,
	 * if this is NULL then the bus is available
	 */
	struct clsic_message *current_msg;
	/* List of messages that are blocked waiting to send */
	struct list_head waiting_to_send;
	/* List of messages sent and/or ack'd but not completed */
	struct list_head waiting_for_response;
	/* List of messages completed but not released */
	struct list_head completed_messages;

	/*
	 * Service enumeration is only required once per boot or if the
	 * firmware changes
	 */
	enum clsic_service_states service_states;

	/* Array of pointers to service handlers */
	struct list_head inactive_services;
	struct clsic_service *service_handlers[CLSIC_SERVICE_COUNT];

	/* Pre-allocated area for a panic message and debug info payload */
	struct clsic_panic last_panic;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
	struct dentry *debugfs_debuginfo;
	struct work_struct refresh_debuginfo;
	bool debuginfo_refreshing;

	/* Debugcontrol members protected by message_lock */
	struct completion *debugcontrol_completion;

	/* Simulated IRQ
	 * Represent interrupt enabled status and asserted state
	 * Supporting timer and work structure.
	 */
	bool simirq_enabled;
	enum clsic_simirq_state simirq_state;
	struct work_struct simirq_work;
	struct timer_list simirq_timer;
#endif

	struct gpio_desc *reset_gpio;

	/* PM related members */
	int num_core_supplies;
	struct regulator_bulk_data core_supplies[CLSIC_MAX_CORE_SUPPLIES];
	struct regulator *vdd_d;
	struct notifier_block vdd_d_notifier;
	bool vdd_d_powered_off;
	struct completion pm_completion;

	struct delayed_work clsic_msgproc_shutdown_work;
	DECLARE_BITMAP(clsic_services_state, CLSIC_SERVICE_COUNT);
};

int clsic_dev_init(struct clsic *clsic);
int clsic_dev_exit(struct clsic *clsic);
int clsic_fwupdate_reset(struct clsic *clsic);
void clsic_soft_reset(struct clsic *clsic);
void clsic_dev_panic(struct clsic *clsic, struct clsic_message *msg);
void clsic_maintenance(struct work_struct *data);

/*
 * This service struct contains instance specific information about a service
 * handler.
 *
 * It is allocated by a service and passed during register_service_handler
 */
struct clsic_service {
	int (*callback)(struct clsic *clsic,
			struct clsic_service *handler,
			struct clsic_message *msg);

	/*
	 * start()
	 * can send messages
	 * cannot take the service_lock
	 */
	int (*start)(struct clsic *clsic, struct clsic_service *handler);
	/*
	 * stop()
	 * can send messages
	 * can take the service_lock
	 */
	void (*stop)(struct clsic *clsic, struct clsic_service *handler);

	uint8_t service_instance;
	uint16_t service_type;
	uint32_t service_version;
	bool supports_debuginfo;
	struct list_head link;
	bool mfd_loaded;

	/* A pointer the handler can use to stash instance specific stuff */
	void *data;

	/* service specific PM handler */
	int (*pm_handler)(struct clsic_service *handler, int pm_event);
};

int clsic_update_service(struct clsic *clsic,
			 uint8_t service_instance,
			 uint16_t service_type,
			 uint32_t service_version);

void clsic_suspend_services_from(struct clsic *clsic,
				 uint8_t service_instance);

#define CLSIC_STATE_CHANGE_CHECK true
#define CLSIC_STATE_CHANGE_NOCHECK false
#define CLSIC_STATE_CHANGE_LOCKHELD true
#define CLSIC_STATE_CHANGE_LOCKNOTHELD false
void clsic_state_set(struct clsic *clsic,
		     const enum clsic_states new_state,
		     bool lock_held);

static inline const char *clsic_pm_rpm_to_string(int event)
{
	switch (event) {
	case RPM_ACTIVE:
		return "ACTIVE";
	case RPM_RESUMING:
		return "RESUMING";
	case RPM_SUSPENDED:
		return "SUSPENDED";
	case RPM_SUSPENDING:
		return "SUSPENDING";
	default:
		return "UNKNOWN";
	}
}

/* Cause the device to power up (until it auto suspends again) */
static inline void clsic_pm_wake(struct clsic *clsic)
{
	pm_runtime_get_sync(clsic->dev);
	pm_runtime_mark_last_busy(clsic->dev);
	pm_runtime_put_autosuspend(clsic->dev);
}

void clsic_init_debugfs(struct clsic *clsic);
void clsic_deinit_debugfs(struct clsic *clsic);

/* in Tables */
bool clsic_readable_register(struct device *dev, unsigned int reg);
bool clsic_volatile_register(struct device *dev, unsigned int reg);
extern const struct reg_default clsic_reg_defaults[];
#define CLSIC_REG_DEFAULTS_SZ	656
extern const struct regmap_config clsic_regmap_config_dsp2_template;

/*
 * Simple utility function that pauses until the clsic->state becomes state, if
 * it doesn't become that state it returns true else it returns false.
 *
 * The delay_ms is how long to pause (in milliseconds) when the state doesn't
 * match and the max_cycles is how many times the state should be examined.
 *
 * e.g. max_cycles=10, delay_ms=100 will wait between 0 and 1 seconds, added
 * these figures as constants for now
 */
#define CLSIC_WAIT_FOR_STATE_MAX_CYCLES	10
#define CLSIC_WAIT_FOR_STATE_DELAY_MS	100
static inline bool clsic_wait_for_state(struct clsic *clsic,
					enum clsic_states state,
					int max_cycles, int delay_ms)
{
	int i;
	enum clsic_states saved_state;

	for (i = 0; i < max_cycles; i++) {
		saved_state = clsic->state;
		if (saved_state == state)
			return false;
		clsic_info(clsic, "Pausing (%d != %d)\n", saved_state, state);
		msleep(delay_ms);
	}
	return true;
}

/*
 * if the device misbehaves then this function is called to bring the device to
 * an inert state
 */
#define CLSIC_DEVICE_ERROR_LOCKHELD true
#define CLSIC_DEVICE_ERROR_LOCKNOTHELD false
void clsic_purge_message_queues(struct clsic *clsic);
static inline void clsic_device_error(struct clsic *clsic,
				      bool lock_held)
{
	clsic_state_set(clsic, CLSIC_STATE_HALTED,
			lock_held);

	pm_runtime_set_suspended(clsic->dev);

	if (!lock_held)
		mutex_lock(&clsic->message_lock);

	clsic_purge_message_queues(clsic);
	if (!lock_held)
		mutex_unlock(&clsic->message_lock);
}

#endif
