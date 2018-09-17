/*
 * debugcontrol.h -- CLSIC debugcontrol constant definitions
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CIRRUS_CLSIC_DEBUGCONTROL_H
#define CIRRUS_CLSIC_DEBUGCONTROL_H

/**
 * These constants are intended to be shared between the core driver and
 * Cirrus Logic tools infrastructure to control the operation of the
 * debugcontrol mechanism.
 *
 * This mechanism provides a method for Cirrus Logic tooling to gain exclusive
 * access to a device without normal driver operations occurring, such as
 * consuming ACKs and notifications. This enables test infrastructure to
 * interact with the device at the message protocol level.
 *
 * The mechanism is accessed through an instanced debugfs node, for example;
 *
 * /sys/kernel/debug/clsic/debugcontrol
 *
 * Reading this file returns one of the enumerations below indicating the
 * driver state.
 *
 * It is legal for the tooling to write to this file to request that the driver
 * disable normal access (writing CLSIC_DEBUGCONTROL_REQUESTED) and to attempt
 * to re-enable normal access (CLSIC_DEBUGCONTROL_RELEASED).
 *
 * Re-enabling normal device operation (using CLSIC_DEBUGCONTROL_RELEASED) is
 * permitted, but as it is possible that software state no-longer matches the
 * hardware it is safest to reboot the device.
 *
 * The write method will return when the request has been completed and the
 * return errno for the write will indicate general success or failure, however
 * when releasing a granted request the device may still be left in an unusable
 * state.
 *
 * This interface is not exclusive and concurrent access must be managed by the
 * caller.
 */
enum clsic_debugcontrol_states {
	/**
	 * This state indicates that the debugcontrol mechanism is not engaged
	 * and that the driver is in normal operation.
	 *
	 * Writing this value to the debugcontrol interface will;
	 * - do nothing if debugcontrol is not requested or granted
	 * - interrupt an outstanding request
	 * - attempt to release a granted debugcontrol request if policy permits
	 */
	CLSIC_DEBUGCONTROL_RELEASED = 0,

	/**
	 * The requested state indicates that the debugcontrol mechanism has
	 * been requested but has not yet been granted. This would either be
	 * because the worker thread has not yet run or if there is a message
	 * outstanding that has not been ACKd.
	 *
	 * Writing this value to the debugcontrol interface will;
	 * - grant debugcontrol if the messaging layer has no currently active
	 *   message
	 * - return -EBUSY if there is already a debugcontrol request
	 * - return -EBUSY if debugcontrol has already been granted
	 * - wait until debugcontrol can be granted if there is an active
	 *   message, in this state the request can be interrupted by
	 *   signalling the requesting thread or another thread writing
	 *   CLSIC_DEBUGCONTROL_RELEASED. When interrupted the request will
	 *   return -ERESTARTSYS
	 */
	CLSIC_DEBUGCONTROL_REQUESTED = 1,

	/**
	 * The granted state indicates that the debugcontrol mechanism has
	 * been successfully engaged and the messaging layer will no longer send
	 * messages on the bus nor consume messages in IRQs.
	 *
	 * Writing this value to the debugcontrol interface is a client error
	 * and will be ignored.
	 */
	CLSIC_DEBUGCONTROL_GRANTED = 2,
};

#endif
