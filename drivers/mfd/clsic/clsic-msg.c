/*
 * clsic-msg.c -- CLSIC message interface
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/mfd/clsic/core.h>
#include "clsic-trace.h"
#include <linux/mfd/clsic/message.h>
#include <linux/mfd/clsic/irq.h>
#include <linux/mfd/clsic/debugcontrol.h>
#include <linux/mfd/clsic/syssrv.h>
#include <linux/mfd/clsic/clsicmessagedefines_BLD.h>

/*
 * When there are messages in the queues (waiting for response or waiting to
 * send) the messaging layer has a timer running to trigger the worker thread
 * to detect message protocol timeouts.
 *
 * The period of that timer and the length of time before a message times out
 * are measured in seconds.
 *
 * These values can be easily tuned so that the worker thread runs each second
 * or runs at a much higher granularity (e.g. the timeout period of a message)
 *
 * CLSIC_WORKERTHREAD_TIMER_PERIOD	1
 * CLSIC_WORKERTHREAD_TIMER_PERIOD	CLSIC_MSG_TIMEOUT_PERIOD
 *
 * Bootloader messages perform flash operations and can take considerably
 * longer than a normal message to complete, they have a different timeout
 * value.
 */
#define CLSIC_MSG_TIMEOUT_PERIOD 30
#define CLSIC_MSG_TIMEOUT_PERIOD_BOOTLOADER 90
#define CLSIC_WORKERTHREAD_TIMER_PERIOD 1

#define CLSIC_WORKERTHREAD_NAME "clsic_worker"
static void clsic_message_worker(struct work_struct *data);

#define CLSIC_MSGPROC_SHUTDOWN_TIMEOUT 10

/*
 * Buffer size required to hold biggest msg2message string
 *
 * 11 for "%p : "
 * 5 for message state (1 or 2 digits) and space "s:XX "
 * 5 for timeout (1 or 2 digits) and space "t:XX "
 * 12 for "[%d %c %c %d]" as msgid and servinst have a max of 3 and 2 digits
 * 36 for 12 fsm bytes in hex with a space each
 * 2 for a newline and a null terminator
 * = 71 bytes
 */
#define CLSIC_MSG2STRING_BUFLEN 71

/*
 * Decode a given message into a given buffer
 *
 * The fixed sized message (fsm) is specified separately as a message structure
 * may contain both a message FSM and a response FSM.
 */
static int clsic_msg2string(char *buf, const int space,
			    const struct clsic_message *msg,
			    const uint8_t *fsm)
{
	struct clsic_cmd_hdr *hdr =
		(struct clsic_cmd_hdr *) fsm;

	return snprintf(buf, space,
			"%p : s:%d t:%d [%d %c %c %d] %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			msg,
			CLSIC_GET_MSGSTATE(msg),
			msg->timeout,
			clsic_get_srv_inst(hdr->sbc),
			clsic_get_bulk_bit(hdr->sbc) == 1 ? 'b' : 'f',
			clsic_cran_to_char(clsic_get_cran(hdr->sbc)),
			hdr->msgid,
			fsm[CLSIC_FSM0], fsm[CLSIC_FSM1],
			fsm[CLSIC_FSM2], fsm[CLSIC_FSM3],
			fsm[CLSIC_FSM4], fsm[CLSIC_FSM5],
			fsm[CLSIC_FSM6], fsm[CLSIC_FSM7],
			fsm[CLSIC_FSM8], fsm[CLSIC_FSM9],
			fsm[CLSIC_FSM10], fsm[CLSIC_FSM11]);
}

void clsic_dump_message(const struct clsic *clsic,
			const struct clsic_message *msg, const char *string)
{
	char buffer[CLSIC_MSG2STRING_BUFLEN];

	clsic_msg2string(buffer, CLSIC_MSG2STRING_BUFLEN, msg, msg->fsm.raw);
	dev_info(clsic->dev, "%s %s", string, buffer);

	/*
	 * The response is zero initialised which equates to a CRAN of command,
	 * so if the CRAN is set to something other than a command then include
	 * the response buffer.
	 */
	if (clsic_get_cran(msg->response.cmd.hdr.sbc) != CLSIC_CRAN_CMD) {
		clsic_msg2string(buffer, CLSIC_MSG2STRING_BUFLEN, msg,
				 msg->response.raw);
		dev_info(clsic->dev, "%s %s", string, buffer);
	}

	/* if this is a bulk message request print the bulk metadata */
	if ((msg->bulk_txbuf != NULL) || (msg->bulk_rxbuf != NULL)) {
		dev_info(clsic->dev,
			 "%s Msg %p (%d): tx %p %p %d rx %p %d\n",
			 string,
			 msg,
			 msg->state,
			 msg->bulk_txbuf,
			 msg->bulk_txbuf_marker,
			 msg->bulk_txbuf_maxsize,
			 msg->bulk_rxbuf,
			 msg->bulk_rxbuf_maxsize);
	}
}

/*
 * Utility function to allocate a message structure.
 */
static struct clsic_message *clsic_allocate_msg(struct clsic *clsic)
{
	struct clsic_message *msg;

	msg = kmem_cache_zalloc(clsic->message_cache, GFP_KERNEL);
	if (msg == NULL)
		return NULL;

	INIT_LIST_HEAD(&msg->client_link);
	INIT_LIST_HEAD(&msg->private_link);
	init_completion(&msg->completion);

	return msg;
}

/*
 * Utility function to dispose of a message structure.
 */
void clsic_release_msg(struct clsic *clsic, struct clsic_message *msg)
{
	list_del(&msg->private_link);

	/*
	 * If the message is on a client list then this structure (and related
	 * memory) could be being used, BUT there is a good chance this message
	 * is being deleted as the driver is being shutdown and the driver
	 * cannot destroy the cache with messages outstanding.
	 */
	if (!list_empty(&msg->client_link)) {
		clsic_err(clsic, "Warning: message %p client_link set", msg);
		clsic_dump_message(clsic, msg, "client_link set");
	}

	kmem_cache_free(clsic->message_cache, msg);
}

/*
 * Simple function to assign a new state to a message and issue a matching
 * trace event.
 *
 * The clsic->message_lock is expected to be held when calling this function
 * once the message has been submitted to the messaging layer for processing.
 */
static inline void clsic_set_msgstate(struct clsic_message *msg,
				      const enum clsic_message_states newstate)
{
	msg->state = newstate;
	trace_clsic_msg_statechange(msg);
}

/*
 * This function must be called with the clsic->message_lock held
 */
static void clsic_unlink_message(struct clsic *clsic,
				 struct clsic_message *msg_p)
{
	if (msg_p == clsic->current_msg) {
		clsic->current_msg = NULL;

		/*
		 * Clearing the current_msg unblocks the messaging bus for
		 * commands to be sent - if there are messages waiting to be
		 * sent signal the worker thread to be run.
		 */
		if (!list_empty(&clsic->waiting_to_send))
			queue_work(clsic->message_worker_queue,
				   &clsic->message_work);
	}
	list_del(&msg_p->private_link);
}

/*
 * This function must be called with the clsic->message_lock held
 */
static void clsic_complete_message_core(struct clsic *clsic,
					struct clsic_message *msg_p)
{
	bool isa_shutdown_message = false;

	clsic_unlink_message(clsic, msg_p);
	list_add_tail(&msg_p->private_link, &clsic->completed_messages);

	/* Test whether this is a shutdown message before it is released */
	if ((clsic_get_servinst(msg_p) == CLSIC_SRV_INST_SYS) &&
	    (clsic_get_messageid(msg_p) == CLSIC_SYS_MSG_CR_SP_SHDN)) {
		isa_shutdown_message = true;
		clsic->msgproc_state = CLSIC_MSGPROC_OFF;
	}

	/*
	 * Perform the asynchronous callback - if the callback does not keep
	 * the message then release it
	 */
	if (msg_p->cb(clsic, msg_p) != CLSIC_MSG_RETAINED)
		clsic_release_msg(clsic, msg_p);

	/*
	 * Messaging becomes idle when the send and response lists are empty,
	 * schedule sending a shutdown message if the message being processed
	 * is not a shutdown message.
	 */
	if (list_empty(&clsic->waiting_to_send) &&
	    list_empty(&clsic->waiting_for_response) &&
	    !isa_shutdown_message)
		clsic_msgproc_shutdown_schedule(clsic);
}

/*
 * This function must be called with the clsic->message_lock held
 */
static void clsic_complete_message(struct clsic *clsic,
				   struct clsic_message *msg_p,
				   enum clsic_message_states state)
{
	/*
	 * This function is called in error cases on the current message, which
	 * may be empty
	 */
	if (msg_p == NULL)
		return;

	clsic_set_msgstate(msg_p, state);

	clsic_complete_message_core(clsic, msg_p);
}

#ifdef CONFIG_DEBUG_FS

/*
 * The messages file in debugfs prints the current state of the messaging
 * queues along with some additional statistics.
 */
static ssize_t clsic_messages_read_file(struct file *file,
					char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct clsic *clsic = file_inode(file)->i_private;
	char *buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	ssize_t len;
	ssize_t ret = 0;
	struct clsic_message *tmp_msg;
	struct clsic_message *next_msg;

	if (buf == NULL)
		return -ENOMEM;

	/* populate header */
	len = snprintf(buf + ret, PAGE_SIZE - ret,
		       "Sent: %d Received: %d (msgproc: %d, state: %s : %d, usage_count: %d)\nWaiting to send:\n",
		       clsic->messages_sent,
		       clsic->messages_received,
		       clsic->msgproc_state,
		       clsic_state_to_string(clsic->state),
		       clsic->blrequest,
		       atomic_read(&clsic->dev->power.usage_count));
	if (len >= 0)
		ret += len;

	if (mutex_lock_interruptible(&clsic->message_lock)) {
		kfree(buf);
		return -EINTR;
	}

	/* dump waiting to send list */
	if (!list_empty(&clsic->waiting_to_send)) {
		list_for_each_entry_safe(tmp_msg,
					 next_msg,
					 &clsic->waiting_to_send,
					 private_link) {
			len = clsic_msg2string(buf + ret, PAGE_SIZE - ret,
					       tmp_msg, tmp_msg->fsm.raw);
			if (len >= 0)
				ret += len;
			if (ret > PAGE_SIZE) {
				ret = PAGE_SIZE;
				break;
			}
		}
	}

	/* dump current message details */
	len = snprintf(buf + ret, PAGE_SIZE - ret,
		       "Current Message: (Timer: %d)\n",
		       clsic->timeout_counter);
	if (len >= 0)
		ret += len;
	if (ret > PAGE_SIZE)
		ret = PAGE_SIZE;

	if (clsic->current_msg != NULL) {
		tmp_msg = clsic->current_msg;
		len = clsic_msg2string(buf + ret, PAGE_SIZE - ret,
				       tmp_msg, tmp_msg->fsm.raw);
		if (len >= 0)
			ret += len;
		if (ret > PAGE_SIZE)
			ret = PAGE_SIZE;
	}

	/* waiting for response header */
	len = snprintf(buf + ret, PAGE_SIZE - ret, "Waiting for response:\n");
	if (len >= 0)
		ret += len;
	if (ret > PAGE_SIZE)
		ret = PAGE_SIZE;

	/* dump waiting for response list */
	if (!list_empty(&clsic->waiting_for_response)) {
		list_for_each_entry_safe(tmp_msg,
					 next_msg,
					 &clsic->waiting_for_response,
					 private_link) {
			len = clsic_msg2string(buf + ret, PAGE_SIZE - ret,
					       tmp_msg, tmp_msg->fsm.raw);
			if (len >= 0)
				ret += len;
			if (ret > PAGE_SIZE)
				ret = PAGE_SIZE;
		}
	}

	/* Completed header */
	len = snprintf(buf + ret, PAGE_SIZE - ret, "Completed messages:\n");
	if (len >= 0)
		ret += len;
	if (ret > PAGE_SIZE)
		ret = PAGE_SIZE;

	/* dump completed message list */
	if (!list_empty(&clsic->completed_messages)) {
		list_for_each_entry_safe(tmp_msg,
					 next_msg,
					 &clsic->completed_messages,
					 private_link) {
			len = clsic_msg2string(buf + ret, PAGE_SIZE - ret,
					       tmp_msg, tmp_msg->fsm.raw);
			if (len >= 0)
				ret += len;
			if (ret > PAGE_SIZE)
				ret = PAGE_SIZE;
		}
	}


	/* unlock messaging layer and copy back */
	mutex_unlock(&clsic->message_lock);
	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);

	return ret;
}

static const struct file_operations clsic_messages_fops = {
	.read = &clsic_messages_read_file,
	.llseek = &default_llseek,
};

/*
 * The custom message interface in debugfs allows messages to be sent and
 * responses to be received by test programs, it is not intended to be an
 * interface used in production devices.
 *
 * Transfers are currently limited to at most PAGE_SIZE, for both the fixed
 * size message and the payload.
 */
#define CLSIC_CUSTOM_MESSAGE_MAX	PAGE_SIZE
struct clsic_custom_message_struct {
	ssize_t len;
	uint8_t buf[CLSIC_CUSTOM_MESSAGE_MAX];
};

static int clsic_custom_message_open(struct inode *inode, struct file *file)
{
	if (file->private_data == NULL) {
		file->private_data = kzalloc
			(sizeof(struct clsic_custom_message_struct),
			 GFP_KERNEL);
		if (file->private_data == NULL)
			return -ENOMEM;
	}

	return 0;
}

static int clsic_custom_message_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	file->private_data = NULL;

	return 0;
}

/* Let user space read back a message sent from the device to the host. */
static ssize_t clsic_custom_message_read(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct clsic *clsic = file_inode(file)->i_private;
	struct clsic_custom_message_struct *custom_msg = file->private_data;
	ssize_t ret;

	if (custom_msg == NULL)
		return -ENOMEM;

	if (custom_msg->buf == NULL)
		return -ENOMEM;

	ret = simple_read_from_buffer(user_buf, count, ppos,
				      custom_msg->buf, custom_msg->len);

	if (ret < 0)
		clsic_err(clsic, "simple_read_from_buffer error %zd", ret);

	return ret;
}

/* Send a custom fixed size message from host user space to the device. */
static ssize_t clsic_custom_message_write(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct clsic *clsic = file_inode(file)->i_private;
	struct clsic_custom_message_struct *custom_msg = file->private_data;
	char *buf;
	ssize_t ret;
	union t_clsic_generic_message *msg_p;
	char *txbuf = NULL;
	ssize_t txcount = 0;
	char *rxbuf;
	ssize_t rxcount;

	if (custom_msg == NULL)
		return -ENOMEM;

	buf = custom_msg->buf;
	if (buf == NULL)
		return -ENOMEM;

	/* The whole message must be written by the client in one go */
	if (count > CLSIC_CUSTOM_MESSAGE_MAX) {
		clsic_err(clsic, "Message too big: %zd (MAX %ld)\n",
			  count, CLSIC_CUSTOM_MESSAGE_MAX);
		return -EINVAL;
	}

	/* There must be at least one fixed size message to send */
	if (count < CLSIC_FIXED_MSG_SZ) {
		clsic_err(clsic, "Message too small: %zd (MIN %d)\n",
			  count, CLSIC_FIXED_MSG_SZ);
		return -EINVAL;
	}

	/* Copy in message from debugfs */
	ret = simple_write_to_buffer(buf, CLSIC_CUSTOM_MESSAGE_MAX,
				     ppos, user_buf, count);
	if (ret < 0) {
		clsic_err(clsic, "simple_write_to_buffer error %zd\n", ret);
		return ret;
	}

	if (count != ret) {
		clsic_err(clsic, "mismatch count %zd ret %zd\n", count, ret);
		return ret;
	}

	/*
	 * Populate the bulk receive parameters in case the FSM sent causes a
	 * bulk response
	 */
	rxbuf = &buf[CLSIC_FIXED_MSG_SZ];
	rxcount = CLSIC_CUSTOM_MESSAGE_MAX - CLSIC_FIXED_MSG_SZ;

	if (count != CLSIC_FIXED_MSG_SZ) {
		clsic_dbg(clsic, "%zd bytes in - bulk message.\n", count);
		txbuf = &buf[CLSIC_FIXED_MSG_SZ];
		txcount = count - CLSIC_FIXED_MSG_SZ;
	} else {
		clsic_dbg(clsic, "%zd bytes in - fsm.\n", count);
	}

	msg_p = (union t_clsic_generic_message *) buf;
	ret = clsic_send_msg_sync_pm(clsic, msg_p, msg_p,
				     txbuf, txcount, rxbuf, rxcount);

	/*
	 * Response gets stored in private_data automatically but we need to
	 * know the size of the response so that userspace can read it out. The
	 * size will vary depending on whether it is a fsm or bulk response.
	 */
	if (clsic_get_bulk_bit(msg_p->rsp.hdr.sbc) == 1) {
		clsic_dbg(clsic, "bulk ret %d err 0x%xp size 0x%x\n",
			  ret, msg_p->bulk_rsp.hdr.err,
			  msg_p->bulk_rsp.hdr.bulk_sz);
		custom_msg->len = CLSIC_FIXED_MSG_SZ +
			msg_p->bulk_rsp.hdr.bulk_sz;
	} else {
		clsic_dbg(clsic, "fsm ret %d err 0x%x\n",
			  ret, msg_p->rsp.hdr.err);
		custom_msg->len = CLSIC_FIXED_MSG_SZ;
	}

	/*
	 * Propagate errors if there was a transfer error, otherwise the
	 * function should return the length of data read from user space.
	 *
	 * It is the responsibility of the caller to examine the FSM portion of
	 * the response to determine whether there was a failure at the
	 * protocol level.
	 */
	if (ret != 0)
		return ret;

	return count;
}

static const struct file_operations clsic_custom_message_fops = {
	.read = &clsic_custom_message_read,
	.write = &clsic_custom_message_write,
	.release = &clsic_custom_message_release,
	.open = &clsic_custom_message_open,
	.llseek = &default_llseek,
};

/*
 * The triggerworker interface is a debugging tool that allows the messaging
 * worker thread to be force started.
 *
 * Since the input and output paths of the driver were separated into different
 * contexts the only consumer of this is the debug control test and that is to
 * verify that the debug control mechanism is preventing message processing
 * active.
 */
static int clsic_triggerworker_write(void *data, u64 val)
{
	struct clsic *clsic = data;

	queue_work(clsic->message_worker_queue, &clsic->message_work);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(clsic_triggerworker_fops, NULL,
			 clsic_triggerworker_write, "%llu\n");

/*
 * The debug control interface is an interface intended to be used by Cirrus
 * tools during device bring up and integration.
 *
 * For more details see the debugcontrol.h header file.
 */
static int clsic_debugcontrol_write(void *data, u64 val)
{
	struct clsic *clsic = data;
	int ret = 0;
	struct completion a_completion;
	bool force_suspend = false;

	clsic_dbg(clsic, "Begin state: %d (%s) : %llu\n",
		  clsic->state, clsic_state_to_string(clsic->state), val);

	switch (val) {
	case CLSIC_DEBUGCONTROL_RELEASED:
		/*
		 * if the state was 0, it was already released
		 * if the state was 1, there was a request pending, interrupt it
		 * if the state was 2, a previous request was granted
		 */
		mutex_lock(&clsic->message_lock);

		if (clsic->state == CLSIC_STATE_DEBUGCONTROL_REQUESTED) {
			if (clsic->debugcontrol_completion != NULL)
				complete(clsic->debugcontrol_completion);
			clsic_state_set(clsic, CLSIC_STATE_ON,
					CLSIC_STATE_CHANGE_LOCKHELD);
		}
		if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED) {
			/* this put matches the one on grant */
			pm_runtime_mark_last_busy(clsic->dev);
			pm_runtime_put_autosuspend(clsic->dev);

			force_suspend = true;
			clsic_irq_enable(clsic);
		}
		mutex_unlock(&clsic->message_lock);

		if (force_suspend)
			pm_runtime_suspend(clsic->dev);

		break;
	case CLSIC_DEBUGCONTROL_REQUESTED:
		/*
		 * When in a state of debug control the expectation is that the
		 * device will be left in a usable state - in driver terms this
		 * means that the debug control has a pm runtime get on the
		 * device to prevent it being powered off using the autosuspend
		 * mechanism.
		 *
		 * Performing the get may also cause the device to resume if it
		 * is currenly powered OFF, if that occurs give this a chance to
		 * complete.
		 */
		pm_runtime_get_sync(clsic->dev);
		if (clsic_wait_for_state(clsic, CLSIC_STATE_ON,
					 CLSIC_WAIT_FOR_STATE_MAX_CYCLES,
					 CLSIC_WAIT_FOR_STATE_DELAY_MS))
			clsic_info(clsic, "Warning: state is %s\n",
				   clsic_state_to_string(clsic->state));

		/*
		 * if was state 0 and no current message -> successful lock
		 * if was state 1, someone is already attempting to lock
		 * if was state 2, was already locked
		 * if was state 0, but there is a current message we have to
		 * wait
		 */

		mutex_lock(&clsic->message_lock);
		if ((clsic->state == CLSIC_STATE_ON) &&
		    (clsic->current_msg == NULL)) {
			clsic_irq_disable(clsic);
			clsic_state_set(clsic, CLSIC_STATE_DEBUGCONTROL_GRANTED,
					CLSIC_STATE_CHANGE_LOCKHELD);
			/*
			 * Purge the message queues to clear any messages that
			 * have been added to the waiting to send queue but the
			 * worker thread has not started processing.
			 */
			clsic_purge_message_queues(clsic);

			ret = 0;
		} else if (clsic->state == CLSIC_STATE_DEBUGCONTROL_REQUESTED) {
			ret = -EBUSY;
		} else if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED) {
			ret = -EBUSY;
		} else if (clsic->state == CLSIC_STATE_OFF) {
			ret = -EBUSY;
		} else if (clsic->state == CLSIC_STATE_ON) {
			init_completion(&a_completion);
			clsic->debugcontrol_completion = &a_completion;
			clsic_state_set(clsic,
					CLSIC_STATE_DEBUGCONTROL_REQUESTED,
					CLSIC_STATE_CHANGE_LOCKHELD);
			mutex_unlock(&clsic->message_lock);
			ret = wait_for_completion_interruptible(&a_completion);
			if (schedule_work(&clsic->message_work) == false) {
				flush_scheduled_work();
				schedule_work(&clsic->message_work);
			}

			mutex_lock(&clsic->message_lock);
		} else {
			clsic_err(clsic, "state %d unhandled\n", clsic->state);
		}

		if (ret == -ERESTARTSYS)
			clsic_err(clsic, "interrupted\n");

		if (clsic->debugcontrol_completion == &a_completion) {
			clsic->debugcontrol_completion = NULL;

			/*
			 * There is a possible race where the worker thread
			 * accepts debugcontrol and whilst completing it this
			 * thread is interrupted, in that case clear the error
			 * code from the interruption and return success
			 */
			if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED)
				ret = 0;
		}
		mutex_unlock(&clsic->message_lock);

		/*
		 * If this context failed to obtain debug control - release the
		 * power request
		 */
		if (ret != 0) {
			pm_runtime_mark_last_busy(clsic->dev);
			pm_runtime_put_autosuspend(clsic->dev);
		}

		break;
	default:
		clsic_dbg(clsic, "defaulted 0x%llx\n", val);
	}

	clsic_dbg(clsic, "Final state: %d (%s) ret %d\n", clsic->state,
		  clsic_state_to_string(clsic->state), ret);

	return ret;
}

static int clsic_debugcontrol_read(void *data, u64 *val)
{
	struct clsic *clsic = data;

	/*
	 * This bridges from the driver state enumerations to the debugcontrol
	 * enumerations (which are part of the userspace API)
	 */
	if (clsic->state == CLSIC_STATE_DEBUGCONTROL_REQUESTED)
		*val = CLSIC_DEBUGCONTROL_REQUESTED;
	else if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED)
		*val = CLSIC_DEBUGCONTROL_GRANTED;
	else
		*val = CLSIC_DEBUGCONTROL_RELEASED;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(clsic_debugcontrol_fops,
			 clsic_debugcontrol_read,
			 clsic_debugcontrol_write, "%llu\n");
#endif

/**
 * This allows services to mark themselves as using the secure processor.
 *
 * This bitfield is checked in the timer callback, if if any service is marked
 * as busy then automatic shutdown of the secure processor is prevented.
 *
 * When all services release their marks then this function will restart the
 * secure processor shutdown timer.
 *
 * It is not a reference counted feature and services are expected to track
 * their own usage.
 *
 * This function also ensures that the device remains powered
 */
void clsic_msgproc_use(struct clsic *clsic, uint8_t service_instance)
{
	uint8_t bit_val = (1 << service_instance);

	if (!test_and_set_bit(bit_val, clsic->clsic_services_state)) {
		clsic_msgproc_shutdown_cancel(clsic, true);
		pm_runtime_get_sync(clsic->dev);
	} else
		clsic_info(clsic, "double use %d", service_instance);

}
EXPORT_SYMBOL_GPL(clsic_msgproc_use);

void clsic_msgproc_release(struct clsic *clsic, uint8_t service_instance)
{
	uint8_t bit_val = (1 << service_instance);

	if (test_and_clear_bit(bit_val, clsic->clsic_services_state)) {
		mutex_lock(&clsic->message_lock);
		if (list_empty(&clsic->waiting_to_send) &&
		    list_empty(&clsic->waiting_for_response))
			clsic_msgproc_shutdown_schedule(clsic);
		mutex_unlock(&clsic->message_lock);
		pm_runtime_mark_last_busy(clsic->dev);
		pm_runtime_put_autosuspend(clsic->dev);
	} else
		clsic_info(clsic, "double release %d", service_instance);
}
EXPORT_SYMBOL_GPL(clsic_msgproc_release);

/*
 * Check the services bitmap
 * If at least one service is active (= 1), return false
 * If all registered services are idle (= 0), return true
 */
static bool clsic_msgproc_services_active(struct clsic *clsic)
{
	return !bitmap_empty(clsic->clsic_services_state, CLSIC_SERVICE_COUNT);
}

/**
 * Callback function for clsic_msgproc_shutdown_work delayed work queue (that
 * just attempts to issue a shutdown command)
 */
static void clsic_msgproc_shutdown_fn(struct work_struct *data)
{
	struct clsic *clsic = container_of(data, struct clsic,
					   clsic_msgproc_shutdown_work.work);

	if (clsic_msgproc_services_active(clsic))
		return;

	clsic_send_shutdown_cmd(clsic);
}

/*
 * Called at points where the messaging layer is idle, if no services are using
 * the messaging processor then start the timer to shut it down
 */
void clsic_msgproc_shutdown_schedule(struct clsic *clsic)
{
	int ret;

	if (clsic_msgproc_services_active(clsic))
		return;

	ret = schedule_delayed_work(&clsic->clsic_msgproc_shutdown_work,
				    (HZ * CLSIC_MSGPROC_SHUTDOWN_TIMEOUT));

	trace_clsic_msgproc_shutdown_schedule(ret);
}

bool clsic_msgproc_shutdown_cancel(struct clsic *clsic, bool sync)
{
	bool ret = 0;

	if (sync)
		ret = cancel_delayed_work_sync(
					   &clsic->clsic_msgproc_shutdown_work);
	else
		ret = cancel_delayed_work(&clsic->clsic_msgproc_shutdown_work);

	trace_clsic_msgproc_shutdown_cancel(sync, ret ? 1 : 0);

	return ret;
}

/*
 * The workerthread timer callback provides the timeout mechanism
 */
static void clsic_workerthread_timer_function(unsigned long data)
{
	struct clsic *clsic = (struct clsic *)data;

	queue_work(clsic->message_worker_queue, &clsic->message_work);
}

int clsic_setup_message_interface(struct clsic *clsic)
{
	int ret = 0;

	mutex_init(&clsic->message_lock);

	clsic->message_cache = kmem_cache_create("clsic_messages",
						 sizeof(struct clsic_message),
						 0, 0, NULL);
	if (clsic->message_cache == NULL)
		return -ENOMEM;

	clsic->message_worker_queue =
		create_singlethread_workqueue(CLSIC_WORKERTHREAD_NAME);
	if (clsic->message_worker_queue == NULL) {
		kmem_cache_destroy(clsic->message_cache);
		return -ENOMEM;
	}

	INIT_WORK(&clsic->message_work, clsic_message_worker);

	INIT_DELAYED_WORK(&clsic->clsic_msgproc_shutdown_work,
			  clsic_msgproc_shutdown_fn);

	clsic->current_msg = NULL;
	INIT_LIST_HEAD(&clsic->waiting_to_send);
	INIT_LIST_HEAD(&clsic->waiting_for_response);
	INIT_LIST_HEAD(&clsic->completed_messages);

	init_timer(&clsic->workerthread_timer);
	clsic->workerthread_timer.function = &clsic_workerthread_timer_function;
	clsic->workerthread_timer.data = (unsigned long) clsic;

	clsic->timeout_counter = 0;
	clsic->messages_sent = 0;
	clsic->messages_received = 0;
	clsic->msgproc_state = CLSIC_MSGPROC_OFF;

#ifdef CONFIG_DEBUG_FS
	debugfs_create_file("messages", 0444, clsic->debugfs_root, clsic,
			    &clsic_messages_fops);

	debugfs_create_file("send_message", 0660, clsic->debugfs_root, clsic,
			    &clsic_custom_message_fops);

	debugfs_create_file("debugcontrol", 0220, clsic->debugfs_root, clsic,
			    &clsic_debugcontrol_fops);

	debugfs_create_file("triggerworker", 0220, clsic->debugfs_root, clsic,
			    &clsic_triggerworker_fops);
#endif

	return ret;
}

void clsic_shutdown_message_interface(struct clsic *clsic)
{
	struct clsic_message *tmp_msg;
	struct clsic_message *next_msg;

	/*
	 * If any service has requested the messaging processor to remain on,
	 * release that request.
	 */
	if (clsic_msgproc_services_active(clsic)) {
		clsic_info(clsic, "info; a service was active\n");
		pm_runtime_mark_last_busy(clsic->dev);
		pm_runtime_put_autosuspend(clsic->dev);
	}

	/*
	 * XXX check whether the put is required, which get does it match up
	 * with?
	 */
	if (clsic_msgproc_shutdown_cancel(clsic, true)) {
		clsic_info(clsic, "info; msgproc timer cancelled\n");
		pm_runtime_put_autosuspend(clsic->dev);
	}

	del_timer_sync(&clsic->workerthread_timer);

	flush_workqueue(clsic->message_worker_queue);
	destroy_workqueue(clsic->message_worker_queue);
	mutex_lock(&clsic->message_lock);

	/*
	 * kmem_cache_destroy() demands that all objects in the cache have been
	 * released.
	 *
	 * Asynchronous messages can be retained by a service handler's
	 * callback, services are notified when the system is shutting down and
	 * should release all of their outstanding messages.
	 *
	 * The messaging layer tracks messages that have been retained by
	 * asynchronous callbacks - iterate the list and dump the message
	 * details and forcefully release any that are outstanding.
	 */

	if (!list_empty(&clsic->waiting_to_send)) {
		list_for_each_entry_safe(tmp_msg,
					 next_msg,
					 &clsic->waiting_to_send,
					 private_link) {
			clsic_dump_message(clsic, tmp_msg,
					   "Force release (WTS)");
			clsic_release_msg(clsic, tmp_msg);
		}
	}

	if (!list_empty(&clsic->waiting_for_response)) {
		list_for_each_entry_safe(tmp_msg,
					 next_msg,
					 &clsic->waiting_for_response,
					 private_link) {
			clsic_dump_message(clsic, tmp_msg,
					   "Force release (WFR)");
			clsic_release_msg(clsic, tmp_msg);
		}
	}

	if (!list_empty(&clsic->completed_messages)) {
		list_for_each_entry_safe(tmp_msg,
					 next_msg,
					 &clsic->completed_messages,
					 private_link) {
			clsic_dump_message(clsic, tmp_msg,
					   "Force release (Completed)");
			clsic_release_msg(clsic, tmp_msg);
		}
	}
	mutex_unlock(&clsic->message_lock);

	kmem_cache_destroy(clsic->message_cache);
}

/*
 * Iterate through the waiting to send and waiting for response lists
 * cancelling the messages on the queues.
 *
 * The callers that have their messages cancelled will receive an -EINTR return
 * code.
 *
 * The completed list is left alone as this function can cause async messages
 * to be moved to it. It is the responsibilty of the client to release messages
 * it had retained before it can sent more with the same service instance
 * message id pair.
 *
 * The clsic->message_lock is expected to be held when calling this function
 */
void clsic_purge_message_queues(struct clsic *clsic)
{
	struct clsic_message *tmp_msg;
	struct clsic_message *next_msg;

	if (!list_empty(&clsic->waiting_to_send)) {
		list_for_each_entry_safe(tmp_msg,
					 next_msg,
					 &clsic->waiting_to_send,
					 private_link) {
			clsic_set_msgstate(tmp_msg, CLSIC_MSG_INTERRUPTED);
			clsic_complete_message_core(clsic, tmp_msg);
		}
	}

	/*
	 * clsic->current_msg will be in the waiting for response list so
	 * requires no additional handling
	 */

	if (!list_empty(&clsic->waiting_for_response)) {
		list_for_each_entry_safe(tmp_msg,
					 next_msg,
					 &clsic->waiting_for_response,
					 private_link) {
			clsic_set_msgstate(tmp_msg, CLSIC_MSG_INTERRUPTED);
			clsic_complete_message_core(clsic, tmp_msg);
		}
	}
}

/*
 * Must only be called from the incoming message context
 *
 * This function enforces a maximum read size when accessing the SCP FIFO
 */
static int clsic_fifo_read(struct clsic *clsic, uint8_t *dest,
			   const uint32_t len)
{
	int ret = 0;
	uint32_t bytes_thisread;
	uint32_t bytes_left = len;
	uint8_t *tmp_dest = dest;

	while (bytes_left != 0) {
		bytes_thisread = min_t(uint32_t, bytes_left,
				       CLSIC_FIFO_TRANSACTION_MAX);
		ret = regmap_raw_read(clsic->regmap, clsic->fifo_tx,
				      tmp_dest, bytes_thisread);
		if (ret != 0)
			goto err;
		bytes_left -= bytes_thisread;
		tmp_dest += bytes_thisread;
	}
err:
	pm_runtime_mark_last_busy(clsic->dev);
	return ret;
}

/*
 * Must only be called from the message thread context
 *
 * This function enforces a maximum write size when accessing the SCP FIFO
 */
static int clsic_fifo_write(struct clsic *clsic, uint8_t *src,
			    const uint32_t len)
{
	int ret = 0;
	uint32_t bytes_thiswrite;
	uint32_t bytes_left = len;
	uint8_t *tmp_src = src;

	while (bytes_left != 0) {
		bytes_thiswrite = min_t(uint32_t, bytes_left,
					CLSIC_FIFO_TRANSACTION_MAX);
		ret = regmap_raw_write(clsic->regmap,
				       TACNA_CPF1_RX_WRDATA,
				       tmp_src, bytes_thiswrite);
		if (ret != 0)
			goto err;
		bytes_left -= bytes_thiswrite;
		tmp_src += bytes_thiswrite;
	}

err:
	pm_runtime_mark_last_busy(clsic->dev);
	return ret;
}

/*
 * Must only be called from the incoming message context
 */
static int clsic_fifo_drain(struct clsic *clsic,
			    struct clsic_message *msg)
{
	uint8_t *scratchbuf;
	uint32_t bytes_left;
	uint32_t bytes_thisread;
	int ret = 0;

	/* There is nothing to do on non-bulk messages */
	if (clsic_get_bulkbit(msg) == false)
		return 0;

	scratchbuf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (scratchbuf == NULL)
		return -ENOMEM;

	bytes_left = CLSIC_BULKSZ_ROUNDUP(clsic_get_bulk_sz(&msg->fsm));
	trace_clsic_fifo_readbulk(bytes_left);

	while (bytes_left != 0) {
		bytes_thisread = min_t(uint32_t, bytes_left, PAGE_SIZE);

		ret = clsic_fifo_read(clsic, scratchbuf, bytes_thisread);
		if (ret != 0)
			break;

		bytes_left -= bytes_thisread;
	}

	kfree(scratchbuf);

	return ret;
}

/*
 * Must only be called from the incoming message context
 */
static int clsic_fifo_readmessage(struct clsic *clsic,
				  struct clsic_message *msg)
{
	int ret;

	/* Read the fixed size message from the TX FIFO */
	ret = clsic_fifo_read(clsic, msg->fsm.raw, CLSIC_FIXED_MSG_SZ);
	if (ret != 0)
		return ret;

	clsic->messages_received++;

	trace_clsic_fifo_readmessage(msg);

	return ret;
}

/*
 * Must only be called from the incoming message context
 */
int clsic_fifo_readbulk_payload(struct clsic *clsic,
				struct clsic_message *msg,
				uint8_t *dest, size_t dest_size)
{
	uint32_t bulk_sz;
	uint32_t read_size;
	int ret;

	if (!clsic_get_bulkbit(msg))
		return -EINVAL;

	bulk_sz = clsic_get_bulk_sz(&msg->fsm);
	read_size = CLSIC_BULKSZ_ROUNDUP(bulk_sz);
	if (read_size <= dest_size) {
		trace_clsic_fifo_readbulk(read_size);
		ret = clsic_fifo_read(clsic, dest, read_size);
		if (ret == 0)
			ret = bulk_sz;
	} else {
		clsic_dbg(clsic, "too small: sz 0x%x max 0x%zx (drain)\n",
			  read_size, dest_size);
		clsic_fifo_drain(clsic, msg);
		ret = -E2BIG;
	}

	return ret;
}

static int clsic_fifo_writemessage(struct clsic *clsic,
				   struct clsic_message *msg)
{
	int ret;

	ret = clsic_fifo_write(clsic, msg->fsm.raw, CLSIC_FIXED_MSG_SZ);

	trace_clsic_fifo_writemessage(msg);

	clsic->messages_sent++;

	return ret;
}

/*
 * Find a message that matches the supplied service instance and message id.
 *
 * This must be called with clsic->message_lock held
 */
static struct clsic_message *clsic_findmessage(struct clsic *clsic,
					       uint8_t svcinst, uint8_t msgid)
{
	struct clsic_message *tmp_msg;
	struct clsic_message *next_msg;

	/* Check most likely first - it is the current message */
	if (clsic_ismatch(clsic, clsic->current_msg, svcinst, msgid))
		return clsic->current_msg;

	/* Examine any messages waiting to send */
	list_for_each_entry_safe(tmp_msg,
				 next_msg,
				 &clsic->waiting_to_send,
				 private_link)
		if (clsic_ismatch(clsic, tmp_msg, svcinst, msgid))
			return tmp_msg;

	/* Examine any messages waiting for a response */
	list_for_each_entry_safe(tmp_msg,
				 next_msg,
				 &clsic->waiting_for_response,
				 private_link)
		if (clsic_ismatch(clsic, tmp_msg, svcinst, msgid))
			return tmp_msg;

	/* No match was found */
	clsic_dbg(clsic, "No match for %d %d\n", svcinst, msgid);
	return NULL;
}

/*
 * This is a helper function that handles the scenario where the sync context
 * sending a message is signalled, this is interpreted as an interruption.
 *
 * If it is possible to remove the message from the messaging layer then it
 * does so, but when the message is involved in an ongoing message exchange it
 * must be allowed to finish.
 *
 * In situations where the transfer must finish the non-interruptible form of
 * the wait for completion is used.
 */
static int clsic_send_message_interrupted(struct clsic *clsic,
					   struct clsic_message *msg)
{
	int ret = 0;

	mutex_lock(&clsic->message_lock);

	switch (CLSIC_GET_MSGSTATE(msg)) {
	case CLSIC_MSG_SENDING:
	case CLSIC_MSG_SENT:
	case CLSIC_MSG_ACK:
	case CLSIC_MSG_BULKTX:
	case CLSIC_MSG_BULKTX_WAIT:
	case CLSIC_MSG_BULKRX:
		/*
		 * In these states there is an ongoing transfer so cannot
		 * cancel the message - must wait for it to finish.
		 */
		clsic_err(clsic, "%p is in use, must wait %d\n", msg,
			  CLSIC_GET_MSGSTATE(msg));

		mutex_unlock(&clsic->message_lock);
		wait_for_completion(&msg->completion);
		clsic_err(clsic, "%p wait finished, state %d\n", msg,
			  CLSIC_GET_MSGSTATE(msg));
		break;
	case CLSIC_MSG_INITIAL:
	case CLSIC_MSG_WAITING:
		/*
		 * Message hasn't been started yet and can be trivially
		 * stopped.
		 */
		clsic_set_msgstate(msg, CLSIC_MSG_INTERRUPTED);
		clsic_unlink_message(clsic, msg);
		mutex_unlock(&clsic->message_lock);
		ret = -EINTR;
		break;
	case CLSIC_MSG_SUCCESS:
	case CLSIC_MSG_FAILED:
	case CLSIC_MSG_INTERRUPTED:
	case CLSIC_MSG_TIMEOUT:
	default:
		/* nothing to do */
		mutex_unlock(&clsic->message_lock);
	}

	return ret;
}

/*
 * This is the message sending internals of the clsic_send_message() function,
 * it handles the interactions with the messaging layer.
 *
 * It's primary function is to make that the integrity of the data structures
 * in the messaging layer are preserved, this function introduces the message
 * structure into the layer and returns. The calling sync and async utility
 * functions handle whether to block or return to the calling context.
 */
static int clsic_send_message_core(struct clsic *clsic,
				   struct clsic_message *msg)
{
	int i;

	/* Sanity check message to be sent */
	if (clsic_get_cran(msg->fsm.cmd.hdr.sbc) != CLSIC_CRAN_CMD)
		return -EINVAL;

	/*
	 * Bulk message sanity checks
	 *
	 * - if a bulk buffer is provided then the size must also be set
	 * - if the size is set then a bulk buffer must also be provided
	 *
	 * This rule applies for both transmit and receive buffers
	 */
	if (((msg->bulk_txbuf != NULL) && (msg->bulk_txbuf_maxsize == 0)) ||
	    ((msg->bulk_txbuf_maxsize != 0) && (msg->bulk_txbuf == NULL)) ||
	    ((msg->bulk_rxbuf != NULL) && (msg->bulk_rxbuf_maxsize == 0)) ||
	    ((msg->bulk_rxbuf_maxsize != 0) && (msg->bulk_rxbuf == NULL)))
		return -EINVAL;

	/*
	 * If this is a message with a bulk transmit buffer then sanity check
	 * the fields in the message payload
	 * - the bulk_sz should be non zero
	 * - the bulk_sz should not be greater than the provided buffer
	 *
	 * If the message gets through those checks then also set the bulk bit.
	 */
	if (msg->bulk_txbuf != NULL) {
		if (msg->fsm.bulk_cmd.hdr.bulk_sz == 0)
			return -EINVAL;

		if (msg->fsm.bulk_cmd.hdr.bulk_sz > msg->bulk_txbuf_maxsize)
			return -EINVAL;

		clsic_set_bulk(&msg->fsm.cmd.hdr.sbc, 1);
	}

	if (clsic_get_servinst(msg) == CLSIC_SRV_INST_BLD)
		msg->timeout = CLSIC_MSG_TIMEOUT_PERIOD_BOOTLOADER;
	else
		msg->timeout = CLSIC_MSG_TIMEOUT_PERIOD;

	/* Check whether messaging is limited to the bootloader service */
	if ((clsic->blrequest != CLSIC_BL_IDLE) &&
	    (clsic_get_servinst(msg) != CLSIC_SRV_INST_BLD)) {
		for (i = 0; i < CLSIC_WAIT_FOR_STATE_MAX_CYCLES; i++)
			if (clsic->blrequest != CLSIC_BL_IDLE)
				msleep(CLSIC_WAIT_FOR_STATE_DELAY_MS);

		if (clsic->blrequest != CLSIC_BL_IDLE) {
			clsic_dump_message(clsic, msg, "Can't send");
			clsic_info(clsic, "Can't send: %s %d %d %d\n",
				   clsic_state_to_string(clsic->state),
				   clsic->blrequest,
				   clsic->service_states,
				   clsic->msgproc_state);
			return -EBUSY;
		}
	}

	/*
	 * The message will be added to the messaging layer - take the
	 * appropriate guards and action
	 */
	if (mutex_lock_interruptible(&clsic->message_lock))
		return -EINTR;

	/* Check that it is possible to send the message */
	switch (clsic->state) {
	case CLSIC_STATE_HALTED:
		/* allow bootloader messages to pass in halted state */
		if (clsic_get_servinst(msg) == CLSIC_SRV_INST_BLD)
			break;
		/* fallthrough for all other messages */
	case CLSIC_STATE_DEBUGCONTROL_GRANTED:
		/*
		 * The driver has lost communication with the device or is
		 * being prevented from communicating with the device.
		 */
		mutex_unlock(&clsic->message_lock);
		return -EIO;
	default:
		/*
		 * In all other states allow the message to pass this
		 * checkpoint
		 */
		break;
	}

	/*
	 * Check that no message exists in the system with the current service
	 * instance and message id.
	 */
	if (clsic_findmessage(clsic, clsic_get_srv_inst(msg->fsm.cmd.hdr.sbc),
			      msg->fsm.cmd.hdr.msgid) != NULL) {
		mutex_unlock(&clsic->message_lock);
		return -EINVAL;
	}

	clsic_msgproc_shutdown_cancel(clsic, false);

	clsic_set_msgstate(msg, CLSIC_MSG_WAITING);

	list_add_tail(&msg->private_link, &clsic->waiting_to_send);

	mutex_unlock(&clsic->message_lock);

	queue_work(clsic->message_worker_queue, &clsic->message_work);

	return 0;
}

/*
 * The driver has received a message with a CRAN type of response, the purpose
 * of this function is to match it with the originating message, handle the
 * response payload and pass it to the caller.
 *
 * If this response was for the current_msg then the handling code will trigger
 * any messages waiting to send, they are not signalled here as we'd get device
 * lock contention and we'd like the caller of this message to win.
 */
static int clsic_handle_message_response(struct clsic *clsic,
					 struct clsic_message *msg)
{
	struct clsic_message *found_msg;
	uint32_t payload_size = 0;
	int ret;
	union t_clsic_generic_message *msg_p =
		(union t_clsic_generic_message *) &msg->fsm;

	mutex_lock(&clsic->message_lock);
	found_msg = clsic_findmessage(clsic, clsic_get_servinst(msg),
				      clsic_get_messageid(msg));

	/*
	 * The message wasn't matched to an outstanding request - it could
	 * could be due to the message's caller being interrupted before it
	 * could be serviced (we can't do anything more for these messages)
	 */
	if (found_msg == NULL) {
		mutex_unlock(&clsic->message_lock);
		clsic_dump_message(clsic, msg, "Response not found");
		clsic_fifo_drain(clsic, msg);
		return CLSIC_UNHANDLED;
	}

	/* Copy back the fixed message response */
	memcpy(&found_msg->response, &msg->fsm, CLSIC_FIXED_MSG_SZ);

	if (clsic_get_bulkbit(msg)) {
		/* The response contains a bulk transfer */
		payload_size =
			CLSIC_BULKSZ_ROUNDUP(msg_p->bulk_rsp.hdr.bulk_sz);

		if (payload_size > found_msg->bulk_rxbuf_maxsize) {
			/*
			 * If the buffer was too small, drain the FIFO
			 * and return error
			 */
			clsic_fifo_drain(clsic, msg);
			clsic_dump_message(clsic, msg,
					   "Bulk response too large:");
			clsic_dump_message(clsic, found_msg,
					   "Bulk response too large original:");
			clsic_set_msgstate(found_msg, CLSIC_MSG_FAILED);
			goto message_handled;
		}

		clsic_set_msgstate(found_msg, CLSIC_MSG_BULKRX);
		trace_clsic_fifo_readbulk(payload_size);
		ret = clsic_fifo_read(clsic, found_msg->bulk_rxbuf,
				      payload_size);
		if (ret != 0) {
			clsic_err(clsic, "clsic_fifo_read : %d\n", ret);
			/*
			 * A read error from the device is bad news, the FIFO
			 * could still be partially filled. Transition to the
			 * halted state
			 */
			clsic_device_error(clsic, CLSIC_DEVICE_ERROR_LOCKHELD);
			mutex_unlock(&clsic->message_lock);
			return CLSIC_HANDLED;
		}
		clsic_set_msgstate(found_msg, CLSIC_MSG_SUCCESS);
	} else {
		/* message is now serviced */
		clsic_set_msgstate(found_msg, CLSIC_MSG_SUCCESS);
	}

message_handled:
	clsic_complete_message_core(clsic, found_msg);

	mutex_unlock(&clsic->message_lock);

	return CLSIC_HANDLED;
}

/*
 * The messaging layer has received a CRAN_ACKNOWLEDGMENT message - the purpose
 * of this function is to unblock the command interface so that other messages
 * may be sent.
 *
 * The normal scenario is that the current_msg will match the ACK (messages
 * should only ever receive a single ACK and when they are ACK'd they cease to
 * be the current message).
 */
static void clsic_handle_message_acknowledgment(struct clsic *clsic,
						struct clsic_message *ack)
{
	mutex_lock(&clsic->message_lock);
	if (clsic_compare(clsic, ack, clsic->current_msg)) {
		/*
		 * It is an ACK for the currently outstanding message, unblock
		 * the messaging layer so it may send another command message.
		 */
		clsic_set_msgstate(clsic->current_msg, CLSIC_MSG_ACK);
		clsic->current_msg = NULL;
	} else if (clsic->current_msg != NULL) {
		/*
		 * This ACK can't be paired with the current message.
		 * Log the service instance and message id of both.
		 */
		clsic_err(clsic, "mismatch %d:%d %d:%d\n",
			  clsic_get_servinst(ack),
			  clsic_get_servinst(clsic->current_msg),
			  clsic_get_messageid(ack),
			  clsic_get_messageid(clsic->current_msg));
	} else {
		/*
		 * This ACK can't be paired and there is no current message.
		 * Log its service instance and message id.
		 */
		clsic_err(clsic, "%d:%d no current_msg\n",
			  clsic_get_servinst(ack), clsic_get_messageid(ack));
	}
	mutex_unlock(&clsic->message_lock);
}

/*
 * Called by the system service when it receives a RXDMA setup status
 * notification message, this stores the necessary information and releases the
 * waiting message to continue sending data.
 */
void clsic_handle_message_rxdma_status(struct clsic *clsic,
				       struct clsic_message *msg)
{
	union clsic_sys_msg *msg_p = (union clsic_sys_msg *) &msg->fsm;
	uint8_t svcinst = msg_p->nty_rxdma_sts.srv_inst;
	uint8_t msgid = msg_p->nty_rxdma_sts.msgid;

	mutex_lock(&clsic->message_lock);
	/*
	 * An RXDMA notification message is only valid for the current message
	 * - i.e. sending a bulk message command will never defer handling with
	 * an ACK
	 */
	if (clsic->current_msg == NULL) {
		clsic_err(clsic, "RXDMASTATUS no current message 0x%x 0x%x\n",
			  svcinst, msgid);
		goto rxdmastatus_error;
	} else if (!clsic_ismatch(clsic, clsic->current_msg, svcinst, msgid)
		   || !clsic_get_bulkbit(clsic->current_msg)) {
		clsic_err(clsic,
			  "RXDMASTATUS mismatch svcinst 0x%x 0x%x\n",
			  svcinst, clsic_get_servinst(clsic->current_msg));
		clsic_err(clsic,
			  "RXDMASTATUS mismatch msgid 0x%x 0x%x\n",
			  msgid, clsic_get_messageid(clsic->current_msg));
		clsic_err(clsic, "RXDMASTATUS mismatch\n");
		clsic_dump_message(clsic, clsic->current_msg, "Current");
		goto rxdmastatus_error;
	} else {
		/*
		 * Copy back the payload and signal the caller to continue
		 * handling this message
		 */
		memcpy(&clsic->current_msg->response, &msg->fsm,
		       CLSIC_FIXED_MSG_SZ);
		clsic_set_msgstate(clsic->current_msg, CLSIC_MSG_BULKTX);

		/* Trigger the worker thread to send more */
		queue_work(clsic->message_worker_queue, &clsic->message_work);
	}

	mutex_unlock(&clsic->message_lock);
	return;

rxdmastatus_error:
	clsic_device_error(clsic, CLSIC_DEVICE_ERROR_LOCKHELD);
	mutex_unlock(&clsic->message_lock);
}

/*
 * Called by the system service when it receives an invalid command
 * notification message, this stores the necessary information and fails the
 * waiting command so it can propagate the error.
 */
void clsic_handle_message_invalid_cmd(struct clsic *clsic,
				      struct clsic_message *msg)
{
	struct clsic_message *tmp_msg = NULL;
	union clsic_sys_msg *msg_p = (union clsic_sys_msg *) &msg->fsm.raw;
	uint8_t svcinst = msg_p->nty_inval_cmd.srv_inst;
	uint8_t msgid = msg_p->nty_inval_cmd.msgid;

	clsic_dbg(clsic, "INVALID COMMAND 0x%x\n", msg_p->nty_inval_cmd.err);
	mutex_lock(&clsic->message_lock);

	/* this notification should apply to the current_msg */
	if (clsic->current_msg == NULL) {
		clsic_info(clsic, "No current message...\n");
		goto error_unlock;
	}

	/* and the current_msg should match the details in the notification */
	if (!clsic_ismatch(clsic, clsic->current_msg, svcinst, msgid)) {
		clsic_info(clsic, "Not the current message...\n");
		goto error_unlock;
	}

	tmp_msg = clsic->current_msg;

	/* Copy the invalid message response for to the client */
	memcpy(&tmp_msg->response, &msg->fsm, CLSIC_FIXED_MSG_SZ);

	clsic_dump_message(clsic, tmp_msg, "INVALID COMMAND");

	/* Complete the message with error */
	clsic_set_msgstate(tmp_msg, CLSIC_MSG_FAILED);
	clsic_complete_message_core(clsic, tmp_msg);

error_unlock:
	mutex_unlock(&clsic->message_lock);
}

/*
 * Pass a message received from the device to the associated service handler
 * for processing.
 *
 * Notifications will always be sent to the service handler, this is a normal
 * operation.
 *
 * Response messages will be sent to the service handler when an originating
 * message cannot be found, services may then choose to handle the message (or
 * not).
 *
 * At the end of this function the processing of this message is "done", any
 * payload associated with unhandled bulk message should also be drained from
 * the FIFO or that data will interfere with the operation of the messaging
 * protocol.
 *
 * This function expects neither the messaging lock nor the service lock to be
 * held.
 */
static void clsic_handle_message_inservice(struct clsic *clsic,
					   struct clsic_message *msg)
{
	int ret = CLSIC_UNHANDLED;
	uint8_t servinst = clsic_get_servinst(msg);
	struct clsic_service *service_handler = NULL;

	if (servinst > CLSIC_SERVICE_MAX) {
		clsic_dump_message(clsic, msg, "service out of range");
	} else {
		mutex_lock(&clsic->service_lock);
		service_handler = clsic->service_handlers[servinst];
		if ((service_handler != NULL) &&
		    (service_handler->callback != NULL))
			ret = service_handler->callback(clsic,
							service_handler, msg);
		mutex_unlock(&clsic->service_lock);
	}

	if (ret == CLSIC_UNHANDLED) {
		/*
		 * The message is unhandled - this means it was not matched to
		 * an originating message and the handler associated with that
		 * service instance chose not to handle the message either.
		 *
		 * There is nothing more than can be done for the message so it
		 * will be dropped.
		 */
		clsic_dump_message(clsic, msg, "message unhandled by service");

		/*
		 * If this is a bulk response then the FIFO needs to be drained
		 */
		if (clsic_get_bulkbit(msg)) {
			clsic_err(clsic, "dropped message had bulk payload\n");
			clsic_fifo_drain(clsic, msg);
		}
	}
}

static void clsic_message_handler(struct clsic *clsic,
				  struct clsic_message *msg)
{
	clsic_msgproc_shutdown_cancel(clsic, false);

	/*
	 * If a message has just been received then the messaging processor is
	 * certainly on
	 */
	mutex_lock(&clsic->message_lock);
	if (clsic->msgproc_state == CLSIC_MSGPROC_OFF)
		clsic->msgproc_state = CLSIC_MSGPROC_ON;
	mutex_unlock(&clsic->message_lock);

	switch (clsic_get_cran_frommsg(msg)) {
	case CLSIC_CRAN_RSP:
		clsic_handle_message_response(clsic, msg);
		break;
	case CLSIC_CRAN_ACK:
		clsic_handle_message_acknowledgment(clsic, msg);
		break;
	case CLSIC_CRAN_NTY:
		/* Direct notification to the supporting service */
		clsic_handle_message_inservice(clsic, msg);
		break;
	case CLSIC_CRAN_CMD:
		/* The device should never send command messages to the host */
		/* fallthrough */
	default:
		clsic_dump_message(clsic, msg,
				   "message_handler() Unexpected CRAN");
	}

	/*
	 * TODO Review whether this could be moved to before the handler calls
	 * to prevent shutdown timer churn, though when I quickly tried it it
	 * didn't work.
	 */

	pm_runtime_mark_last_busy(clsic->dev);
	mutex_lock(&clsic->message_lock);
	if (list_empty(&clsic->waiting_to_send) &&
	    list_empty(&clsic->waiting_for_response))
		clsic_msgproc_shutdown_schedule(clsic);
	mutex_unlock(&clsic->message_lock);
}

/*
 * This handler function progresses messages and data being received.
 * It is called by the interrupt handling context.
 */
void clsic_handle_incoming_messages(struct clsic *clsic)
{
	int ret;
	uint32_t sts;
	struct clsic_message msg;

	/* If debugcontrol is asserted then do nothing */
	if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED) {
		clsic_dbg(clsic, "debugcontrol asserted\n");
		return;
	}

	/*
	 * Read the status register of the FIFO to see whether there are
	 * messages that should be processed
	 */
	do {
		ret = regmap_read(clsic->regmap, TACNA_CPF1_TX_GPR_STATUS1,
				  &sts);
		if (ret != 0) {
			clsic_err(clsic, "TACNA_CPF1_TX_GPR_STATUS1 ret %d\n",
				  ret);
			return;
		}

		/*
		 * If the status register shows that the FIFO is not empty then
		 * read a fixed sized message from the FIFO and call the message
		 * handler to progress it
		 */
		if ((sts & TACNA_CPF1_TX_FIFO_NOT_EMPTY_STS) != 0) {
			memset(&msg, 0, sizeof(struct clsic_message));
			ret = clsic_fifo_readmessage(clsic, &msg);
			if (ret != 0) {
				clsic_err(clsic, "readmessage %d\n", ret);
				return;
			}

			clsic_message_handler(clsic, &msg);
		}
		/* the loop handled a FIFO message then go around again */
	} while ((sts & TACNA_CPF1_TX_FIFO_NOT_EMPTY_STS) != 0);
}

static void clsic_message_worker_sending(struct clsic *clsic,
					 struct clsic_message *msg)
{
	int ret;

	if (clsic->msgproc_state == CLSIC_MSGPROC_OFF)
		clsic->msgproc_state = CLSIC_MSGPROC_ON;

	/*
	 * Send the message - the message_lock must be held over this
	 * process as it is possible that an IRQ based response could
	 * come back before the message state has been changed
	 */
	ret = clsic_fifo_writemessage(clsic, msg);
	if (ret != 0) {
		clsic_err(clsic, "writemessage error %d\n", ret);
		clsic_complete_message(clsic, msg, CLSIC_MSG_FAILED);
		return;
	}

	/*
	 * If the fixed size message just sent had a bulk payload to
	 * send too then it transitions to the BULKTX_WAIT state.
	 */
	if (msg->bulk_txbuf != NULL)
		clsic_set_msgstate(msg, CLSIC_MSG_BULKTX_WAIT);
	else
		clsic_set_msgstate(msg, CLSIC_MSG_SENT);
}

/*
 * Called from the message worker when the current message is a bulk message in
 * the BULK_TX state (i.e. the worker thread has sent the fixed size message
 * portion of the message and the system service has responded with a rxdma
 * notification message indicating how much data can be sent in this iteration.
 *
 * It is possible that it could take a series of iterations (of rxdma
 * notification -> send data) to send the full bulk payload.
 */
static void clsic_message_worker_bulktx(struct clsic *clsic,
					struct clsic_message *msg)
{
	union clsic_sys_msg *syssrv_msg =
		(union clsic_sys_msg *) &msg->response;
	size_t remaining;
	size_t sending;
	int ret;

	/*
	 * The system service notification may have communicated an error,
	 * handle them first.
	 */
	if (syssrv_msg->nty_rxdma_sts.err != CLSIC_ERR_NONE) {
		clsic_dbg(clsic, "%p error %d\n", msg,
			  syssrv_msg->nty_rxdma_sts.err);

		clsic_complete_message(clsic, msg, CLSIC_MSG_FAILED);
		return;
	}

	remaining = msg->bulk_txbuf_maxsize -
		(msg->bulk_txbuf_marker - msg->bulk_txbuf);
	sending = min(syssrv_msg->nty_rxdma_sts.slice_sz, remaining);

	if (sending != 0) {
		trace_clsic_fifo_writebulk(sending, remaining);
		ret = clsic_fifo_write(clsic, msg->bulk_txbuf_marker, sending);
		if (ret != 0) {
			clsic_complete_message(clsic, msg, CLSIC_MSG_FAILED);
			clsic_err(clsic, "fifo_write error %d\n", ret);
			return;
		}
		msg->bulk_txbuf_marker += sending;
	}

	if (sending == remaining)
		clsic_set_msgstate(msg, CLSIC_MSG_SENT);
	else
		clsic_set_msgstate(msg, CLSIC_MSG_BULKTX_WAIT);
}

/*
 * This work handler progresses the sending of messages and data, it operates
 * in it's own worker thread context.
 *
 * Symmetrically, the receiving of messages and data is performed by
 * clsic_handle_incoming_messages() in the context of the interrupt handling
 * thread.
 */
static void clsic_message_worker(struct work_struct *data)
{
	struct clsic *clsic = container_of(data, struct clsic,
					   message_work);
	struct clsic_message *msg_p = NULL;

	mutex_lock(&clsic->message_lock);

	/*
	 * If debugcontrol is requested (but not asserted, as that would have
	 * been handled earlier), check whether there are any outstanding
	 * messages that would prevent granting it.
	 */
	if ((clsic->state == CLSIC_STATE_DEBUGCONTROL_REQUESTED) &&
	    (clsic->current_msg == NULL)) {
		clsic_dbg(clsic, "debugcontrol granted\n");
		clsic_state_set(clsic, CLSIC_STATE_DEBUGCONTROL_GRANTED,
				CLSIC_STATE_CHANGE_LOCKHELD);

		/*
		 * The messaging worker could have been triggered by both a
		 * message being enqueued and debugcontrol being requested,
		 * purge the message queues now that debugcontrol is granted.
		 */
		clsic_purge_message_queues(clsic);

		/*
		 * Need to disable interrupts to prevent messages sent to the
		 * host being processed
		 */
		clsic_irq_disable(clsic);

		if (clsic->debugcontrol_completion != NULL) {
			complete(clsic->debugcontrol_completion);
			clsic_dbg(clsic, "debugcontrol completed\n");
		}
		goto unlock_return;
	} else if (clsic->state == CLSIC_STATE_DEBUGCONTROL_GRANTED) {
		goto unlock_return;
	}

	/*
	 * If there is no message currently being handled then try to start one
	 * from the waiting to send queue.
	 */
	if ((clsic->current_msg == NULL) &&
	    !list_empty(&clsic->waiting_to_send)) {
		msg_p = list_first_entry(&clsic->waiting_to_send,
					 struct clsic_message, private_link);

		list_del(&msg_p->private_link);
		clsic_set_msgstate(msg_p, CLSIC_MSG_SENDING);
		clsic->current_msg = msg_p;
		list_add_tail(&msg_p->private_link,
			      &clsic->waiting_for_response);
	}

	/* There is no command to progress, exit now */
	if (clsic->current_msg == NULL)
		goto unlock_return;

	/*
	 * The messaging worker can progress messages when they are in the
	 * SENDING or BULKTX states, in all other states it is waiting for the
	 * device to respond before it can act.
	 */
	msg_p = clsic->current_msg;
	if (CLSIC_GET_MSGSTATE(msg_p) == CLSIC_MSG_SENDING) {
		clsic_message_worker_sending(clsic, msg_p);
		clsic->timeout_counter = 0;
	} else if (CLSIC_GET_MSGSTATE(msg_p) == CLSIC_MSG_BULKTX) {
		clsic->timeout_counter = 0;
		clsic_message_worker_bulktx(clsic, msg_p);
	} else {
		/*
		 * In addition to sending data the worker thread also tracks
		 * whether messages have been outstanding (without an ACK or
		 * response) for a long period of time.
		 *
		 * In these cases it completes the message with a TIMEOUT error
		 * so the caller may process the condition.
		 *
		 * If messages are timing out then the messaging protocol is
		 * not functioning as expected - if this occurs the driver
		 * treats the condition as a fatal error, the currently
		 * outstanding messages are purged and the maintenance_handler
		 * is started which will reboot and reenumerate the device.
		 */
		if (clsic->timeout_counter >= msg_p->timeout) {
			clsic_err(clsic,
				  "Can't progress current message %d (timeout %d, waited %d, state %s)\n",
				  CLSIC_GET_MSGSTATE(msg_p),
				  msg_p->timeout,
				  clsic->timeout_counter,
				  clsic_state_to_string(clsic->state));
			clsic->timeout_counter = 0;
			clsic_complete_message(clsic, msg_p, CLSIC_MSG_TIMEOUT);

			clsic_device_error(clsic, CLSIC_DEVICE_ERROR_LOCKHELD);
		} else {
			/*
			 * the current message has not timed out
			 */
			clsic->timeout_counter +=
				CLSIC_WORKERTHREAD_TIMER_PERIOD;
		}
	}

	/*
	 * Start or adjust the worker thread timer.  This timer is only left
	 * running if there are messages being processed.
	 */
	mod_timer(&clsic->workerthread_timer,
		  jiffies + (HZ * CLSIC_WORKERTHREAD_TIMER_PERIOD));
unlock_return:
	mutex_unlock(&clsic->message_lock);
}

/*
 * Simple callback function to release the waiting synchronous message context,
 * return RETAINED so that the async code does not free the message structure.
 */
static enum clsic_message_cb_ret clsic_send_msg_sync_cb(struct clsic *clsic,
						      struct clsic_message *msg)
{
	complete(&msg->completion);
	return CLSIC_MSG_RETAINED;
}

int clsic_send_msg_sync(struct clsic *clsic,
			const union t_clsic_generic_message *fsm_tx,
			union t_clsic_generic_message *fsm_rx,
			const uint8_t *txbuf, const size_t txsize,
			uint8_t *rxbuf, const size_t rxsize)
{
	struct clsic_message *msg;
	int ret = 0;

	msg = clsic_allocate_msg(clsic);
	if (msg == NULL)
		return -ENOMEM;

	memcpy(&msg->fsm.raw, fsm_tx, sizeof(*fsm_tx));

	msg->cb = &clsic_send_msg_sync_cb;

	msg->bulk_txbuf = txbuf;
	msg->bulk_txbuf_marker = (uint8_t *)txbuf;
	msg->bulk_txbuf_maxsize = txsize;

	msg->bulk_rxbuf = rxbuf;
	msg->bulk_rxbuf_maxsize = rxsize;

	ret = clsic_send_message_core(clsic, msg);
	if (ret != 0) {
		clsic_release_msg(clsic, msg);
		return ret;
	}

	/*
	 * Synchronous messages now wait for the message to finish it's
	 * journey.
	 *
	 * This is a potentially interruptible operation, if this context is
	 * signalled then we attempt to remove the message from the system if
	 * possible or wait until it is dealt with.
	 *
	 * The function cannot return until the message layer has completely
	 * finished with the structure.
	 */
	ret = wait_for_completion_interruptible(&msg->completion);
	if (ret == -ERESTARTSYS) {
		clsic_err(clsic, "%p interrupted %d\n", msg,
			  msg->state);
		ret = clsic_send_message_interrupted(clsic, msg);
		if (ret == -EINTR)
			return ret;
	}

	/*
	 * At this point the message has been handled by the messaging layer,
	 * determine an appropriate return value
	 */
	if (CLSIC_GET_MSGSTATE(msg) == CLSIC_MSG_TIMEOUT) {
		/*
		 * Messaging layer detected a timeout - the message was sent
		 * but there was no ACK or response.
		 */
		clsic_dump_message(clsic, msg,
				   "clsic_send_message_core() timed out");
		ret = -ETIMEDOUT;
	} else if (CLSIC_GET_MSGSTATE(msg) != CLSIC_MSG_SUCCESS) {
		/*
		 * General catch of failures - this indicates that there was an
		 * issue with sending the message through the messaging layer
		 * to the device.
		 */
		clsic_err(clsic,
			  "%p message not success ret %d, state %d (%s)\n",
			  msg, ret, msg->state,
			  clsic_message_state_to_string(msg->state));

		clsic_dump_message(clsic, msg,
				   "clsic_send_message_core() failed");

		/*
		 * check whether this message was rejected as invalid or failed
		 * for a different reason
		 */
		if (clsic_system_cmd_is_inval(&msg->response,
				   clsic_get_srv_inst(msg->fsm.cmd.hdr.sbc),
				   msg->fsm.cmd.hdr.msgid))
			/* Device indicated invalid message */
			ret = -EINVAL;
		else if (CLSIC_GET_MSGSTATE(msg) == CLSIC_MSG_INTERRUPTED)
			/* Message cancelled by a device event */
			ret = -EINTR;
		else
			ret = -EIO;
	} else {
		/*
		 * Succeeded - this means that the message passed through the
		 * layer successfully, but the response to the message may
		 * indicate an error.
		 */
		ret = 0;
	}

	memcpy(fsm_rx, msg->response.raw, sizeof(msg->response));

	clsic_release_msg(clsic, msg);

	return ret;
}
EXPORT_SYMBOL_GPL(clsic_send_msg_sync);

int clsic_send_msg_async(struct clsic *clsic,
			 const union t_clsic_generic_message *fsm_tx,
			 const uint8_t *txbuf, const size_t txsize,
			 uint8_t *rxbuf, const size_t rxsize,
			 const uint64_t cookie,
			 enum clsic_message_cb_ret (*cb)(struct clsic *clsic,
						     struct clsic_message *msg))
{
	struct clsic_message *msg;
	int ret;

	/* There must be a callback function */
	if (cb == NULL)
		return -EINVAL;

	msg = clsic_allocate_msg(clsic);
	if (msg == NULL)
		return -ENOMEM;

	msg->cb = cb;
	msg->cookie = cookie;

	memcpy(&msg->fsm, fsm_tx, sizeof(*fsm_tx));

	msg->bulk_txbuf = txbuf;
	msg->bulk_txbuf_marker = (uint8_t *)txbuf;
	msg->bulk_txbuf_maxsize = txsize;

	msg->bulk_rxbuf = rxbuf;
	msg->bulk_rxbuf_maxsize = rxsize;

	ret = clsic_send_message_core(clsic, msg);

	/*
	 * If the message sending encountered an error it may need to be
	 * released in this function - if the message state is INITIAL then the
	 * callback will not have been made, if it isn't then the callback
	 * function is responsible releasing it
	 */
	if ((ret != 0) && (msg->state == CLSIC_MSG_INITIAL))
		clsic_release_msg(clsic, msg);
	return ret;
}
EXPORT_SYMBOL_GPL(clsic_send_msg_async);
