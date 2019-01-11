/*
 * message.h -- CLSIC message interface
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CIRRUS_CLSIC_MESSAGE_H
#define CIRRUS_CLSIC_MESSAGE_H

#include <linux/completion.h>

#include <linux/mfd/clsic/core.h>

/**
 * This extension to clsicmessagedefines.h is used for handling and
 * identifying messages
 */
union t_clsic_generic_message {
	uint8_t raw[CLSIC_FIXED_MSG_SZ];

	struct {
		struct clsic_cmd_hdr hdr;
		uint8_t payload[10];
	} __packed cmd;

	struct {
		struct clsic_blkcmd_hdr hdr;
		uint8_t payload[6];
	} __packed bulk_cmd;

	struct {
		struct clsic_rsp_hdr hdr;
		uint8_t payload[9];
	} __packed rsp;

	struct {
		struct clsic_blkrsp_hdr hdr;
		uint8_t payload[5];
	} __packed bulk_rsp;

	struct {
		struct clsic_ack_hdr hdr;
		uint8_t payload[10];
	} __packed ack;

	struct {
		struct clsic_nty_hdr hdr;
		uint8_t payload[10];
	} __packed notif;

	struct {
		struct clsic_blknty_hdr hdr;
		uint8_t payload[6];
	} __packed bulk_notif;
} PACKED;


enum clsic_message_states {
	CLSIC_MSG_INITIAL,	/* Allocated */
	CLSIC_MSG_WAITING,	/* Waiting to send */
	CLSIC_MSG_SENDING,	/* Message is currently being sent */
	CLSIC_MSG_SENT,		/* Message sent to CLSIC */
	CLSIC_MSG_ACK,		/* Message ack'd by CLSIC (async) */
	CLSIC_MSG_BULKTX,	/* Transmitting bulk data */
	CLSIC_MSG_BULKTX_WAIT,	/* Waiting to send bulk data */
	CLSIC_MSG_BULKRX,	/* Reading bulk data */
	CLSIC_MSG_SUCCESS,	/* Exchange succeeded */
	CLSIC_MSG_FAILED,	/* Exchange failed */
	CLSIC_MSG_INTERRUPTED,	/* Exchange interrupted */
	CLSIC_MSG_TIMEOUT,	/* Exchange timed out */
};

#define CLSIC_GET_MSGSTATE(msg)			((msg)->state)

static inline const char *clsic_message_state_to_string(uint8_t state)
{
	switch (state) {
	case CLSIC_MSG_INITIAL:
		return "INITIAL";
	case CLSIC_MSG_WAITING:
		return "WAITING";
	case CLSIC_MSG_SENDING:
		return "SENDING";
	case CLSIC_MSG_SENT:
		return "SENT";
	case CLSIC_MSG_ACK:
		return "ACKd";
	case CLSIC_MSG_BULKTX:
		return "BULKTX";
	case CLSIC_MSG_BULKTX_WAIT:
		return "BULKTX_WAIT";
	case CLSIC_MSG_BULKRX:
		return "BULKRX";
	case CLSIC_MSG_SUCCESS:
		return "SUCCEEDED";
	case CLSIC_MSG_FAILED:
		return "FAILED";
	case CLSIC_MSG_INTERRUPTED:
		return "INTERRUPTED";
	case CLSIC_MSG_TIMEOUT:
		return "TIMEOUT";
	default:
		return "UNKNOWN";
	}
}

/*
 * Constants for use when referring to fixed size message bytes
 * (Indexing and addressing the FSM byte arrays)
 */
#define CLSIC_FSM0		0
#define CLSIC_FSM1		1
#define CLSIC_FSM2		2
#define CLSIC_FSM3		3
#define CLSIC_FSM4		4
#define CLSIC_FSM5		5
#define CLSIC_FSM6		6
#define CLSIC_FSM7		7
#define CLSIC_FSM8		8
#define CLSIC_FSM9		9
#define CLSIC_FSM10		10
#define CLSIC_FSM11		11

/*
 * Simple enumeration indicating whether the client of the message callback has
 * finished with a message or not.
 *
 * If they retain a copy of the message then they need to explicitly call
 * clsic_msg_release().
 *
 * If they do not wish to retain the message then it is immediately freed.
 */
enum clsic_message_cb_ret {
	CLSIC_MSG_RETAINED,
	CLSIC_MSG_RELEASED,
};

struct clsic_message {
	/* buffer for a fixed sized message */
	union t_clsic_generic_message fsm;

	enum clsic_message_states state;

	/* Length in seconds before this message should timeout */
	uint8_t timeout;

	/*
	 * Synchronous method - completion so a caller can sleep waiting for
	 * message event
	 */
	struct completion completion;

	/*
	 * Asynchronous callback so that a client can be notified when this
	 * message is finished
	 */
	enum clsic_message_cb_ret (*cb)(struct clsic *clsic,
					       struct clsic_message *msg);

	/* Possible value supplied by client */
	uint64_t cookie;

	/*
	 * Links for putting this message into lists; the private_link is for
	 * the use of the messaging layer and the client_link is for the use of
	 * service handlers receiving async commands
	 */
	struct list_head private_link;
	struct list_head client_link;

	/* Buffer for the last fixed message response */
	union t_clsic_generic_message response;

	/* Bulk transfer members */
	const uint8_t *bulk_txbuf;
	uint8_t *bulk_txbuf_marker;	/* next buffer byte to send */
	uint32_t bulk_txbuf_maxsize;	/* size of buffer in bytes */
	uint8_t *bulk_rxbuf;
	uint32_t bulk_rxbuf_maxsize;	/* size of buffer in bytes */
};

/* Header layout and access functions */
static inline bool clsic_get_bulkbit(const struct clsic_message *msg)
{
	struct clsic_blkcmd_hdr *hdr;

	if (!msg)
		return false;

	hdr = (struct clsic_blkcmd_hdr *) &msg->fsm;

	return clsic_get_bulk_bit(hdr->sbc) == 1;
}

static inline uint8_t clsic_get_servinst(const struct clsic_message *msg)
{
	struct clsic_blkcmd_hdr *hdr;

	if (!msg)
		return 0;

	hdr = (struct clsic_blkcmd_hdr *) &msg->fsm;

	return clsic_get_srv_inst(hdr->sbc);
}

static inline uint32_t clsic_get_bulk_sz(union t_clsic_generic_message
					 *msg_p)
{
	const struct clsic_blkcmd_hdr *hdr =
		(struct clsic_blkcmd_hdr *) &msg_p->cmd.hdr;

	/* If this isn't a bulk message there is no bulk payload */
	if (clsic_get_bulk_bit(msg_p->cmd.hdr.sbc) == 0)
		return 0;

	if (clsic_get_cran(hdr->sbc) == CLSIC_CRAN_CMD)
		return msg_p->bulk_cmd.hdr.bulk_sz;

	if (clsic_get_cran(hdr->sbc) == CLSIC_CRAN_RSP)
		return msg_p->bulk_rsp.hdr.bulk_sz;

	if (clsic_get_cran(hdr->sbc) == CLSIC_CRAN_NTY)
		return msg_p->bulk_notif.hdr.bulk_sz;

	return 0;
}

static inline uint8_t clsic_get_cran_frommsg(const struct clsic_message *msg)
{
	struct clsic_blkcmd_hdr *hdr;

	if (!msg)
		return 0;

	hdr = (struct clsic_blkcmd_hdr *) &msg->fsm;

	return clsic_get_cran(hdr->sbc);
}

static inline const char clsic_cran_to_char(uint8_t cran)
{
	switch (cran) {
	case CLSIC_CRAN_CMD:
		return 'C';
	case CLSIC_CRAN_RSP:
		return 'R';
	case CLSIC_CRAN_ACK:
		return 'A';
	case CLSIC_CRAN_NTY:
		return 'N';
	default:
		return '?';
	}
}

static inline uint8_t clsic_get_messageid(const struct clsic_message *msg)
{
	struct clsic_blkcmd_hdr *hdr;

	if (!msg)
		return 0;

	hdr = (struct clsic_blkcmd_hdr *) &msg->fsm;

	return hdr->msgid;
}

/*
 * Utility function; do the messages have matching service instances and
 * message ids?
 */
static inline bool clsic_compare(const struct clsic *clsic,
				 const struct clsic_message *msg_a,
				 const struct clsic_message *msg_b)
{
	if (!msg_a)
		return false;
	if (!msg_b)
		return false;

	if (clsic_get_messageid(msg_a) != clsic_get_messageid(msg_b))
		return false;

	if (clsic_get_servinst(msg_a) != clsic_get_servinst(msg_b))
		return false;

	return true;
}

/*
 * Utility function; does a given message match the given parameters?
 */
static inline bool clsic_ismatch(const struct clsic *clsic,
				 const struct clsic_message *msg,
				 const uint8_t svcinst,
				 const uint8_t msgid)
{
	if (!msg)
		return false;

	if (msgid != clsic_get_messageid(msg))
		return false;

	if (svcinst != clsic_get_servinst(msg))
		return false;

	return true;
}

void clsic_dump_message(const struct clsic *clsic,
			const struct clsic_message *msg,
			const char *string);

int clsic_setup_message_interface(struct clsic *clsic);
void clsic_shutdown_message_interface(struct clsic *clsic);

/*
 * Utility constants for unused message sending parameters for code readability
 */
#define CLSIC_NO_RXBUF		NULL
#define CLSIC_NO_RXBUF_LEN	0
#define CLSIC_NO_TXBUF		NULL
#define CLSIC_NO_TXBUF_LEN	0

/*
 * Synchronous message sending - send the given fsm message and waits until the
 * entire message exchange is completed, returning response in provided memory.
 * When this function returns the exchange has completed and the message is no
 * longer in the messaging layer.
 *
 * The parameter fsm_tx is the 12 byte fixed sized message that will be sent to
 * the device.
 *
 * The 12 byte fixed sized response message will be stored in the fsm_rx
 * parameter.
 *
 * This function has optional parameters for bulk transmission; if the txbuf is
 * specified then it will be used to send the bulk payload, likewise rxbuf is
 * used to store the bulk portion of the bulk response.
 *
 * Error conditions may be indicated by the return code; (XXX TBC -
 * -EINVAL = invalid message parameters
 * -ENOMEM = internal allocation failed
 * -ENXIO = device not in a state where the message can be sent
 * -ETIMEDOUT = the message timed out
 * -EINTR = message sending interrupted (explicitly cancelled or the device
 *  panic'd))
 *
 * Only one message with a given service instance and message id combination is
 * permitted to be within the messaging layer at any point in time, subsequent
 * calls will respond with -EINVAL.
 *
 * This message will be queued for sending and this request may be cancelled up
 * to the point where the message begins to be sent, after that point it
 * becomes uninterruptible.
 *
 * If no rxbuf is specified and the response is a bulk message then this
 * message exchange will fail - the bulk payload will be discarded and the
 * function will return an failure error code (XXX define the error code).
 */
int clsic_send_msg_sync(struct clsic *clsic,
			const union t_clsic_generic_message *fsm_tx,
			union t_clsic_generic_message *fsm_rx,
			const uint8_t *txbuf, const size_t txsize,
			uint8_t *rxbuf, const size_t rxsize);

/*
 * The clsic_send_msg_sync_pm() function has the same parameters and behaviour
 * as the regular clsic_send_msg_sync() function except it is bookended with
 * pm_runtime get and put calls
 */
static inline int clsic_send_msg_sync_pm(struct clsic *clsic,
			const union t_clsic_generic_message *fsm_tx,
			union t_clsic_generic_message *fsm_rx,
			const uint8_t *txbuf, const size_t txsize,
			uint8_t *rxbuf, const size_t rxsize)
{
	int ret;

	pm_runtime_get_sync(clsic->dev);
	ret = clsic_send_msg_sync(clsic, fsm_tx, fsm_rx,
				  txbuf, txsize, rxbuf, rxsize);
	pm_runtime_put_autosuspend(clsic->dev);

	return ret;
}

/*
 * Asynchronous message sending - send the given message and waits until this
 * message receives an ACK or Response (unblocking the messaging protocol) at
 * which point this function returns to the caller. The caller will be notified
 * of a change in the state of this message via callback.
 *
 * The parameters follow that of the synchronous function with the addition of
 * a callback function and a cookie and the fsm_rx buffer is not provided.
 *
 * When the exchange associated with this message is completed then the message
 * structure shall be stored on a completed list within the messaging layer
 * where it will prevent the sending of similar messages (to the same service
 * instance and message id combination) until it is released.
 *
 * The caller guarantees that the fsm_tx, txbuf and rxbuf will persist over the
 * life of the message exchange, releasing or reusing this memory will result
 * in undefined behaviour.
 *
 * The callback function will be invoked when the response is received or in
 * error conditions. The callback has two parameters, the clsic instance
 * structure and the message structure.
 *
 * The return code from the client callback indicates whether the client has
 * retained the message - retained messages are not immediately released by the
 * messaging layer and the client must call clsic_release_msg() on the
 * structure when it has finished with it.
 *
 * The supplied cookie is stashed in the message structure.
 *
 * In the asynchronous version of this function the rxbuf is an optional
 * parameter, the messaging layer will attempt to dynamically allocate the
 * memory required to store the response bulk payload. Errors in allocation
 * will cause the bulk payload to be discarded and the message will be
 * completed with the error state set.
 */
int clsic_send_msg_async(struct clsic *clsic,
			 const union t_clsic_generic_message *fsm_tx,
			 const uint8_t *txbuf, const size_t txsize,
			 uint8_t *rxbuf, const size_t rxsize,
			 const uint64_t cookie,
			 enum clsic_message_cb_ret (*cb)(struct clsic *clsic,
						    struct clsic_message *msg));

/* Utility function to dispose of a message structure */
void clsic_release_msg(struct clsic *clsic, struct clsic_message *msg);

void clsic_purge_message_queues(struct clsic *clsic);

void clsic_handle_incoming_messages(struct clsic *clsic);

void clsic_handle_message_rxdma_status(struct clsic *clsic,
				       struct clsic_message *msg);

void clsic_handle_message_invalid_cmd(struct clsic *clsic,
				      struct clsic_message *msg);

int clsic_fifo_readbulk_payload(struct clsic *clsic,
				struct clsic_message *msg,
				uint8_t *dest,
				size_t maxsize);

void clsic_msgproc_shutdown_schedule(struct clsic *clsic);
bool clsic_msgproc_shutdown_cancel(struct clsic *clsic, bool sync);
void clsic_msgproc_use(struct clsic *clsic, uint8_t service_instance);
void clsic_msgproc_release(struct clsic *clsic, uint8_t service_instance);

static inline void clsic_init_message(union t_clsic_generic_message *msg,
				      const uint8_t service_instance,
				      const uint8_t msgid)
{
		memset(msg, 0, CLSIC_FIXED_MSG_SZ);

		/*
		 * Clearing the structure will set the CRAN message type to CMD
		 * (as that is represented as all bits clear) and clear the
		 * bulk bit.
		 *
		 * clsic_set_cran(&msg->cmd.hdr.sbc, CLSIC_CRAN_CMD);
		 * clsic_set_bulk(&msg->cmd.hdr.sbc, 0);
		 */

		clsic_set_srv_inst(&msg->cmd.hdr.sbc, service_instance);
		msg->cmd.hdr.msgid = msgid;
}

#endif
