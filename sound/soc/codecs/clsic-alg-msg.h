/*****************************************************************************
 *
 * Copyright (c) 2018
 * Cirrus Logic, Inc. and Cirrus Logic International Semiconductor Ltd.
 * All rights reserved.
 * This software as well as any related documentation is furnished under
 * license and may only be used or copied in accordance with the terms of the
 * license. The information in this file is furnished for informational use
 * only, is subject to change without notice, and should not be construed as
 * a commitment by Cirrus Logic. Cirrus Logic assumes no responsibility or
 * liability for any errors or inaccuracies that may appear in this document or
 * any software that may be provided in association with this document.
 * Except as permitted by such license, no part of this document may be
 * reproduced, stored in a retrieval system, or transmitted in any form or by
 * any means without the express written consent of Cirrus Logic.
 *
 ******************************************************************************/

#ifndef CLSICMESSAGEDEFINES_ALG_H_
#define CLSICMESSAGEDEFINES_ALG_H_

#include <linux/mfd/clsic/clsicmessagedefines.h>

/**
 *  Service type identifier.
 */
#define CLSIC_SRV_TYPE_ALG			(0x4C41)

/**
 *  Service version number.
 */
#define CLSIC_SRV_VERSION_ALG			(0x01000000)

/**
 *  Algorithm service event identifiers
 */
enum clsic_algo_service_event_id {
	CLSIC_ALGOSRV_EVENT_GP0		= 0,
	CLSIC_ALGOSRV_EVENT_VTE		= 1,
	CLSIC_ALGOSRV_EVENT_COUNT	= 2,
};

#endif /* CLSICMESSAGEDEFINES_ALG_H_ */
