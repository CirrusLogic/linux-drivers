/*
 * linux/sound/cs35l35.h -- Platform data for CS35l35
 *
 * Copyright (c) 2016 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS35L35_H
#define __CS35L35_H

struct classh_cfg {
	/*
	 * Class H Algorithm Control Variables
	 * You can either have it done
	 * automatically or you can adjust
	 * these variables for tuning
	 *
	 * if you do not enable the internal algorithm
	 * you will get a set of mixer controls for
	 * Class H tuning
	 *
	 * Section 4.3 of the datasheet
	 */
	/* Internal ClassH Algorithm  */
	bool classh_bst_override;
	bool classh_algo_enable;
	int classh_bst_max_limit;
	int classh_mem_depth;
	int classh_release_rate;
	int classh_headroom;
	int classh_wk_fet_disable;
	int classh_wk_fet_delay;
	int classh_wk_fet_thld;
	int classh_vpch_auto;
	int classh_vpch_rate;
	int classh_vpch_man;
};

struct cs35l35_platform_data {

	/* Stereo (2 Device) */
	bool stereo;
	/* serial port drive strength */
	int sp_drv_str;
	/* Boost Power Down with FET */
	bool bst_pdn_fet_on;
	/* Boost Voltage : used if ClassH Algo Enabled */
	int bst_vctl;
	/* Amp Gain Zero Cross */
	bool gain_zc;
	/* Audio Input Location */
	int aud_channel;
	/* Advisory Input Location */
	int adv_channel;
	/* Shared Boost for stereo */
	bool shared_bst;
	/* ClassH Algorithm */
	struct classh_cfg classh_algo;
};

#endif /* __CS35L35_H */
