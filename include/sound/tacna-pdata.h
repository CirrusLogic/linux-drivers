/*
 * Platform data for Tacna codec driver
 *
 * Copyright 2016-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef TACNA_CODEC_PDATA_H
#define TACNA_CODEC_PDATA_H

#include <linux/kernel.h>

#define TACNA_MAX_INPUT			6
#define TACNA_MAX_MUXED_IN_CHANNELS	4
#define TACNA_MAX_OUTPUT		4
#define TACNA_MAX_OUTPUT_CHANNELS	(TACNA_MAX_OUTPUT * 2)
#define TACNA_MAX_ASP			4
#define TACNA_MAX_DSP			2
#define TACNA_MAX_AUXPDM		3

struct tacna_codec_pdata {
	/**
	 * Maximum number of channels that clocks will be generated for,
	 * useful for systems where an I2S bus with multiple data
	 * lines is mastered.
	 */
	u32 max_channels_clocked[TACNA_MAX_ASP];

	/**
	 * Mode of input structures
	 * One of the TACNA_INMODE_xxx values
	 * Two-dimensional array [input_number][channel number]
	 * Four slots per input in the order:
	 * [n][0]=INnL_1 [n][1]=INnR_1 [n][2]=INnL_2 [n][3]=INnR_2
	 */
	u32 inmode[TACNA_MAX_INPUT][TACNA_MAX_MUXED_IN_CHANNELS];

	/**
	 * Output mono mode control
	 * For each output set the value to TRUE to indicate that
	 * the output is mono
	 * For CS47L94/CS47L95:
	 *   [0]=OUT1_HP1, [1]=OUT2, [2] = OUT1_HP2, [3] = OUTH ...
	 * For all other codecs:
	 *   [0]=OUT1 [1]=OUT2 [2]=OUT3 ...
	 */
	bool out_mono[TACNA_MAX_OUTPUT];

	/**
	 * PDM mute control settings.
	 * One of the TACNA_PDM_MUTE_xxx values.
	 */
	u32 pdm_mute;

	/**
	 * PDM speaker format
	 * One of the TACNA_PDM_FMT_MODE_x values
	 */
	u32 pdm_fmt;

	/** Auxiliary PDM configuration options. */
	bool auxpdm_slave_mode[TACNA_MAX_AUXPDM];
	bool auxpdm_falling_edge[TACNA_MAX_AUXPDM];
};

#endif
