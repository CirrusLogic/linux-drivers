/*
 * Device Tree defines for Tacna codec extcon driver
 *
 * Copyright 2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DT_BINDINGS_EXTCON_TACNA_H
#define DT_BINDINGS_EXTCON_TACNA_H

/* output clamp pin for cirrus,hpd-pins */
#define TACNA_HPD_OUT_OUT1L			0x0
#define TACNA_HPD_OUT_OUT1R			0x1
#define TACNA_HPD_OUT_OUT2L			0x2
#define TACNA_HPD_OUT_OUT2R			0x3
#define TACNA_HPD_OUT_OUT3L			0x4
#define TACNA_HPD_OUT_OUT3R			0x5

/* Sense pin selection for cirrus,hpd-pins */
#define TACNA_HPD_SENSE_MICDET1			0x0
#define TACNA_HPD_SENSE_MICDET2			0x1
#define TACNA_HPD_SENSE_MICDET3			0x2
#define TACNA_HPD_SENSE_MICDET4			0x3
#define TACNA_HPD_SENSE_MICDET5			0x4
#define TACNA_HPD_SENSE_HPDET1			0x8
#define TACNA_HPD_SENSE_HPDET2			0x9
#define TACNA_HPD_SENSE_JD1			0xc
#define TACNA_HPD_SENSE_JD2			0xd

/* HP gnd setting for cirrus,micd-configs */
#define TACNA_HPD_GND_HPOUTFB1			0x0
#define TACNA_HPD_GND_HPOUTFB2			0x1
#define TACNA_HPD_GND_HPOUTFB3			0x2
#define TACNA_HPD_GND_HPOUTFB4			0x3
#define TACNA_HPD_GND_HPOUTFB5			0x4

/* source setting for cirrus,micd-configs */
#define TACNA_MICD_SENSE_MICDET1		0x0
#define TACNA_MICD_SENSE_MICDET2		0x1
#define TACNA_MICD_SENSE_MICDET3		0x2
#define TACNA_MICD_SENSE_MICDET4		0x3
#define TACNA_MICD_SENSE_MICDET5		0x4
#define TACNA_MICD_SENSE_HPDET1			0x8
#define TACNA_MICD_SENSE_HPDET2			0x9
#define TACNA_MICD_SENSE_JD1			0xc
#define TACNA_MICD_SENSE_JD2			0xd

/* ground setting for cirrus,micd-configs */
#define TACNA_MICD_GND_MICDET1			0x0
#define TACNA_MICD_GND_MICDET2			0x1
#define TACNA_MICD_GND_MICDET3			0x2
#define TACNA_MICD_GND_MICDET4			0x3
#define TACNA_MICD_GND_MICDET5			0x4

/* bias setting for cirrus,micd-configs */
#define TACNA_MICD_BIAS_SRC_MICBIAS1A		0x0
#define TACNA_MICD_BIAS_SRC_MICBIAS1B		0x1
#define TACNA_MICD_BIAS_SRC_MICBIAS1C		0x2
#define TACNA_MICD_BIAS_SRC_MICBIAS1D		0x3
#define TACNA_MICD_BIAS_SRC_MICBIAS2A		0x4
#define TACNA_MICD_BIAS_SRC_MICBIAS2B		0x5
#define TACNA_MICD_BIAS_SRC_MICVDD		0xf

/* clamp mode settings */
#define TACNA_MICD_CLAMP_MODE_DISABLED		0x0
#define TACNA_MICD_CLAMP_MODE_ACTIVE		0x1
#define TACNA_MICD_CLAMP_MODE_JD1L		0x4
#define TACNA_MICD_CLAMP_MODE_JD1H		0x5
#define TACNA_MICD_CLAMP_MODE_JD2L		0x6
#define TACNA_MICD_CLAMP_MODE_JD2H		0x7
#define TACNA_MICD_CLAMP_MODE_JD1L_OR_JD2L	0x8
#define TACNA_MICD_CLAMP_MODE_JD1L_OR_JD2H	0x9
#define TACNA_MICD_CLAMP_MODE_JD1H_OR_JD2L	0xa
#define TACNA_MICD_CLAMP_MODE_JD1H_OR_JD2H	0xb
#define TACNA_MICD_CLAMP_MODE_JD1L_AND_JD2L	0xc
#define TACNA_MICD_CLAMP_MODE_JD1L_AND_JD2H	0xd
#define TACNA_MICD_CLAMP_MODE_JD1H_AND_JD2L	0xe
#define TACNA_MICD_CLAMP_MODE_JD1H_AND_JD2H	0xf

#endif
