/*
 * Device Tree defines for Madera codecs
 *
 * Copyright 2016-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DT_BINDINGS_SOUND_MADERA_H
#define DT_BINDINGS_SOUND_MADERA_H

#define MADERA_INMODE_DIFF		0
#define MADERA_INMODE_SE		1
#define MADERA_INMODE_DMIC		2

#define MADERA_DMIC_REF_MICVDD		0
#define MADERA_DMIC_REF_MICBIAS1	1
#define MADERA_DMIC_REF_MICBIAS2	2
#define MADERA_DMIC_REF_MICBIAS3	3

#define CS47L35_DMIC_REF_MICBIAS1B	1
#define CS47L35_DMIC_REF_MICBIAS2A	2
#define CS47L35_DMIC_REF_MICBIAS2B	3

#endif
