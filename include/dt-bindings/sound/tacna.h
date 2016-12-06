/*
 * Device Tree defines for Tacna codecs
 *
 * Copyright 2016-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DT_BINDINGS_SOUND_TACNA_H
#define DT_BINDINGS_SOUND_TACNA_H

#define TACNA_INMODE_DIFF	0
#define TACNA_INMODE_SE		1
#define TACNA_INMODE_DMIC	2

#define TACNA_PDM_FMT_MODE_A_LSB_FIRST	0x0000
#define TACNA_PDM_FMT_MODE_B_LSB_FIRST	0x4000
#define TACNA_PDM_FMT_MODE_A_MSB_FIRST	0x8000
#define TACNA_PDM_FMT_MODE_B_MSB_FIRST	0xc000

#endif
