/* SPDX-License-Identifier: GPL-2.0
 *
 * CS40L50 Advanced Haptic Driver with waveform memory,
 * integrated DSP, and closed-loop algorithms
 *
 * Copyright 2024 Cirrus Logic, Inc.
 *
 * Author: James Ogletree <james.ogletree@cirrus.com>
 */

#ifndef __MFD_CS40L50_H__
#define __MFD_CS40L50_H__

#include <linux/firmware/cirrus/cs_dsp.h>
#include <linux/gpio/consumer.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>

/* Power Supply Configuration */
#define CS40L50_BLOCK_ENABLES2		0x201C
#define CS40L50_ERR_RLS			0x2034
#define CS40L50_BST_LPMODE_SEL		0x3810
#define CS40L50_DCM_LOW_POWER		0x1
#define CS40L50_OVERTEMP_WARN		0x4000010

/* Interrupts */
#define CS40L50_IRQ1_INT_1		0xE010
#define CS40L50_IRQ1_BASE		CS40L50_IRQ1_INT_1
#define CS40L50_IRQ1_INT_2		0xE014
#define CS40L50_IRQ1_INT_8		0xE02C
#define CS40L50_IRQ1_INT_9		0xE030
#define CS40L50_IRQ1_INT_10		0xE034
#define CS40L50_IRQ1_INT_18		0xE054
#define CS40L50_IRQ1_MASK_1		0xE090
#define CS40L50_IRQ1_MASK_2		0xE094
#define CS40L50_IRQ1_MASK_20		0xE0DC
#define CS40L50_IRQ1_INT_1_OFFSET	(CS40L50_IRQ1_INT_1 - CS40L50_IRQ1_BASE)
#define CS40L50_IRQ1_INT_2_OFFSET	(CS40L50_IRQ1_INT_2 - CS40L50_IRQ1_BASE)
#define CS40L50_IRQ1_INT_8_OFFSET	(CS40L50_IRQ1_INT_8 - CS40L50_IRQ1_BASE)
#define CS40L50_IRQ1_INT_9_OFFSET	(CS40L50_IRQ1_INT_9 - CS40L50_IRQ1_BASE)
#define CS40L50_IRQ1_INT_10_OFFSET	(CS40L50_IRQ1_INT_10 - CS40L50_IRQ1_BASE)
#define CS40L50_IRQ1_INT_18_OFFSET	(CS40L50_IRQ1_INT_18 - CS40L50_IRQ1_BASE)
#define CS40L50_IRQ_MASK_2_OVERRIDE	0xFFDF7FFF
#define CS40L50_IRQ_MASK_20_OVERRIDE	0x15C01000
#define CS40L50_AMP_SHORT_MASK		BIT(31)
#define CS40L50_DSP_QUEUE_MASK		BIT(21)
#define CS40L50_TEMP_ERR_MASK		BIT(31)
#define CS40L50_BST_UVP_MASK		BIT(6)
#define CS40L50_BST_SHORT_MASK		BIT(7)
#define CS40L50_BST_ILIMIT_MASK		BIT(18)
#define CS40L50_UVLO_VDDBATT_MASK	BIT(16)
#define CS40L50_GLOBAL_ERROR_MASK	BIT(15)

enum cs40l50_irq_list {
	CS40L50_DSP_QUEUE_IRQ,
	CS40L50_GLOBAL_ERROR_IRQ,
	CS40L50_UVLO_VDDBATT_IRQ,
	CS40L50_BST_ILIMIT_IRQ,
	CS40L50_BST_SHORT_IRQ,
	CS40L50_BST_UVP_IRQ,
	CS40L50_TEMP_ERR_IRQ,
	CS40L50_AMP_SHORT_IRQ,
};

/* DSP */
#define CS40L50_XMEM_PACKED_0		0x2000000
#define CS40L50_XMEM_UNPACKED24_0	0x2800000
#define CS40L50_SYS_INFO_ID		0x25E0000
#define CS40L50_DSP_QUEUE_WT		0x28042C8
#define CS40L50_DSP_QUEUE_RD		0x28042CC
#define CS40L50_NUM_WAVES		0x2805C18
#define CS40L50_CORE_BASE		0x2B80000
#define CS40L50_YMEM_PACKED_0		0x2C00000
#define CS40L50_YMEM_UNPACKED24_0	0x3400000
#define CS40L50_PMEM_0			0x3800000
#define CS40L50_DSP_POLL_US		1000
#define CS40L50_DSP_TIMEOUT_COUNT	100
#define CS40L50_RESET_PULSE_US		2200
#define CS40L50_CP_READY_US		3100
#define CS40L50_AUTOSUSPEND_MS		2000
#define CS40L50_PM_ALGO			0x9F206
#define CS40L50_HAPTICS_ALGO		0x9013C
#define CS40L50_GLOBAL_ERR_RLS_SET	BIT(11)
#define CS40L50_GLOBAL_ERR_RLS_CLEAR	0

enum cs40l50_wseqs {
	CS40L50_PWR_ON,
	CS40L50_STANDBY,
	CS40L50_ACTIVE,
	CS40L50_NUM_WSEQS,
};

/* DSP Queue */
#define CS40L50_DSP_QUEUE_BASE		0x11004
#define CS40L50_DSP_QUEUE_END		0x1101C
#define CS40L50_DSP_QUEUE		0x11020
#define CS40L50_PREVENT_HIBER		0x2000003
#define CS40L50_ALLOW_HIBER		0x2000004
#define CS40L50_SHUTDOWN		0x2000005
#define CS40L50_SYSTEM_RESET		0x2000007
#define CS40L50_START_I2S		0x3000002
#define CS40L50_OWT_PUSH		0x3000008
#define CS40L50_STOP_PLAYBACK		0x5000000
#define CS40L50_OWT_DELETE		0xD000000

/* Firmware files */
#define CS40L50_FW			"cs40l50.wmfw"
#define CS40L50_WT			"cs40l50.bin"

/* Device */
#define CS40L50_DEVID			0x0
#define CS40L50_REVID			0x4
#define CS40L50_DEVID_A			0x40A50
#define CS40L51_DEVID			0x40A51
#define CS40L52_DEVID			0x40A52
#define CS40L53_DEVID			0x40A53
#define CS40L50_REVID_B0		0xB0

/* Broadcast */
#define CS40L50_CTRL_I2C_BROADCAST			0x168
#define CS40L50_I2C_BROADCAST_ENABLE_MASK	BIT(15)
#define CS40L50_I2C_BROADCAST_ADDR_SHIFT	1
#define CS40L50_I2C_BROADCAST_ADDR_MAX		0x78
#define CS40L50_I2C_BROADCAST_ADDR_MIN		0x8
#define CS40L50_BROADCAST_TRIGGER_MASK		0x8000

struct cs40l50 {
	struct device *dev;
	struct regmap *regmap;
	struct regmap *broadcast_regmap;
	struct mutex lock;
	struct cs_dsp dsp;
	struct gpio_desc *reset_gpio;
	struct regmap_irq_chip_data *irq_data;
	const struct firmware *fw;
	const struct firmware *bin;
	const char *bin_filename;
	struct cs_dsp_wseq wseqs[CS40L50_NUM_WSEQS];
	struct i2c_client *broadcast_client;
	bool external_boost;
	int i2c_id;
	int irq;
	u32 devid;
	u32 revid;
	u32 broadcast_addr;
};

int cs40l50_dsp_write(struct device *dev, struct regmap *regmap, u32 val);
int cs40l50_set_dsp_gain(struct device *dev, u16 gain_pct);
int cs40l50_probe(struct cs40l50 *cs40l50);
int cs40l50_remove(struct cs40l50 *cs40l50);
bool cs40l50_broadcast_readable_reg(struct device *dev, unsigned int reg);
bool cs40l50_broadcast_writeable_reg(struct device *dev, unsigned int reg);

extern const struct regmap_config cs40l50_regmap;
extern const struct dev_pm_ops cs40l50_pm_ops;

#define CS40L50_NUM_PCT_MAP_VALUES	101

const u32 cs40l50_attn_q21_2_vals[CS40L50_NUM_PCT_MAP_VALUES] = {
	400, /* MUTE */
	160, /* 1% */
	136, 122, 112, 104, 98, 92, 88, 84, 80, 77, 74, 71,
	68, 66, 64, 62, 60, 58, 56, 54, 53, 51, 50, 48, /* 25% */
	47, 45, 44, 43, 42, 41, 40, 39, 37, 36, 35, 35, 34,
	33, 32, 31, 30, 29, 29, 28, 27, 26, 26, 25, 24, /* 50 % */
	23, 23, 22, 21, 21, 20, 20, 19, 18, 18, 17, 17, 16,
	16, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, /* 75% */
	10, 9, 9, 8, 8, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 0, 0, /* 100% */
};

#endif /* __MFD_CS40L50_H__ */
