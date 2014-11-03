/*
 * Copyright © 2006-2007 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 */

#include <linux/cpufreq.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vgaarb.h>
#include <drm/drm_edid.h>
#include <drm/drmP.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "i915_trace.h"
#include <drm/drm_dp_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/dma_remapping.h>

bool intel_pipe_has_type(struct drm_crtc *crtc, int type);
static void intel_increase_pllclock(struct drm_crtc *crtc);
static void intel_crtc_update_cursor(struct drm_crtc *crtc, bool on);

typedef struct {
	/* given values */
	int n;
	int m1, m2;
	int p1, p2;
	/* derived values */
	int	dot;
	int	vco;
	int	m;
	int	p;
} intel_clock_t;

typedef struct {
	int	min, max;
} intel_range_t;

typedef struct {
	int	dot_limit;
	int	p2_slow, p2_fast;
} intel_p2_t;

#define INTEL_P2_NUM		      2
typedef struct intel_limit intel_limit_t;
struct intel_limit {
	intel_range_t   dot, vco, n, m, m1, m2, p, p1;
	intel_p2_t	    p2;
	/**
	 * find_pll() - Find the best values for the PLL
	 * @limit: limits for the PLL
	 * @crtc: current CRTC
	 * @target: target frequency in kHz
	 * @refclk: reference clock frequency in kHz
	 * @match_clock: if provided, @best_clock P divider must
	 *               match the P divider from @match_clock
	 *               used for LVDS downclocking
	 * @best_clock: best PLL values found
	 *
	 * Returns true on success, false on failure.
	 */
	bool (*find_pll)(const intel_limit_t *limit,
			 struct drm_crtc *crtc,
			 int target, int refclk,
			 intel_clock_t *match_clock,
			 intel_clock_t *best_clock);
};

/* FDI */
#define IRONLAKE_FDI_FREQ		2700000 /* in kHz for mode->clock */

int
intel_pch_rawclk(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	WARN_ON(!HAS_PCH_SPLIT(dev));

	return I915_READ(PCH_RAWCLK_FREQ) & RAWCLK_FREQ_MASK;
}

static bool
intel_find_best_PLL(const intel_limit_t *limit, struct drm_crtc *crtc,
		    int target, int refclk, intel_clock_t *match_clock,
		    intel_clock_t *best_clock);
static bool
intel_g4x_find_best_PLL(const intel_limit_t *limit, struct drm_crtc *crtc,
			int target, int refclk, intel_clock_t *match_clock,
			intel_clock_t *best_clock);

static bool
intel_find_pll_g4x_dp(const intel_limit_t *, struct drm_crtc *crtc,
		      int target, int refclk, intel_clock_t *match_clock,
		      intel_clock_t *best_clock);
static bool
intel_find_pll_ironlake_dp(const intel_limit_t *, struct drm_crtc *crtc,
			   int target, int refclk, intel_clock_t *match_clock,
			   intel_clock_t *best_clock);

static bool
intel_vlv_find_best_pll(const intel_limit_t *limit, struct drm_crtc *crtc,
			int target, int refclk, intel_clock_t *match_clock,
			intel_clock_t *best_clock);

static inline u32 /* units of 100MHz */
intel_fdi_link_freq(struct drm_device *dev)
{
	if (IS_GEN5(dev)) {
		struct drm_i915_private *dev_priv = dev->dev_private;
		return (I915_READ(FDI_PLL_BIOS_0) & FDI_PLL_FB_CLOCK_MASK) + 2;
	} else
		return 27;
}

static const intel_limit_t intel_limits_i8xx_dvo = {
	.dot = { .min = 25000, .max = 350000 },
	.vco = { .min = 930000, .max = 1400000 },
	.n = { .min = 3, .max = 16 },
	.m = { .min = 96, .max = 140 },
	.m1 = { .min = 18, .max = 26 },
	.m2 = { .min = 6, .max = 16 },
	.p = { .min = 4, .max = 128 },
	.p1 = { .min = 2, .max = 33 },
	.p2 = { .dot_limit = 165000,
		.p2_slow = 4, .p2_fast = 2 },
	.find_pll = intel_find_best_PLL,
};

static const intel_limit_t intel_limits_i8xx_lvds = {
	.dot = { .min = 25000, .max = 350000 },
	.vco = { .min = 930000, .max = 1400000 },
	.n = { .min = 3, .max = 16 },
	.m = { .min = 96, .max = 140 },
	.m1 = { .min = 18, .max = 26 },
	.m2 = { .min = 6, .max = 16 },
	.p = { .min = 4, .max = 128 },
	.p1 = { .min = 1, .max = 6 },
	.p2 = { .dot_limit = 165000,
		.p2_slow = 14, .p2_fast = 7 },
	.find_pll = intel_find_best_PLL,
};

static const intel_limit_t intel_limits_i9xx_sdvo = {
	.dot = { .min = 20000, .max = 400000 },
	.vco = { .min = 1400000, .max = 2800000 },
	.n = { .min = 1, .max = 6 },
	.m = { .min = 70, .max = 120 },
	.m1 = { .min = 8, .max = 18 },
	.m2 = { .min = 3, .max = 7 },
	.p = { .min = 5, .max = 80 },
	.p1 = { .min = 1, .max = 8 },
	.p2 = { .dot_limit = 200000,
		.p2_slow = 10, .p2_fast = 5 },
	.find_pll = intel_find_best_PLL,
};

static const intel_limit_t intel_limits_i9xx_lvds = {
	.dot = { .min = 20000, .max = 400000 },
	.vco = { .min = 1400000, .max = 2800000 },
	.n = { .min = 1, .max = 6 },
	.m = { .min = 70, .max = 120 },
	.m1 = { .min = 10, .max = 22 },
	.m2 = { .min = 5, .max = 9 },
	.p = { .min = 7, .max = 98 },
	.p1 = { .min = 1, .max = 8 },
	.p2 = { .dot_limit = 112000,
		.p2_slow = 14, .p2_fast = 7 },
	.find_pll = intel_find_best_PLL,
};


static const intel_limit_t intel_limits_g4x_sdvo = {
	.dot = { .min = 25000, .max = 270000 },
	.vco = { .min = 1750000, .max = 3500000},
	.n = { .min = 1, .max = 4 },
	.m = { .min = 104, .max = 138 },
	.m1 = { .min = 17, .max = 23 },
	.m2 = { .min = 5, .max = 11 },
	.p = { .min = 10, .max = 30 },
	.p1 = { .min = 1, .max = 3},
	.p2 = { .dot_limit = 270000,
		.p2_slow = 10,
		.p2_fast = 10
	},
	.find_pll = intel_g4x_find_best_PLL,
};

static const intel_limit_t intel_limits_g4x_hdmi = {
	.dot = { .min = 22000, .max = 400000 },
	.vco = { .min = 1750000, .max = 3500000},
	.n = { .min = 1, .max = 4 },
	.m = { .min = 104, .max = 138 },
	.m1 = { .min = 16, .max = 23 },
	.m2 = { .min = 5, .max = 11 },
	.p = { .min = 5, .max = 80 },
	.p1 = { .min = 1, .max = 8},
	.p2 = { .dot_limit = 165000,
		.p2_slow = 10, .p2_fast = 5 },
	.find_pll = intel_g4x_find_best_PLL,
};

static const intel_limit_t intel_limits_g4x_single_channel_lvds = {
	.dot = { .min = 20000, .max = 115000 },
	.vco = { .min = 1750000, .max = 3500000 },
	.n = { .min = 1, .max = 3 },
	.m = { .min = 104, .max = 138 },
	.m1 = { .min = 17, .max = 23 },
	.m2 = { .min = 5, .max = 11 },
	.p = { .min = 28, .max = 112 },
	.p1 = { .min = 2, .max = 8 },
	.p2 = { .dot_limit = 0,
		.p2_slow = 14, .p2_fast = 14
	},
	.find_pll = intel_g4x_find_best_PLL,
};

static const intel_limit_t intel_limits_g4x_dual_channel_lvds = {
	.dot = { .min = 80000, .max = 224000 },
	.vco = { .min = 1750000, .max = 3500000 },
	.n = { .min = 1, .max = 3 },
	.m = { .min = 104, .max = 138 },
	.m1 = { .min = 17, .max = 23 },
	.m2 = { .min = 5, .max = 11 },
	.p = { .min = 14, .max = 42 },
	.p1 = { .min = 2, .max = 6 },
	.p2 = { .dot_limit = 0,
		.p2_slow = 7, .p2_fast = 7
	},
	.find_pll = intel_g4x_find_best_PLL,
};

static const intel_limit_t intel_limits_g4x_display_port = {
	.dot = { .min = 161670, .max = 227000 },
	.vco = { .min = 1750000, .max = 3500000},
	.n = { .min = 1, .max = 2 },
	.m = { .min = 97, .max = 108 },
	.m1 = { .min = 0x10, .max = 0x12 },
	.m2 = { .min = 0x05, .max = 0x06 },
	.p = { .min = 10, .max = 20 },
	.p1 = { .min = 1, .max = 2},
	.p2 = { .dot_limit = 0,
		.p2_slow = 10, .p2_fast = 10 },
	.find_pll = intel_find_pll_g4x_dp,
};

static const intel_limit_t intel_limits_pineview_sdvo = {
	.dot = { .min = 20000, .max = 400000},
	.vco = { .min = 1700000, .max = 3500000 },
	/* Pineview's Ncounter is a ring counter */
	.n = { .min = 3, .max = 6 },
	.m = { .min = 2, .max = 256 },
	/* Pineview only has one combined m divider, which we treat as m2. */
	.m1 = { .min = 0, .max = 0 },
	.m2 = { .min = 0, .max = 254 },
	.p = { .min = 5, .max = 80 },
	.p1 = { .min = 1, .max = 8 },
	.p2 = { .dot_limit = 200000,
		.p2_slow = 10, .p2_fast = 5 },
	.find_pll = intel_find_best_PLL,
};

static const intel_limit_t intel_limits_pineview_lvds = {
	.dot = { .min = 20000, .max = 400000 },
	.vco = { .min = 1700000, .max = 3500000 },
	.n = { .min = 3, .max = 6 },
	.m = { .min = 2, .max = 256 },
	.m1 = { .min = 0, .max = 0 },
	.m2 = { .min = 0, .max = 254 },
	.p = { .min = 7, .max = 112 },
	.p1 = { .min = 1, .max = 8 },
	.p2 = { .dot_limit = 112000,
		.p2_slow = 14, .p2_fast = 14 },
	.find_pll = intel_find_best_PLL,
};

/* Ironlake / Sandybridge
 *
 * We calculate clock using (register_value + 2) for N/M1/M2, so here
 * the range value for them is (actual_value - 2).
 */
static const intel_limit_t intel_limits_ironlake_dac = {
	.dot = { .min = 25000, .max = 350000 },
	.vco = { .min = 1760000, .max = 3510000 },
	.n = { .min = 1, .max = 5 },
	.m = { .min = 79, .max = 127 },
	.m1 = { .min = 12, .max = 22 },
	.m2 = { .min = 5, .max = 9 },
	.p = { .min = 5, .max = 80 },
	.p1 = { .min = 1, .max = 8 },
	.p2 = { .dot_limit = 225000,
		.p2_slow = 10, .p2_fast = 5 },
	.find_pll = intel_g4x_find_best_PLL,
};

static const intel_limit_t intel_limits_ironlake_single_lvds = {
	.dot = { .min = 25000, .max = 350000 },
	.vco = { .min = 1760000, .max = 3510000 },
	.n = { .min = 1, .max = 3 },
	.m = { .min = 79, .max = 118 },
	.m1 = { .min = 12, .max = 22 },
	.m2 = { .min = 5, .max = 9 },
	.p = { .min = 28, .max = 112 },
	.p1 = { .min = 2, .max = 8 },
	.p2 = { .dot_limit = 225000,
		.p2_slow = 14, .p2_fast = 14 },
	.find_pll = intel_g4x_find_best_PLL,
};

static const intel_limit_t intel_limits_ironlake_dual_lvds = {
	.dot = { .min = 25000, .max = 350000 },
	.vco = { .min = 1760000, .max = 3510000 },
	.n = { .min = 1, .max = 3 },
	.m = { .min = 79, .max = 127 },
	.m1 = { .min = 12, .max = 22 },
	.m2 = { .min = 5, .max = 9 },
	.p = { .min = 14, .max = 56 },
	.p1 = { .min = 2, .max = 8 },
	.p2 = { .dot_limit = 225000,
		.p2_slow = 7, .p2_fast = 7 },
	.find_pll = intel_g4x_find_best_PLL,
};

/* LVDS 100mhz refclk limits. */
static const intel_limit_t intel_limits_ironlake_single_lvds_100m = {
	.dot = { .min = 25000, .max = 350000 },
	.vco = { .min = 1760000, .max = 3510000 },
	.n = { .min = 1, .max = 2 },
	.m = { .min = 79, .max = 126 },
	.m1 = { .min = 12, .max = 22 },
	.m2 = { .min = 5, .max = 9 },
	.p = { .min = 28, .max = 112 },
	.p1 = { .min = 2, .max = 8 },
	.p2 = { .dot_limit = 225000,
		.p2_slow = 14, .p2_fast = 14 },
	.find_pll = intel_g4x_find_best_PLL,
};

static const intel_limit_t intel_limits_ironlake_dual_lvds_100m = {
	.dot = { .min = 25000, .max = 350000 },
	.vco = { .min = 1760000, .max = 3510000 },
	.n = { .min = 1, .max = 3 },
	.m = { .min = 79, .max = 126 },
	.m1 = { .min = 12, .max = 22 },
	.m2 = { .min = 5, .max = 9 },
	.p = { .min = 14, .max = 42 },
	.p1 = { .min = 2, .max = 6 },
	.p2 = { .dot_limit = 225000,
		.p2_slow = 7, .p2_fast = 7 },
	.find_pll = intel_g4x_find_best_PLL,
};

static const intel_limit_t intel_limits_ironlake_display_port = {
	.dot = { .min = 25000, .max = 350000 },
	.vco = { .min = 1760000, .max = 3510000},
	.n = { .min = 1, .max = 2 },
	.m = { .min = 81, .max = 90 },
	.m1 = { .min = 12, .max = 22 },
	.m2 = { .min = 5, .max = 9 },
	.p = { .min = 10, .max = 20 },
	.p1 = { .min = 1, .max = 2},
	.p2 = { .dot_limit = 0,
		.p2_slow = 10, .p2_fast = 10 },
	.find_pll = intel_find_pll_ironlake_dp,
};

static bool is_dual_link_lvds(struct drm_i915_private *dev_priv,
			      unsigned int reg)
{
	unsigned int val;

	if (dev_priv->lvds_val)
		val = dev_priv->lvds_val;
	else {
		/* BIOS should set the proper LVDS register value at boot, but
		 * in reality, it doesn't set the value when the lid is closed;
		 * we need to check "the value to be set" in VBT when LVDS
		 * register is uninitialized.
		 */
		val = I915_READ(reg);
		if (!(val & ~LVDS_DETECTED))
			val = dev_priv->bios_lvds_val;
		dev_priv->lvds_val = val;
	}
	return (val & LVDS_CLKB_POWER_MASK) == LVDS_CLKB_POWER_UP;
}

static const intel_limit_t intel_limits_vlv_dac = {
	.dot = { .min = 25000, .max = 270000 },
	.vco = { .min = 4000000, .max = 6000000 },
	.n = { .min = 1, .max = 7 },
	.m = { .min = 22, .max = 450 }, /* guess */
	.m1 = { .min = 2, .max = 3 },
	.m2 = { .min = 11, .max = 156 },
	.p = { .min = 10, .max = 30 },
	.p1 = { .min = 2, .max = 3 },
	.p2 = { .dot_limit = 270000,
		.p2_slow = 2, .p2_fast = 20 },
	.find_pll = intel_vlv_find_best_pll,
};

static const intel_limit_t intel_limits_vlv_hdmi = {
	.dot = { .min = 20000, .max = 165000 },
	.vco = { .min = 4000000, .max = 5994000},
	.n = { .min = 1, .max = 7 },
	.m = { .min = 60, .max = 300 }, /* guess */
	.m1 = { .min = 2, .max = 3 },
	.m2 = { .min = 11, .max = 156 },
	.p = { .min = 10, .max = 30 },
	.p1 = { .min = 2, .max = 3 },
	.p2 = { .dot_limit = 270000,
		.p2_slow = 2, .p2_fast = 20 },
	.find_pll = intel_vlv_find_best_pll,
};

static const intel_limit_t intel_limits_vlv_dp = {
	.dot = { .min = 25000, .max = 270000 },
	.vco = { .min = 4000000, .max = 6000000 },
	.n = { .min = 1, .max = 7 },
	.m = { .min = 22, .max = 450 },
	.m1 = { .min = 2, .max = 3 },
	.m2 = { .min = 11, .max = 156 },
	.p = { .min = 10, .max = 30 },
	.p1 = { .min = 2, .max = 3 },
	.p2 = { .dot_limit = 270000,
		.p2_slow = 2, .p2_fast = 20 },
	.find_pll = intel_vlv_find_best_pll,
};

u32 intel_dpio_read(struct drm_i915_private *dev_priv, int reg)
{
	unsigned long flags;
	u32 val = 0;

	spin_lock_irqsave(&dev_priv->dpio_lock, flags);
	if (wait_for_atomic_us((I915_READ(DPIO_PKT) & DPIO_BUSY) == 0, 100)) {
		DRM_ERROR("DPIO idle wait timed out\n");
		goto out_unlock;
	}

	I915_WRITE(DPIO_REG, reg);
	I915_WRITE(DPIO_PKT, DPIO_RID | DPIO_OP_READ | DPIO_PORTID |
		   DPIO_BYTE);
	if (wait_for_atomic_us((I915_READ(DPIO_PKT) & DPIO_BUSY) == 0, 100)) {
		DRM_ERROR("DPIO read wait timed out\n");
		goto out_unlock;
	}
	val = I915_READ(DPIO_DATA);

out_unlock:
	spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
	return val;
}

static void intel_dpio_write(struct drm_i915_private *dev_priv, int reg,
			     u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->dpio_lock, flags);
	if (wait_for_atomic_us((I915_READ(DPIO_PKT) & DPIO_BUSY) == 0, 100)) {
		DRM_ERROR("DPIO idle wait timed out\n");
		goto out_unlock;
	}

	I915_WRITE(DPIO_DATA, val);
	I915_WRITE(DPIO_REG, reg);
	I915_WRITE(DPIO_PKT, DPIO_RID | DPIO_OP_WRITE | DPIO_PORTID |
		   DPIO_BYTE);
	if (wait_for_atomic_us((I915_READ(DPIO_PKT) & DPIO_BUSY) == 0, 100))
		DRM_ERROR("DPIO write wait timed out\n");

out_unlock:
       spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
}

static void vlv_init_dpio(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* Reset the DPIO config */
	I915_WRITE(DPIO_CTL, 0);
	POSTING_READ(DPIO_CTL);
	I915_WRITE(DPIO_CTL, 1);
	POSTING_READ(DPIO_CTL);
}

static int intel_dual_link_lvds_callback(const struct dmi_system_id *id)
{
	DRM_INFO("Forcing lvds to dual link mode on %s\n", id->ident);
	return 1;
}

static const struct dmi_system_id intel_dual_link_lvds[] = {
	{
		.callback = intel_dual_link_lvds_callback,
		.ident = "Apple MacBook Pro (Core i5/i7 Series)",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Apple Inc."),
			DMI_MATCH(DMI_PRODUCT_NAME, "MacBookPro8,2"),
		},
	},
	{ }	/* terminating entry */
};

static bool is_dual_link_lvds(struct drm_i915_private *dev_priv,
			      unsigned int reg)
{
	unsigned int val;

	/* use the module option value if specified */
	if (i915_lvds_channel_mode > 0)
		return i915_lvds_channel_mode == 2;

	if (dmi_check_system(intel_dual_link_lvds))
		return true;

	if (dev_priv->lvds_val)
		val = dev_priv->lvds_val;
	else {
		/* BIOS should set the proper LVDS register value at boot, but
		 * in reality, it doesn't set the value when the lid is closed;
		 * we need to check "the value to be set" in VBT when LVDS
		 * register is uninitialized.
		 */
		val = I915_READ(reg);
		if (!(val & ~(LVDS_PIPE_MASK | LVDS_DETECTED)))
			val = dev_priv->bios_lvds_val;
		dev_priv->lvds_val = val;
	}
	return (val & LVDS_CLKB_POWER_MASK) == LVDS_CLKB_POWER_UP;
}

static const intel_limit_t *intel_ironlake_limit(struct drm_crtc *crtc,
						int refclk)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	const intel_limit_t *limit;

	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS)) {
		if (is_dual_link_lvds(dev_priv, PCH_LVDS)) {
			/* LVDS dual channel */
			if (refclk == 100000)
				limit = &intel_limits_ironlake_dual_lvds_100m;
			else
				limit = &intel_limits_ironlake_dual_lvds;
		} else {
			if (refclk == 100000)
				limit = &intel_limits_ironlake_single_lvds_100m;
			else
				limit = &intel_limits_ironlake_single_lvds;
		}
	} else if (intel_pipe_has_type(crtc, INTEL_OUTPUT_DISPLAYPORT) ||
		   intel_pipe_has_type(crtc, INTEL_OUTPUT_EDP))
		limit = &intel_limits_ironlake_display_port;
	else
		limit = &intel_limits_ironlake_dac;

	return limit;
}

static const intel_limit_t *intel_g4x_limit(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	const intel_limit_t *limit;

	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS)) {
		if (is_dual_link_lvds(dev_priv, LVDS))
			/* LVDS with dual channel */
			limit = &intel_limits_g4x_dual_channel_lvds;
		else
			/* LVDS with dual channel */
			limit = &intel_limits_g4x_single_channel_lvds;
	} else if (intel_pipe_has_type(crtc, INTEL_OUTPUT_HDMI) ||
		   intel_pipe_has_type(crtc, INTEL_OUTPUT_ANALOG)) {
		limit = &intel_limits_g4x_hdmi;
	} else if (intel_pipe_has_type(crtc, INTEL_OUTPUT_SDVO)) {
		limit = &intel_limits_g4x_sdvo;
	} else if (intel_pipe_has_type(crtc, INTEL_OUTPUT_DISPLAYPORT)) {
		limit = &intel_limits_g4x_display_port;
	} else /* The option is for other outputs */
		limit = &intel_limits_i9xx_sdvo;

	return limit;
}

static const intel_limit_t *intel_limit(struct drm_crtc *crtc, int refclk)
{
	struct drm_device *dev = crtc->dev;
	const intel_limit_t *limit;

	if (HAS_PCH_SPLIT(dev))
		limit = intel_ironlake_limit(crtc, refclk);
	else if (IS_G4X(dev)) {
		limit = intel_g4x_limit(crtc);
	} else if (IS_PINEVIEW(dev)) {
		if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS))
			limit = &intel_limits_pineview_lvds;
		else
			limit = &intel_limits_pineview_sdvo;
	} else if (IS_VALLEYVIEW(dev)) {
		if (intel_pipe_has_type(crtc, INTEL_OUTPUT_ANALOG))
			limit = &intel_limits_vlv_dac;
		else if (intel_pipe_has_type(crtc, INTEL_OUTPUT_HDMI))
			limit = &intel_limits_vlv_hdmi;
		else
			limit = &intel_limits_vlv_dp;
	} else if (!IS_GEN2(dev)) {
		if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS))
			limit = &intel_limits_i9xx_lvds;
		else
			limit = &intel_limits_i9xx_sdvo;
	} else {
		if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS))
			limit = &intel_limits_i8xx_lvds;
		else
			limit = &intel_limits_i8xx_dvo;
	}
	return limit;
}

/* m1 is reserved as 0 in Pineview, n is a ring counter */
static void pineview_clock(int refclk, intel_clock_t *clock)
{
	clock->m = clock->m2 + 2;
	clock->p = clock->p1 * clock->p2;
	clock->vco = refclk * clock->m / clock->n;
	clock->dot = clock->vco / clock->p;
}

static void intel_clock(struct drm_device *dev, int refclk, intel_clock_t *clock)
{
	if (IS_PINEVIEW(dev)) {
		pineview_clock(refclk, clock);
		return;
	}
	clock->m = 5 * (clock->m1 + 2) + (clock->m2 + 2);
	clock->p = clock->p1 * clock->p2;
	clock->vco = refclk * clock->m / (clock->n + 2);
	clock->dot = clock->vco / clock->p;
}

/**
 * Returns whether any output on the specified pipe is of the specified type
 */
bool intel_pipe_has_type(struct drm_crtc *crtc, int type)
{
	struct drm_device *dev = crtc->dev;
	struct intel_encoder *encoder;

	for_each_encoder_on_crtc(dev, crtc, encoder)
		if (encoder->type == type)
			return true;

	return false;
}

#define INTELPllInvalid(s)   do { /* DRM_DEBUG(s); */ return false; } while (0)
/**
 * Returns whether the given set of divisors are valid for a given refclk with
 * the given connectors.
 */

static bool intel_PLL_is_valid(struct drm_device *dev,
			       const intel_limit_t *limit,
			       const intel_clock_t *clock)
{
	if (clock->p1  < limit->p1.min  || limit->p1.max  < clock->p1)
		INTELPllInvalid("p1 out of range\n");
	if (clock->p   < limit->p.min   || limit->p.max   < clock->p)
		INTELPllInvalid("p out of range\n");
	if (clock->m2  < limit->m2.min  || limit->m2.max  < clock->m2)
		INTELPllInvalid("m2 out of range\n");
	if (clock->m1  < limit->m1.min  || limit->m1.max  < clock->m1)
		INTELPllInvalid("m1 out of range\n");
	if (clock->m1 <= clock->m2 && !IS_PINEVIEW(dev))
		INTELPllInvalid("m1 <= m2\n");
	if (clock->m   < limit->m.min   || limit->m.max   < clock->m)
		INTELPllInvalid("m out of range\n");
	if (clock->n   < limit->n.min   || limit->n.max   < clock->n)
		INTELPllInvalid("n out of range\n");
	if (clock->vco < limit->vco.min || limit->vco.max < clock->vco)
		INTELPllInvalid("vco out of range\n");
	/* XXX: We may need to be checking "Dot clock" depending on the multiplier,
	 * connector, etc., rather than just a single range.
	 */
	if (clock->dot < limit->dot.min || limit->dot.max < clock->dot)
		INTELPllInvalid("dot out of range\n");

	return true;
}

static bool
intel_find_best_PLL(const intel_limit_t *limit, struct drm_crtc *crtc,
		    int target, int refclk, intel_clock_t *match_clock,
		    intel_clock_t *best_clock)

{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	intel_clock_t clock;
	int err = target;

	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS) &&
	    (I915_READ(LVDS)) != 0) {
		/*
		 * For LVDS, if the panel is on, just rely on its current
		 * settings for dual-channel.  We haven't figured out how to
		 * reliably set up different single/dual channel state, if we
		 * even can.
		 */
		if (is_dual_link_lvds(dev_priv, LVDS))
			clock.p2 = limit->p2.p2_fast;
		else
			clock.p2 = limit->p2.p2_slow;
	} else {
		if (target < limit->p2.dot_limit)
			clock.p2 = limit->p2.p2_slow;
		else
			clock.p2 = limit->p2.p2_fast;
	}

	memset(best_clock, 0, sizeof(*best_clock));

	for (clock.m1 = limit->m1.min; clock.m1 <= limit->m1.max;
	     clock.m1++) {
		for (clock.m2 = limit->m2.min;
		     clock.m2 <= limit->m2.max; clock.m2++) {
			/* m1 is always 0 in Pineview */
			if (clock.m2 >= clock.m1 && !IS_PINEVIEW(dev))
				break;
			for (clock.n = limit->n.min;
			     clock.n <= limit->n.max; clock.n++) {
				for (clock.p1 = limit->p1.min;
					clock.p1 <= limit->p1.max; clock.p1++) {
					int this_err;

					intel_clock(dev, refclk, &clock);
					if (!intel_PLL_is_valid(dev, limit,
								&clock))
						continue;
					if (match_clock &&
					    clock.p != match_clock->p)
						continue;

					this_err = abs(clock.dot - target);
					if (this_err < err) {
						*best_clock = clock;
						err = this_err;
					}
				}
			}
		}
	}

	return (err != target);
}

static bool
intel_g4x_find_best_PLL(const intel_limit_t *limit, struct drm_crtc *crtc,
			int target, int refclk, intel_clock_t *match_clock,
			intel_clock_t *best_clock)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	intel_clock_t clock;
	int max_n;
	bool found;
	/* approximately equals target * 0.00585 */
	int err_most = (target >> 8) + (target >> 9);
	found = false;

	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS)) {
		int lvds_reg;

		if (HAS_PCH_SPLIT(dev))
			lvds_reg = PCH_LVDS;
		else
			lvds_reg = LVDS;
		if ((I915_READ(lvds_reg) & LVDS_CLKB_POWER_MASK) ==
		    LVDS_CLKB_POWER_UP)
			clock.p2 = limit->p2.p2_fast;
		else
			clock.p2 = limit->p2.p2_slow;
	} else {
		if (target < limit->p2.dot_limit)
			clock.p2 = limit->p2.p2_slow;
		else
			clock.p2 = limit->p2.p2_fast;
	}

	memset(best_clock, 0, sizeof(*best_clock));
	max_n = limit->n.max;
	/* based on hardware requirement, prefer smaller n to precision */
	for (clock.n = limit->n.min; clock.n <= max_n; clock.n++) {
		/* based on hardware requirement, prefere larger m1,m2 */
		for (clock.m1 = limit->m1.max;
		     clock.m1 >= limit->m1.min; clock.m1--) {
			for (clock.m2 = limit->m2.max;
			     clock.m2 >= limit->m2.min; clock.m2--) {
				for (clock.p1 = limit->p1.max;
				     clock.p1 >= limit->p1.min; clock.p1--) {
					int this_err;

					intel_clock(dev, refclk, &clock);
					if (!intel_PLL_is_valid(dev, limit,
								&clock))
						continue;
					if (match_clock &&
					    clock.p != match_clock->p)
						continue;

					this_err = abs(clock.dot - target);
					if (this_err < err_most) {
						*best_clock = clock;
						err_most = this_err;
						max_n = clock.n;
						found = true;
					}
				}
			}
		}
	}
	return found;
}

static bool
intel_find_pll_ironlake_dp(const intel_limit_t *limit, struct drm_crtc *crtc,
			   int target, int refclk, intel_clock_t *match_clock,
			   intel_clock_t *best_clock)
{
	struct drm_device *dev = crtc->dev;
	intel_clock_t clock;

	if (target < 200000) {
		clock.n = 1;
		clock.p1 = 2;
		clock.p2 = 10;
		clock.m1 = 12;
		clock.m2 = 9;
	} else {
		clock.n = 2;
		clock.p1 = 1;
		clock.p2 = 10;
		clock.m1 = 14;
		clock.m2 = 8;
	}
	intel_clock(dev, refclk, &clock);
	memcpy(best_clock, &clock, sizeof(intel_clock_t));
	return true;
}

/* DisplayPort has only two frequencies, 162MHz and 270MHz */
static bool
intel_find_pll_g4x_dp(const intel_limit_t *limit, struct drm_crtc *crtc,
		      int target, int refclk, intel_clock_t *match_clock,
		      intel_clock_t *best_clock)
{
	intel_clock_t clock;
	if (target < 200000) {
		clock.p1 = 2;
		clock.p2 = 10;
		clock.n = 2;
		clock.m1 = 23;
		clock.m2 = 8;
	} else {
		clock.p1 = 1;
		clock.p2 = 10;
		clock.n = 1;
		clock.m1 = 14;
		clock.m2 = 2;
	}
	clock.m = 5 * (clock.m1 + 2) + (clock.m2 + 2);
	clock.p = (clock.p1 * clock.p2);
	clock.dot = 96000 * clock.m / (clock.n + 2) / clock.p;
	clock.vco = 0;
	memcpy(best_clock, &clock, sizeof(intel_clock_t));
	return true;
}
static bool
intel_vlv_find_best_pll(const intel_limit_t *limit, struct drm_crtc *crtc,
			int target, int refclk, intel_clock_t *match_clock,
			intel_clock_t *best_clock)
{
	u32 p1, p2, m1, m2, vco, bestn, bestm1, bestm2, bestp1, bestp2;
	u32 m, n, fastclk;
	u32 updrate, minupdate, fracbits, p;
	unsigned long bestppm, ppm, absppm;
	int dotclk, flag;

	flag = 0;
	dotclk = target * 1000;
	bestppm = 1000000;
	ppm = absppm = 0;
	fastclk = dotclk / (2*100);
	updrate = 0;
	minupdate = 19200;
	fracbits = 1;
	n = p = p1 = p2 = m = m1 = m2 = vco = bestn = 0;
	bestm1 = bestm2 = bestp1 = bestp2 = 0;

	/* based on hardware requirement, prefer smaller n to precision */
	for (n = limit->n.min; n <= ((refclk) / minupdate); n++) {
		updrate = refclk / n;
		for (p1 = limit->p1.max; p1 > limit->p1.min; p1--) {
			for (p2 = limit->p2.p2_fast+1; p2 > 0; p2--) {
				if (p2 > 10)
					p2 = p2 - 1;
				p = p1 * p2;
				/* based on hardware requirement, prefer bigger m1,m2 values */
				for (m1 = limit->m1.min; m1 <= limit->m1.max; m1++) {
					m2 = (((2*(fastclk * p * n / m1 )) +
					       refclk) / (2*refclk));
					m = m1 * m2;
					vco = updrate * m;
					if (vco >= limit->vco.min && vco < limit->vco.max) {
						ppm = 1000000 * ((vco / p) - fastclk) / fastclk;
						absppm = (ppm > 0) ? ppm : (-ppm);
						if (absppm < 100 && ((p1 * p2) > (bestp1 * bestp2))) {
							bestppm = 0;
							flag = 1;
						}
						if (absppm < bestppm - 10) {
							bestppm = absppm;
							flag = 1;
						}
						if (flag) {
							bestn = n;
							bestm1 = m1;
							bestm2 = m2;
							bestp1 = p1;
							bestp2 = p2;
							flag = 0;
						}
					}
				}
			}
		}
	}
	best_clock->n = bestn;
	best_clock->m1 = bestm1;
	best_clock->m2 = bestm2;
	best_clock->p1 = bestp1;
	best_clock->p2 = bestp2;

	return true;
}

enum transcoder intel_pipe_to_cpu_transcoder(struct drm_i915_private *dev_priv,
					     enum pipe pipe)
{
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	return intel_crtc->config.cpu_transcoder;
}

static void ironlake_wait_for_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 frame, frame_reg = PIPEFRAME(pipe);

	frame = I915_READ(frame_reg);

	if (wait_for(I915_READ_NOTRACE(frame_reg) != frame, 50))
		DRM_DEBUG_KMS("vblank wait timed out\n");
}

/**
 * intel_wait_for_vblank - wait for vblank on a given pipe
 * @dev: drm device
 * @pipe: pipe to wait for
 *
 * Wait for vblank to occur on a given pipe.  Needed for various bits of
 * mode setting code.
 */
void intel_wait_for_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipestat_reg = PIPESTAT(pipe);

	if (INTEL_INFO(dev)->gen >= 5) {
		ironlake_wait_for_vblank(dev, pipe);
		return;
	}

	/* Clear existing vblank status. Note this will clear any other
	 * sticky status fields as well.
	 *
	 * This races with i915_driver_irq_handler() with the result
	 * that either function could miss a vblank event.  Here it is not
	 * fatal, as we will either wait upon the next vblank interrupt or
	 * timeout.  Generally speaking intel_wait_for_vblank() is only
	 * called during modeset at which time the GPU should be idle and
	 * should *not* be performing page flips and thus not waiting on
	 * vblanks...
	 * Currently, the result of us stealing a vblank from the irq
	 * handler is that a single frame will be skipped during swapbuffers.
	 */
	I915_WRITE(pipestat_reg,
		   I915_READ(pipestat_reg) | PIPE_VBLANK_INTERRUPT_STATUS);

	/* Wait for vblank interrupt bit to set */
	if (wait_for(I915_READ(pipestat_reg) &
		     PIPE_VBLANK_INTERRUPT_STATUS,
		     50))
		DRM_DEBUG_KMS("vblank wait timed out\n");
}

/*
 * intel_wait_for_pipe_off - wait for pipe to turn off
 * @dev: drm device
 * @pipe: pipe to wait for
 *
 * After disabling a pipe, we can't wait for vblank in the usual way,
 * spinning on the vblank interrupt status bit, since we won't actually
 * see an interrupt when the pipe is disabled.
 *
 * On Gen4 and above:
 *   wait for the pipe register state bit to turn off
 *
 * Otherwise:
 *   wait for the display line value to settle (it usually
 *   ends up stopping at the start of the next frame).
 *
 */
void intel_wait_for_pipe_off(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum transcoder cpu_transcoder = intel_pipe_to_cpu_transcoder(dev_priv,
								      pipe);

	if (INTEL_INFO(dev)->gen >= 4) {
		int reg = PIPECONF(cpu_transcoder);

		/* Wait for the Pipe State to go off */
		if (wait_for((I915_READ(reg) & I965_PIPECONF_ACTIVE) == 0,
			     100))
			WARN(1, "pipe_off wait timed out\n");
	} else {
		u32 last_line, line_mask;
		int reg = PIPEDSL(pipe);
		unsigned long timeout = jiffies + msecs_to_jiffies(100);

		if (IS_GEN2(dev))
			line_mask = DSL_LINEMASK_GEN2;
		else
			line_mask = DSL_LINEMASK_GEN3;

		/* Wait for the display line to settle */
		do {
			last_line = I915_READ(reg) & line_mask;
			mdelay(5);
		} while (((I915_READ(reg) & line_mask) != last_line) &&
			 time_after(timeout, jiffies));
		if (time_after(jiffies, timeout))
			WARN(1, "pipe_off wait timed out\n");
	}
}

static const char *state_string(bool enabled)
{
	return enabled ? "on" : "off";
}

/* Only for pre-ILK configs */
static void assert_pll(struct drm_i915_private *dev_priv,
		       enum pipe pipe, bool state)
{
	int reg;
	u32 val;
	bool cur_state;

	reg = DPLL(pipe);
	val = I915_READ(reg);
	cur_state = !!(val & DPLL_VCO_ENABLE);
	WARN(cur_state != state,
	     "PLL state assertion failure (expected %s, current %s)\n",
	     state_string(state), state_string(cur_state));
}
#define assert_pll_enabled(d, p) assert_pll(d, p, true)
#define assert_pll_disabled(d, p) assert_pll(d, p, false)

/* For ILK+ */
static void assert_pch_pll(struct drm_i915_private *dev_priv,
			   struct intel_pch_pll *pll,
			   struct intel_crtc *crtc,
			   bool state)
{
	u32 val;
	bool cur_state;

	if (HAS_PCH_LPT(dev_priv->dev)) {
		DRM_DEBUG_DRIVER("LPT detected: skipping PCH PLL test\n");
		return;
	}

	if (WARN (!pll,
		  "asserting PCH PLL %s with no PLL\n", state_string(state)))
		return;

	val = I915_READ(pll->pll_reg);
	cur_state = !!(val & DPLL_VCO_ENABLE);
	WARN(cur_state != state,
	     "PCH PLL state for reg %x assertion failure (expected %s, current %s), val=%08x\n",
	     pll->pll_reg, state_string(state), state_string(cur_state), val);

	/* Make sure the selected PLL is correctly attached to the transcoder */
	if (crtc && HAS_PCH_CPT(dev_priv->dev)) {
		u32 pch_dpll;

		pch_dpll = I915_READ(PCH_DPLL_SEL);
		cur_state = pll->pll_reg == _PCH_DPLL_B;
		if (!WARN(((pch_dpll >> (4 * crtc->pipe)) & 1) != cur_state,
			  "PLL[%d] not attached to this transcoder %d: %08x\n",
			  cur_state, crtc->pipe, pch_dpll)) {
			cur_state = !!(val >> (4*crtc->pipe + 3));
			WARN(cur_state != state,
			     "PLL[%d] not %s on this transcoder %d: %08x\n",
			     pll->pll_reg == _PCH_DPLL_B,
			     state_string(state),
			     crtc->pipe,
			     val);
		}
	}
}
#define assert_pch_pll_enabled(d, p, c) assert_pch_pll(d, p, c, true)
#define assert_pch_pll_disabled(d, p, c) assert_pch_pll(d, p, c, false)

static void assert_fdi_tx(struct drm_i915_private *dev_priv,
			  enum pipe pipe, bool state)
{
	int reg;
	u32 val;
	bool cur_state;
	enum transcoder cpu_transcoder = intel_pipe_to_cpu_transcoder(dev_priv,
								      pipe);

	if (IS_HASWELL(dev_priv->dev)) {
		/* On Haswell, DDI is used instead of FDI_TX_CTL */
		reg = TRANS_DDI_FUNC_CTL(cpu_transcoder);
		val = I915_READ(reg);
		cur_state = !!(val & TRANS_DDI_FUNC_ENABLE);
	} else {
		reg = FDI_TX_CTL(pipe);
		val = I915_READ(reg);
		cur_state = !!(val & FDI_TX_ENABLE);
	}
	WARN(cur_state != state,
	     "FDI TX state assertion failure (expected %s, current %s)\n",
	     state_string(state), state_string(cur_state));
}
#define assert_fdi_tx_enabled(d, p) assert_fdi_tx(d, p, true)
#define assert_fdi_tx_disabled(d, p) assert_fdi_tx(d, p, false)

static void assert_fdi_rx(struct drm_i915_private *dev_priv,
			  enum pipe pipe, bool state)
{
	int reg;
	u32 val;
	bool cur_state;

	reg = FDI_RX_CTL(pipe);
	val = I915_READ(reg);
	cur_state = !!(val & FDI_RX_ENABLE);
	WARN(cur_state != state,
	     "FDI RX state assertion failure (expected %s, current %s)\n",
	     state_string(state), state_string(cur_state));
}
#define assert_fdi_rx_enabled(d, p) assert_fdi_rx(d, p, true)
#define assert_fdi_rx_disabled(d, p) assert_fdi_rx(d, p, false)

static void assert_fdi_tx_pll_enabled(struct drm_i915_private *dev_priv,
				      enum pipe pipe)
{
	int reg;
	u32 val;

	/* ILK FDI PLL is always enabled */
	if (dev_priv->info->gen == 5)
		return;

	/* On Haswell, DDI ports are responsible for the FDI PLL setup */
	if (IS_HASWELL(dev_priv->dev))
		return;

	reg = FDI_TX_CTL(pipe);
	val = I915_READ(reg);
	WARN(!(val & FDI_TX_PLL_ENABLE), "FDI TX PLL assertion failure, should be active but is disabled\n");
}

static void assert_fdi_rx_pll_enabled(struct drm_i915_private *dev_priv,
				      enum pipe pipe)
{
	int reg;
	u32 val;

	reg = FDI_RX_CTL(pipe);
	val = I915_READ(reg);
	WARN(!(val & FDI_RX_PLL_ENABLE), "FDI RX PLL assertion failure, should be active but is disabled\n");
}

static void assert_panel_unlocked(struct drm_i915_private *dev_priv,
				  enum pipe pipe)
{
	int pp_reg, lvds_reg;
	u32 val;
	enum pipe panel_pipe = PIPE_A;
	bool locked = true;

	if (HAS_PCH_SPLIT(dev_priv->dev)) {
		pp_reg = PCH_PP_CONTROL;
		lvds_reg = PCH_LVDS;
	} else {
		pp_reg = PP_CONTROL;
		lvds_reg = LVDS;
	}

	val = I915_READ(pp_reg);
	if (!(val & PANEL_POWER_ON) ||
	    ((val & PANEL_UNLOCK_REGS) == PANEL_UNLOCK_REGS))
		locked = false;

	if (I915_READ(lvds_reg) & LVDS_PIPEB_SELECT)
		panel_pipe = PIPE_B;

	WARN(panel_pipe == pipe && locked,
	     "panel assertion failure, pipe %c regs locked\n",
	     pipe_name(pipe));
}

void assert_pipe(struct drm_i915_private *dev_priv,
		 enum pipe pipe, bool state)
{
	int reg;
	u32 val;
	bool cur_state;
	enum transcoder cpu_transcoder = intel_pipe_to_cpu_transcoder(dev_priv,
								      pipe);

	/* if we need the pipe A quirk it must be always on */
	if (pipe == PIPE_A && dev_priv->quirks & QUIRK_PIPEA_FORCE)
		state = true;

	reg = PIPECONF(cpu_transcoder);
	val = I915_READ(reg);
	cur_state = !!(val & PIPECONF_ENABLE);
	WARN(cur_state != state,
	     "pipe %c assertion failure (expected %s, current %s)\n",
	     pipe_name(pipe), state_string(state), state_string(cur_state));
}

static void assert_plane(struct drm_i915_private *dev_priv,
			 enum plane plane, bool state)
{
	int reg;
	u32 val;
	bool cur_state;

	reg = DSPCNTR(plane);
	val = I915_READ(reg);
	cur_state = !!(val & DISPLAY_PLANE_ENABLE);
	WARN(cur_state != state,
	     "plane %c assertion failure (expected %s, current %s)\n",
	     plane_name(plane), state_string(state), state_string(cur_state));
}

#define assert_plane_enabled(d, p) assert_plane(d, p, true)
#define assert_plane_disabled(d, p) assert_plane(d, p, false)

static void assert_planes_disabled(struct drm_i915_private *dev_priv,
				   enum pipe pipe)
{
	int reg, i;
	u32 val;
	int cur_pipe;

	/* Planes are fixed to pipes on ILK+ */
	if (HAS_PCH_SPLIT(dev_priv->dev) || IS_VALLEYVIEW(dev_priv->dev)) {
		reg = DSPCNTR(pipe);
		val = I915_READ(reg);
		WARN((val & DISPLAY_PLANE_ENABLE),
		     "plane %c assertion failure, should be disabled but not\n",
		     plane_name(pipe));
		return;
	}

	/* Need to check both planes against the pipe */
	for (i = 0; i < 2; i++) {
		reg = DSPCNTR(i);
		val = I915_READ(reg);
		cur_pipe = (val & DISPPLANE_SEL_PIPE_MASK) >>
			DISPPLANE_SEL_PIPE_SHIFT;
		WARN((val & DISPLAY_PLANE_ENABLE) && pipe == cur_pipe,
		     "plane %c assertion failure, should be off on pipe %c but is still active\n",
		     plane_name(i), pipe_name(pipe));
	}
}

static void assert_sprites_disabled(struct drm_i915_private *dev_priv,
				    enum pipe pipe)
{
	int reg, i;
	u32 val;

	if (!IS_VALLEYVIEW(dev_priv->dev))
		return;

	/* Need to check both planes against the pipe */
	for (i = 0; i < dev_priv->num_plane; i++) {
		reg = SPCNTR(pipe, i);
		val = I915_READ(reg);
		WARN((val & SP_ENABLE),
		     "sprite %d assertion failure, should be off on pipe %c but is still active\n",
		     pipe * 2 + i, pipe_name(pipe));
	}
}

static void assert_pch_refclk_enabled(struct drm_i915_private *dev_priv)
{
	u32 val;
	bool enabled;

	if (HAS_PCH_LPT(dev_priv->dev)) {
		DRM_DEBUG_DRIVER("LPT does not has PCH refclk, skipping check\n");
		return;
	}

	val = I915_READ(PCH_DREF_CONTROL);
	enabled = !!(val & (DREF_SSC_SOURCE_MASK | DREF_NONSPREAD_SOURCE_MASK |
			    DREF_SUPERSPREAD_SOURCE_MASK));
	WARN(!enabled, "PCH refclk assertion failure, should be active but is disabled\n");
}

static void assert_transcoder_disabled(struct drm_i915_private *dev_priv,
				       enum pipe pipe)
{
	int reg;
	u32 val;
	bool enabled;

	reg = TRANSCONF(pipe);
	val = I915_READ(reg);
	enabled = !!(val & TRANS_ENABLE);
	WARN(enabled,
	     "transcoder assertion failed, should be off on pipe %c but is still active\n",
	     pipe_name(pipe));
}

static bool dp_pipe_enabled(struct drm_i915_private *dev_priv,
			    enum pipe pipe, u32 port_sel, u32 val)
{
	if ((val & DP_PORT_EN) == 0)
		return false;

	if (HAS_PCH_CPT(dev_priv->dev)) {
		u32	trans_dp_ctl_reg = TRANS_DP_CTL(pipe);
		u32	trans_dp_ctl = I915_READ(trans_dp_ctl_reg);
		if ((trans_dp_ctl & TRANS_DP_PORT_SEL_MASK) != port_sel)
			return false;
	} else {
		if ((val & DP_PIPE_MASK) != (pipe << 30))
			return false;
	}
	return true;
}

static bool hdmi_pipe_enabled(struct drm_i915_private *dev_priv,
			      enum pipe pipe, u32 val)
{
	if ((val & SDVO_ENABLE) == 0)
		return false;

	if (HAS_PCH_CPT(dev_priv->dev)) {
		if ((val & SDVO_PIPE_SEL_MASK_CPT) != SDVO_PIPE_SEL_CPT(pipe))
			return false;
	} else {
		if ((val & SDVO_PIPE_SEL_MASK) != SDVO_PIPE_SEL(pipe))
			return false;
	}
	return true;
}

static bool lvds_pipe_enabled(struct drm_i915_private *dev_priv,
			      enum pipe pipe, u32 val)
{
	if ((val & LVDS_PORT_EN) == 0)
		return false;

	if (HAS_PCH_CPT(dev_priv->dev)) {
		if ((val & PORT_TRANS_SEL_MASK) != PORT_TRANS_SEL_CPT(pipe))
			return false;
	} else {
		if ((val & LVDS_PIPE_MASK) != LVDS_PIPE(pipe))
			return false;
	}
	return true;
}

static bool adpa_pipe_enabled(struct drm_i915_private *dev_priv,
			      enum pipe pipe, u32 val)
{
	if ((val & ADPA_DAC_ENABLE) == 0)
		return false;
	if (HAS_PCH_CPT(dev_priv->dev)) {
		if ((val & PORT_TRANS_SEL_MASK) != PORT_TRANS_SEL_CPT(pipe))
			return false;
	} else {
		if ((val & ADPA_PIPE_SELECT_MASK) != ADPA_PIPE_SELECT(pipe))
			return false;
	}
	return true;
}

static void assert_pch_dp_disabled(struct drm_i915_private *dev_priv,
				   enum pipe pipe, int reg, u32 port_sel)
{
	u32 val = I915_READ(reg);
	WARN(dp_pipe_enabled(dev_priv, pipe, port_sel, val),
	     "PCH DP (0x%08x) enabled on transcoder %c, should be disabled\n",
	     reg, pipe_name(pipe));
}

static void assert_pch_hdmi_disabled(struct drm_i915_private *dev_priv,
				     enum pipe pipe, int reg)
{
	u32 val = I915_READ(reg);
	WARN(hdmi_pipe_enabled(dev_priv, pipe, val),
	     "PCH HDMI (0x%08x) enabled on transcoder %c, should be disabled\n",
	     reg, pipe_name(pipe));
}

static void assert_pch_ports_disabled(struct drm_i915_private *dev_priv,
				      enum pipe pipe)
{
	int reg;
	u32 val;

	assert_pch_dp_disabled(dev_priv, pipe, PCH_DP_B, TRANS_DP_PORT_SEL_B);
	assert_pch_dp_disabled(dev_priv, pipe, PCH_DP_C, TRANS_DP_PORT_SEL_C);
	assert_pch_dp_disabled(dev_priv, pipe, PCH_DP_D, TRANS_DP_PORT_SEL_D);

	reg = PCH_ADPA;
	val = I915_READ(reg);
	WARN(adpa_pipe_enabled(dev_priv, pipe, val),
	     "PCH VGA enabled on transcoder %c, should be disabled\n",
	     pipe_name(pipe));

	reg = PCH_LVDS;
	val = I915_READ(reg);
	WARN(lvds_pipe_enabled(dev_priv, pipe, val),
	     "PCH LVDS enabled on transcoder %c, should be disabled\n",
	     pipe_name(pipe));

	assert_pch_hdmi_disabled(dev_priv, pipe, PCH_HDMIB);
	assert_pch_hdmi_disabled(dev_priv, pipe, PCH_HDMIC);
	assert_pch_hdmi_disabled(dev_priv, pipe, PCH_HDMID);
}

/**
 * intel_enable_pll - enable a PLL
 * @dev_priv: i915 private structure
 * @pipe: pipe PLL to enable
 *
 * Enable @pipe's PLL so we can start pumping pixels from a plane.  Check to
 * make sure the PLL reg is writable first though, since the panel write
 * protect mechanism may be enabled.
 *
 * Note!  This is for pre-ILK only.
 *
 * Unfortunately needed by dvo_ns2501 since the dvo depends on it running.
 */
static void intel_enable_pll(struct drm_i915_private *dev_priv, enum pipe pipe)
{
	int reg;
	u32 val;

	/* No really, not for ILK+ */
	BUG_ON(!IS_VALLEYVIEW(dev_priv->dev) && dev_priv->info->gen >= 5);

	/* PLL is protected by panel, make sure we can write it */
	if (IS_MOBILE(dev_priv->dev) && !IS_I830(dev_priv->dev))
		assert_panel_unlocked(dev_priv, pipe);

	reg = DPLL(pipe);
	val = I915_READ(reg);
	val |= DPLL_VCO_ENABLE;

	/* We do this three times for luck */
	I915_WRITE(reg, val);
	POSTING_READ(reg);
	udelay(150); /* wait for warmup */
	I915_WRITE(reg, val);
	POSTING_READ(reg);
	udelay(150); /* wait for warmup */
	I915_WRITE(reg, val);
	POSTING_READ(reg);
	udelay(150); /* wait for warmup */
}

/**
 * intel_disable_pll - disable a PLL
 * @dev_priv: i915 private structure
 * @pipe: pipe PLL to disable
 *
 * Disable the PLL for @pipe, making sure the pipe is off first.
 *
 * Note!  This is for pre-ILK only.
 */
static void intel_disable_pll(struct drm_i915_private *dev_priv, enum pipe pipe)
{
	int reg;
	u32 val;

	/* Don't disable pipe A or pipe A PLLs if needed */
	if (pipe == PIPE_A && (dev_priv->quirks & QUIRK_PIPEA_FORCE))
		return;

	/* Make sure the pipe isn't still relying on us */
	assert_pipe_disabled(dev_priv, pipe);

	reg = DPLL(pipe);
	val = I915_READ(reg);
	val &= ~DPLL_VCO_ENABLE;
	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

/* SBI access */
static void
intel_sbi_write(struct drm_i915_private *dev_priv, u16 reg, u32 value,
		enum intel_sbi_destination destination)
{
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&dev_priv->dpio_lock, flags);
	if (wait_for((I915_READ(SBI_CTL_STAT) & SBI_BUSY) == 0, 100)) {
		DRM_ERROR("timeout waiting for SBI to become ready\n");
		goto out_unlock;
	}

	I915_WRITE(SBI_ADDR, (reg << 16));
	I915_WRITE(SBI_DATA, value);

	if (destination == SBI_ICLK)
		tmp = SBI_CTL_DEST_ICLK | SBI_CTL_OP_CRWR;
	else
		tmp = SBI_CTL_DEST_MPHY | SBI_CTL_OP_IOWR;
	I915_WRITE(SBI_CTL_STAT, SBI_BUSY | tmp);

	if (wait_for((I915_READ(SBI_CTL_STAT) & (SBI_BUSY | SBI_RESPONSE_FAIL)) == 0,
				100)) {
		DRM_ERROR("timeout waiting for SBI to complete write transaction\n");
		goto out_unlock;
	}

out_unlock:
	spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
}

static u32
intel_sbi_read(struct drm_i915_private *dev_priv, u16 reg,
	       enum intel_sbi_destination destination)
{
	unsigned long flags;
	u32 value = 0;

	spin_lock_irqsave(&dev_priv->dpio_lock, flags);
	if (wait_for((I915_READ(SBI_CTL_STAT) & SBI_BUSY) == 0, 100)) {
		DRM_ERROR("timeout waiting for SBI to become ready\n");
		goto out_unlock;
	}

	I915_WRITE(SBI_ADDR, (reg << 16));

	if (destination == SBI_ICLK)
		value = SBI_CTL_DEST_ICLK | SBI_CTL_OP_CRRD;
	else
		value = SBI_CTL_DEST_MPHY | SBI_CTL_OP_IORD;
	I915_WRITE(SBI_CTL_STAT, value | SBI_BUSY);

	if (wait_for((I915_READ(SBI_CTL_STAT) & (SBI_BUSY | SBI_RESPONSE_FAIL)) == 0,
				100)) {
		DRM_ERROR("timeout waiting for SBI to complete read transaction\n");
		goto out_unlock;
	}

	value = I915_READ(SBI_DATA);

out_unlock:
	spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
	return value;
}

/**
 * ironlake_enable_pch_pll - enable PCH PLL
 * @dev_priv: i915 private structure
 * @pipe: pipe PLL to enable
 *
 * The PCH PLL needs to be enabled before the PCH transcoder, since it
 * drives the transcoder clock.
 */
static void ironlake_enable_pch_pll(struct intel_crtc *intel_crtc)
{
	struct drm_i915_private *dev_priv = intel_crtc->base.dev->dev_private;
	struct intel_pch_pll *pll;
	int reg;
	u32 val;

	/* PCH PLLs only available on ILK, SNB and IVB */
	BUG_ON(dev_priv->info->gen < 5);
	pll = intel_crtc->pch_pll;
	if (pll == NULL)
		return;

	if (WARN_ON(pll->refcount == 0))
		return;

	DRM_DEBUG_KMS("enable PCH PLL %x (active %d, on? %d)for crtc %d\n",
		      pll->pll_reg, pll->active, pll->on,
		      intel_crtc->base.base.id);

	/* PCH refclock must be enabled first */
	assert_pch_refclk_enabled(dev_priv);

	if (pll->active++ && pll->on) {
		assert_pch_pll_enabled(dev_priv, pll, NULL);
		return;
	}

	DRM_DEBUG_KMS("enabling PCH PLL %x\n", pll->pll_reg);

	reg = pll->pll_reg;
	val = I915_READ(reg);
	val |= DPLL_VCO_ENABLE;
	I915_WRITE(reg, val);
	POSTING_READ(reg);
	udelay(200);

	pll->on = true;
}

static void intel_disable_pch_pll(struct intel_crtc *intel_crtc)
{
	struct drm_i915_private *dev_priv = intel_crtc->base.dev->dev_private;
	struct intel_pch_pll *pll = intel_crtc->pch_pll;
	int reg;
	u32 val;

	/* PCH only available on ILK+ */
	BUG_ON(dev_priv->info->gen < 5);
	if (pll == NULL)
	       return;

	if (WARN_ON(pll->refcount == 0))
		return;

	DRM_DEBUG_KMS("disable PCH PLL %x (active %d, on? %d) for crtc %d\n",
		      pll->pll_reg, pll->active, pll->on,
		      intel_crtc->base.base.id);

	if (WARN_ON(pll->active == 0)) {
		assert_pch_pll_disabled(dev_priv, pll, NULL);
		return;
	}

	if (--pll->active) {
		assert_pch_pll_enabled(dev_priv, pll, NULL);
		return;
	}

	DRM_DEBUG_KMS("disabling PCH PLL %x\n", pll->pll_reg);

	/* Make sure transcoder isn't still depending on us */
	assert_transcoder_disabled(dev_priv, intel_crtc->pipe);

	reg = pll->pll_reg;
	val = I915_READ(reg);
	val &= ~DPLL_VCO_ENABLE;
	I915_WRITE(reg, val);
	POSTING_READ(reg);
	udelay(200);

	pll->on = false;
}

static void ironlake_enable_pch_transcoder(struct drm_i915_private *dev_priv,
					   enum pipe pipe)
{
	struct drm_device *dev = dev_priv->dev;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
	uint32_t reg, val, pipeconf_val;

	/* PCH only available on ILK+ */
	BUG_ON(dev_priv->info->gen < 5);

	/* Make sure PCH DPLL is enabled */
	assert_pch_pll_enabled(dev_priv,
			       to_intel_crtc(crtc)->pch_pll,
			       to_intel_crtc(crtc));

	/* FDI must be feeding us bits for PCH ports */
	assert_fdi_tx_enabled(dev_priv, pipe);
	assert_fdi_rx_enabled(dev_priv, pipe);

	if (HAS_PCH_CPT(dev)) {
		/* Workaround: Set the timing override bit before enabling the
		 * pch transcoder. */
		reg = TRANS_CHICKEN2(pipe);
		val = I915_READ(reg);
		val |= TRANS_CHICKEN2_TIMING_OVERRIDE;
		I915_WRITE(reg, val);
	}

	reg = TRANSCONF(pipe);
	val = I915_READ(reg);
	pipeconf_val = I915_READ(PIPECONF(pipe));

	if (HAS_PCH_IBX(dev_priv->dev)) {
		/*
		 * make the BPC in transcoder be consistent with
		 * that in pipeconf reg.
		 */
		val &= ~PIPE_BPC_MASK;
		val |= pipeconf_val & PIPE_BPC_MASK;
	}

	val &= ~TRANS_INTERLACE_MASK;
	if ((pipeconf_val & PIPECONF_INTERLACE_MASK) == PIPECONF_INTERLACED_ILK)
		if (HAS_PCH_IBX(dev_priv->dev) &&
		    intel_pipe_has_type(crtc, INTEL_OUTPUT_SDVO))
			val |= TRANS_LEGACY_INTERLACED_ILK;
		else
			val |= TRANS_INTERLACED;
	else
		val |= TRANS_PROGRESSIVE;

	I915_WRITE(reg, val | TRANS_ENABLE);
	if (wait_for(I915_READ(reg) & TRANS_STATE_ENABLE, 100))
		DRM_ERROR("failed to enable transcoder %d\n", pipe);
}

static void lpt_enable_pch_transcoder(struct drm_i915_private *dev_priv,
				      enum transcoder cpu_transcoder)
{
	u32 val, pipeconf_val;

	/* PCH only available on ILK+ */
	BUG_ON(dev_priv->info->gen < 5);

	/* FDI must be feeding us bits for PCH ports */
	assert_fdi_tx_enabled(dev_priv, cpu_transcoder);
	assert_fdi_rx_enabled(dev_priv, TRANSCODER_A);

	/* Workaround: set timing override bit. */
	val = I915_READ(_TRANSA_CHICKEN2);
	val |= TRANS_CHICKEN2_TIMING_OVERRIDE;
	I915_WRITE(_TRANSA_CHICKEN2, val);

	val = TRANS_ENABLE;
	pipeconf_val = I915_READ(PIPECONF(cpu_transcoder));

	if ((pipeconf_val & PIPECONF_INTERLACE_MASK_HSW) ==
	    PIPECONF_INTERLACED_ILK)
		val |= TRANS_INTERLACED;
	else
		val |= TRANS_PROGRESSIVE;

	I915_WRITE(TRANSCONF(TRANSCODER_A), val);
	if (wait_for(I915_READ(_TRANSACONF) & TRANS_STATE_ENABLE, 100))
		DRM_ERROR("Failed to enable PCH transcoder\n");
}

static void ironlake_disable_pch_transcoder(struct drm_i915_private *dev_priv,
					    enum pipe pipe)
{
	struct drm_device *dev = dev_priv->dev;
	uint32_t reg, val;

	/* FDI relies on the transcoder */
	assert_fdi_tx_disabled(dev_priv, pipe);
	assert_fdi_rx_disabled(dev_priv, pipe);

	/* Ports must be off as well */
	assert_pch_ports_disabled(dev_priv, pipe);

	reg = TRANSCONF(pipe);
	val = I915_READ(reg);
	val &= ~TRANS_ENABLE;
	I915_WRITE(reg, val);
	/* wait for PCH transcoder off, transcoder state */
	if (wait_for((I915_READ(reg) & TRANS_STATE_ENABLE) == 0, 50))
		DRM_ERROR("failed to disable transcoder %d\n", pipe);

	if (!HAS_PCH_IBX(dev)) {
		/* Workaround: Clear the timing override chicken bit again. */
		reg = TRANS_CHICKEN2(pipe);
		val = I915_READ(reg);
		val &= ~TRANS_CHICKEN2_TIMING_OVERRIDE;
		I915_WRITE(reg, val);
	}
}

static void lpt_disable_pch_transcoder(struct drm_i915_private *dev_priv)
{
	u32 val;

	val = I915_READ(_TRANSACONF);
	val &= ~TRANS_ENABLE;
	I915_WRITE(_TRANSACONF, val);
	/* wait for PCH transcoder off, transcoder state */
	if (wait_for((I915_READ(_TRANSACONF) & TRANS_STATE_ENABLE) == 0, 50))
		DRM_ERROR("Failed to disable PCH transcoder\n");

	/* Workaround: clear timing override bit. */
	val = I915_READ(_TRANSA_CHICKEN2);
	val &= ~TRANS_CHICKEN2_TIMING_OVERRIDE;
	I915_WRITE(_TRANSA_CHICKEN2, val);
}

/**
 * intel_enable_pipe - enable a pipe, asserting requirements
 * @dev_priv: i915 private structure
 * @pipe: pipe to enable
 * @pch_port: on ILK+, is this pipe driving a PCH port or not
 *
 * Enable @pipe, making sure that various hardware specific requirements
 * are met, if applicable, e.g. PLL enabled, LVDS pairs enabled, etc.
 *
 * @pipe should be %PIPE_A or %PIPE_B.
 *
 * Will wait until the pipe is actually running (i.e. first vblank) before
 * returning.
 */
static void intel_enable_pipe(struct drm_i915_private *dev_priv, enum pipe pipe,
			      bool pch_port)
{
	enum transcoder cpu_transcoder = intel_pipe_to_cpu_transcoder(dev_priv,
								      pipe);
	enum transcoder pch_transcoder;
	int reg;
	u32 val;

	if (IS_HASWELL(dev_priv->dev))
		pch_transcoder = TRANSCODER_A;
	else
		pch_transcoder = pipe;

	/*
	 * A pipe without a PLL won't actually be able to drive bits from
	 * a plane.  On ILK+ the pipe PLLs are integrated, so we don't
	 * need the check.
	 */
	if (!HAS_PCH_SPLIT(dev_priv->dev))
		assert_pll_enabled(dev_priv, pipe);
	else {
		if (pch_port) {
			/* if driving the PCH, we need FDI enabled */
			assert_fdi_rx_pll_enabled(dev_priv, pch_transcoder);
			assert_fdi_tx_pll_enabled(dev_priv, cpu_transcoder);
		}
		/* FIXME: assert CPU port conditions for SNB+ */
	}

	reg = PIPECONF(cpu_transcoder);
	val = I915_READ(reg);
	if (val & PIPECONF_ENABLE)
		return;

	I915_WRITE(reg, val | PIPECONF_ENABLE);
	intel_wait_for_vblank(dev_priv->dev, pipe);
}

/**
 * intel_disable_pipe - disable a pipe, asserting requirements
 * @dev_priv: i915 private structure
 * @pipe: pipe to disable
 *
 * Disable @pipe, making sure that various hardware specific requirements
 * are met, if applicable, e.g. plane disabled, panel fitter off, etc.
 *
 * @pipe should be %PIPE_A or %PIPE_B.
 *
 * Will wait until the pipe has shut down before returning.
 */
static void intel_disable_pipe(struct drm_i915_private *dev_priv,
			       enum pipe pipe)
{
	enum transcoder cpu_transcoder = intel_pipe_to_cpu_transcoder(dev_priv,
								      pipe);
	int reg;
	u32 val;

	/*
	 * Make sure planes won't keep trying to pump pixels to us,
	 * or we might hang the display.
	 */
	assert_planes_disabled(dev_priv, pipe);
	assert_sprites_disabled(dev_priv, pipe);

	/* Don't disable pipe A or pipe A PLLs if needed */
	if (pipe == PIPE_A && (dev_priv->quirks & QUIRK_PIPEA_FORCE))
		return;

	reg = PIPECONF(cpu_transcoder);
	val = I915_READ(reg);
	if ((val & PIPECONF_ENABLE) == 0)
		return;

	I915_WRITE(reg, val & ~PIPECONF_ENABLE);
	intel_wait_for_pipe_off(dev_priv->dev, pipe);
}

/*
 * Plane regs are double buffered, going from enabled->disabled needs a
 * trigger in order to latch.  The display address reg provides this.
 */
void intel_flush_display_plane(struct drm_i915_private *dev_priv,
				      enum plane plane)
{
	if (dev_priv->info->gen >= 4)
		I915_WRITE(DSPSURF(plane), I915_READ(DSPSURF(plane)));
	else
		I915_WRITE(DSPADDR(plane), I915_READ(DSPADDR(plane)));
}

/**
 * intel_enable_plane - enable a display plane on a given pipe
 * @dev_priv: i915 private structure
 * @plane: plane to enable
 * @pipe: pipe being fed
 *
 * Enable @plane on @pipe, making sure that @pipe is running first.
 */
static void intel_enable_plane(struct drm_i915_private *dev_priv,
			       enum plane plane, enum pipe pipe)
{
	int reg;
	u32 val;

	/* If the pipe isn't enabled, we can't pump pixels and may hang */
	assert_pipe_enabled(dev_priv, pipe);

	reg = DSPCNTR(plane);
	val = I915_READ(reg);
	if (val & DISPLAY_PLANE_ENABLE)
		return;

	I915_WRITE(reg, val | DISPLAY_PLANE_ENABLE);
	intel_flush_display_plane(dev_priv, plane);
	intel_wait_for_vblank(dev_priv->dev, pipe);
}

/**
 * intel_disable_plane - disable a display plane
 * @dev_priv: i915 private structure
 * @plane: plane to disable
 * @pipe: pipe consuming the data
 *
 * Disable @plane; should be an independent operation.
 */
static void intel_disable_plane(struct drm_i915_private *dev_priv,
				enum plane plane, enum pipe pipe)
{
	int reg;
	u32 val;

	reg = DSPCNTR(plane);
	val = I915_READ(reg);
	if ((val & DISPLAY_PLANE_ENABLE) == 0)
		return;

	I915_WRITE(reg, val & ~DISPLAY_PLANE_ENABLE);
	intel_flush_display_plane(dev_priv, plane);
	intel_wait_for_vblank(dev_priv->dev, pipe);
}

static bool need_vtd_wa(struct drm_device *dev)
{
#ifdef CONFIG_INTEL_IOMMU
	if (INTEL_INFO(dev)->gen >= 6 && intel_iommu_gfx_mapped)
		return true;
#endif
	return false;
}

int
intel_pin_and_fence_fb_obj(struct drm_device *dev,
			   struct drm_i915_gem_object *obj,
			   struct intel_ring_buffer *pipelined)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 alignment;
	int ret;

	switch (obj->tiling_mode) {
	case I915_TILING_NONE:
		if (IS_BROADWATER(dev) || IS_CRESTLINE(dev))
			alignment = 128 * 1024;
		else if (INTEL_INFO(dev)->gen >= 4)
			alignment = 4 * 1024;
		else
			alignment = 64 * 1024;
		break;
	case I915_TILING_X:
		/* pin() will align the object as required by fence */
		alignment = 0;
		break;
	case I915_TILING_Y:
		/* Despite that we check this in framebuffer_init userspace can
		 * screw us over and change the tiling after the fact. Only
		 * pinned buffers can't change their tiling. */
		DRM_DEBUG_DRIVER("Y tiled not allowed for scan out buffers\n");
		return -EINVAL;
	default:
		BUG();
	}

	/* Note that the w/a also requires 64 PTE of padding following the
	 * bo. We currently fill all unused PTE with the shadow page and so
	 * we should always have valid PTE following the scanout preventing
	 * the VT-d warning.
	 */
	if (need_vtd_wa(dev) && alignment < 256 * 1024)
		alignment = 256 * 1024;

	dev_priv->mm.interruptible = false;
	ret = i915_gem_object_pin_to_display_plane(obj, alignment, pipelined);
	if (ret)
		goto err_interruptible;

	/* Install a fence for tiled scan-out. Pre-i965 always needs a
	 * fence, whereas 965+ only requires a fence if using
	 * framebuffer compression.  For simplicity, we always install
	 * a fence as the cost is not that onerous.
	 */
	ret = i915_gem_object_get_fence(obj);
	if (ret)
		goto err_unpin;

	i915_gem_object_pin_fence(obj);

	dev_priv->mm.interruptible = true;
	return 0;

err_unpin:
	i915_gem_object_unpin(obj);
err_interruptible:
	dev_priv->mm.interruptible = true;
	return ret;
}

void intel_unpin_fb_obj(struct drm_i915_gem_object *obj)
{
	i915_gem_object_unpin_fence(obj);
	i915_gem_object_unpin(obj);
}

/* Computes the linear offset to the base tile and adjusts x, y. bytes per pixel
 * is assumed to be a power-of-two. */
unsigned long intel_gen4_compute_offset_xtiled(int *x, int *y,
					       unsigned int bpp,
					       unsigned int pitch)
{
	int tile_rows, tiles;

	tile_rows = *y / 8;
	*y %= 8;
	tiles = *x / (512/bpp);
	*x %= 512/bpp;

	return tile_rows * pitch * 8 + tiles * 4096;
}

static int i9xx_update_plane(struct drm_crtc *crtc, struct drm_framebuffer *fb,
			     int x, int y)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_framebuffer *intel_fb;
	struct drm_i915_gem_object *obj;
	int plane = intel_crtc->plane;
	unsigned long linear_offset;
	u32 dspcntr;
	u32 reg;

	switch (plane) {
	case 0:
	case 1:
		break;
	default:
		DRM_ERROR("Can't update plane %d in SAREA\n", plane);
		return -EINVAL;
	}

	intel_fb = to_intel_framebuffer(fb);
	obj = intel_fb->obj;

	reg = DSPCNTR(plane);
	dspcntr = I915_READ(reg);
	/* Mask out pixel format bits in case we change it */
	dspcntr &= ~DISPPLANE_PIXFORMAT_MASK;
	switch (fb->pixel_format) {
	case DRM_FORMAT_C8:
		dspcntr |= DISPPLANE_8BPP;
		break;
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ARGB1555:
		dspcntr |= DISPPLANE_BGRX555;
		break;
	case DRM_FORMAT_RGB565:
		dspcntr |= DISPPLANE_BGRX565;
		break;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		dspcntr |= DISPPLANE_BGRX888;
		break;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		dspcntr |= DISPPLANE_RGBX888;
		break;
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
		dspcntr |= DISPPLANE_BGRX101010;
		break;
	case DRM_FORMAT_XBGR2101010:
	case DRM_FORMAT_ABGR2101010:
		dspcntr |= DISPPLANE_RGBX101010;
		break;
	default:
		BUG();
	}

	if (INTEL_INFO(dev)->gen >= 4) {
		if (obj->tiling_mode != I915_TILING_NONE)
			dspcntr |= DISPPLANE_TILED;
		else
			dspcntr &= ~DISPPLANE_TILED;
	}

	I915_WRITE(reg, dspcntr);

	linear_offset = y * fb->pitches[0] + x * (fb->bits_per_pixel / 8);

	if (INTEL_INFO(dev)->gen >= 4) {
		intel_crtc->dspaddr_offset =
			intel_gen4_compute_offset_xtiled(&x, &y,
							 fb->bits_per_pixel / 8,
							 fb->pitches[0]);
		linear_offset -= intel_crtc->dspaddr_offset;
	} else {
		intel_crtc->dspaddr_offset = linear_offset;
	}

	DRM_DEBUG_KMS("Writing base %08X %08lX %d %d %d\n",
		      obj->gtt_offset, linear_offset, x, y, fb->pitches[0]);
	I915_WRITE(DSPSTRIDE(plane), fb->pitches[0]);
	if (INTEL_INFO(dev)->gen >= 4) {
		I915_MODIFY_DISPBASE(DSPSURF(plane),
				     obj->gtt_offset + intel_crtc->dspaddr_offset);
		I915_WRITE(DSPTILEOFF(plane), (y << 16) | x);
		I915_WRITE(DSPLINOFF(plane), linear_offset);
	} else
		I915_WRITE(DSPADDR(plane), obj->gtt_offset + linear_offset);
	POSTING_READ(reg);

	return 0;
}

static int ironlake_update_plane(struct drm_crtc *crtc,
				 struct drm_framebuffer *fb, int x, int y)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_framebuffer *intel_fb;
	struct drm_i915_gem_object *obj;
	int plane = intel_crtc->plane;
	unsigned long linear_offset;
	u32 dspcntr;
	u32 reg;

	switch (plane) {
	case 0:
	case 1:
	case 2:
		break;
	default:
		DRM_ERROR("Can't update plane %d in SAREA\n", plane);
		return -EINVAL;
	}

	intel_fb = to_intel_framebuffer(fb);
	obj = intel_fb->obj;

	reg = DSPCNTR(plane);
	dspcntr = I915_READ(reg);
	/* Mask out pixel format bits in case we change it */
	dspcntr &= ~DISPPLANE_PIXFORMAT_MASK;
	switch (fb->pixel_format) {
	case DRM_FORMAT_C8:
		dspcntr |= DISPPLANE_8BPP;
		break;
	case DRM_FORMAT_RGB565:
		dspcntr |= DISPPLANE_BGRX565;
		break;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		dspcntr |= DISPPLANE_BGRX888;
		break;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		dspcntr |= DISPPLANE_RGBX888;
		break;
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
		dspcntr |= DISPPLANE_BGRX101010;
		break;
	case DRM_FORMAT_XBGR2101010:
	case DRM_FORMAT_ABGR2101010:
		dspcntr |= DISPPLANE_RGBX101010;
		break;
	default:
		BUG();
	}

	if (obj->tiling_mode != I915_TILING_NONE)
		dspcntr |= DISPPLANE_TILED;
	else
		dspcntr &= ~DISPPLANE_TILED;

	/* must disable */
	dspcntr |= DISPPLANE_TRICKLE_FEED_DISABLE;

	I915_WRITE(reg, dspcntr);

	linear_offset = y * fb->pitches[0] + x * (fb->bits_per_pixel / 8);
	intel_crtc->dspaddr_offset =
		intel_gen4_compute_offset_xtiled(&x, &y,
						 fb->bits_per_pixel / 8,
						 fb->pitches[0]);
	linear_offset -= intel_crtc->dspaddr_offset;

	DRM_DEBUG_KMS("Writing base %08X %08lX %d %d %d\n",
		      obj->gtt_offset, linear_offset, x, y, fb->pitches[0]);
	I915_WRITE(DSPSTRIDE(plane), fb->pitches[0]);
	I915_MODIFY_DISPBASE(DSPSURF(plane),
			     obj->gtt_offset + intel_crtc->dspaddr_offset);
	if (IS_HASWELL(dev)) {
		I915_WRITE(DSPOFFSET(plane), (y << 16) | x);
	} else {
		I915_WRITE(DSPTILEOFF(plane), (y << 16) | x);
		I915_WRITE(DSPLINOFF(plane), linear_offset);
	}
	POSTING_READ(reg);

	return 0;
}

/* Assume fb object is pinned & idle & fenced and just update base pointers */
static int
intel_pipe_set_base_atomic(struct drm_crtc *crtc, struct drm_framebuffer *fb,
			   int x, int y, enum mode_set_atomic state)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (dev_priv->display.disable_fbc)
		dev_priv->display.disable_fbc(dev);
	intel_increase_pllclock(crtc);

	return dev_priv->display.update_plane(crtc, fb, x, y);
}

void intel_display_handle_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;

	/*
	 * Flips in the rings have been nuked by the reset,
	 * so complete all pending flips so that user space
	 * will get its events and not get stuck.
	 *
	 * Also update the base address of all primary
	 * planes to the the last fb to make sure we're
	 * showing the correct fb after a reset.
	 *
	 * Need to make two loops over the crtcs so that we
	 * don't try to grab a crtc mutex before the
	 * pending_flip_queue really got woken up.
	 */

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
		enum plane plane = intel_crtc->plane;

		intel_prepare_page_flip(dev, plane);
		intel_finish_page_flip_plane(dev, plane);
	}

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

		mutex_lock(&crtc->mutex);
		if (intel_crtc->active)
			dev_priv->display.update_plane(crtc, crtc->fb,
						       crtc->x, crtc->y);
		mutex_unlock(&crtc->mutex);
	}
}

static int
intel_finish_fb(struct drm_framebuffer *old_fb)
{
	struct drm_i915_gem_object *obj = to_intel_framebuffer(old_fb)->obj;
	struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
	bool was_interruptible = dev_priv->mm.interruptible;
	int ret;

	wait_event(dev_priv->pending_flip_queue,
		   atomic_read(&dev_priv->mm.wedged) ||
		   atomic_read(&obj->pending_flip) == 0);

	/* Big Hammer, we also need to ensure that any pending
	 * MI_WAIT_FOR_EVENT inside a user batch buffer on the
	 * current scanout is retired before unpinning the old
	 * framebuffer.
	 *
	 * This should only fail upon a hung GPU, in which case we
	 * can safely continue.
	 */
	dev_priv->mm.interruptible = false;
	ret = i915_gem_object_finish_gpu(obj);
	dev_priv->mm.interruptible = was_interruptible;

	return ret;
}

static void intel_crtc_update_sarea_pos(struct drm_crtc *crtc, int x, int y)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_master_private *master_priv;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	if (!dev->primary->master)
		return;

	master_priv = dev->primary->master->driver_priv;
	if (!master_priv->sarea_priv)
		return;

	switch (intel_crtc->pipe) {
	case 0:
		master_priv->sarea_priv->pipeA_x = x;
		master_priv->sarea_priv->pipeA_y = y;
		break;
	case 1:
		master_priv->sarea_priv->pipeB_x = x;
		master_priv->sarea_priv->pipeB_y = y;
		break;
	default:
		break;
	}
}

static int
intel_pipe_set_base(struct drm_crtc *crtc, int x, int y,
		    struct drm_framebuffer *fb)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct drm_framebuffer *old_fb;
	int ret;

	/* no fb bound */
	if (!fb) {
		DRM_ERROR("No FB bound\n");
		return 0;
	}

	if (intel_crtc->plane > INTEL_INFO(dev)->num_pipes) {
		DRM_ERROR("no plane for crtc: plane %d, num_pipes %d\n",
				intel_crtc->plane,
				INTEL_INFO(dev)->num_pipes);
		return -EINVAL;
	}

	mutex_lock(&dev->struct_mutex);
	ret = intel_pin_and_fence_fb_obj(dev,
					 to_intel_framebuffer(fb)->obj,
					 NULL);
	if (ret != 0) {
		mutex_unlock(&dev->struct_mutex);
		DRM_ERROR("pin & fence failed\n");
		return ret;
	}

	ret = dev_priv->display.update_plane(crtc, fb, x, y);
	if (ret) {
		intel_unpin_fb_obj(to_intel_framebuffer(fb)->obj);
		mutex_unlock(&dev->struct_mutex);
		DRM_ERROR("failed to update base address\n");
		return ret;
	}

	old_fb = crtc->fb;
	crtc->fb = fb;
	crtc->x = x;
	crtc->y = y;

	if (old_fb) {
		intel_wait_for_vblank(dev, intel_crtc->pipe);
		intel_unpin_fb_obj(to_intel_framebuffer(old_fb)->obj);
	}

	intel_update_fbc(dev);
	mutex_unlock(&dev->struct_mutex);

	intel_crtc_update_sarea_pos(crtc, x, y);

	return 0;
}

static void ironlake_set_pll_edp(struct drm_crtc *crtc, int clock)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 dpa_ctl;

	DRM_DEBUG_KMS("eDP PLL enable for clock %d\n", clock);
	dpa_ctl = I915_READ(DP_A);
	dpa_ctl &= ~DP_PLL_FREQ_MASK;

	if (clock < 200000) {
		u32 temp;
		dpa_ctl |= DP_PLL_FREQ_160MHZ;
		/* workaround for 160Mhz:
		   1) program 0x4600c bits 15:0 = 0x8124
		   2) program 0x46010 bit 0 = 1
		   3) program 0x46034 bit 24 = 1
		   4) program 0x64000 bit 14 = 1
		   */
		temp = I915_READ(0x4600c);
		temp &= 0xffff0000;
		I915_WRITE(0x4600c, temp | 0x8124);

		temp = I915_READ(0x46010);
		I915_WRITE(0x46010, temp | 1);

		temp = I915_READ(0x46034);
		I915_WRITE(0x46034, temp | (1 << 24));
	} else {
		dpa_ctl |= DP_PLL_FREQ_270MHZ;
	}
	I915_WRITE(DP_A, dpa_ctl);

	POSTING_READ(DP_A);
	udelay(500);
}

static void intel_fdi_normal_train(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	u32 reg, temp;

	/* enable normal train */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	if (IS_IVYBRIDGE(dev)) {
		temp &= ~FDI_LINK_TRAIN_NONE_IVB;
		temp |= FDI_LINK_TRAIN_NONE_IVB | FDI_TX_ENHANCE_FRAME_ENABLE;
	} else {
		temp &= ~FDI_LINK_TRAIN_NONE;
		temp |= FDI_LINK_TRAIN_NONE | FDI_TX_ENHANCE_FRAME_ENABLE;
	}
	I915_WRITE(reg, temp);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	if (HAS_PCH_CPT(dev)) {
		temp &= ~FDI_LINK_TRAIN_PATTERN_MASK_CPT;
		temp |= FDI_LINK_TRAIN_NORMAL_CPT;
	} else {
		temp &= ~FDI_LINK_TRAIN_NONE;
		temp |= FDI_LINK_TRAIN_NONE;
	}
	I915_WRITE(reg, temp | FDI_RX_ENHANCE_FRAME_ENABLE);

	/* wait one idle pattern time */
	POSTING_READ(reg);
	udelay(1000);

	/* IVB wants error correction enabled */
	if (IS_IVYBRIDGE(dev))
		I915_WRITE(reg, I915_READ(reg) | FDI_FS_ERRC_ENABLE |
			   FDI_FE_ERRC_ENABLE);
}

/* The FDI link training functions for ILK/Ibexpeak. */
static void ironlake_fdi_link_train(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	u32 reg, temp, tries;

	/* FDI needs bits from pipe & plane first */
	assert_pipe_enabled(dev_priv, pipe);
	assert_plane_enabled(dev_priv, plane);

	/* Train 1: umask FDI RX Interrupt symbol_lock and bit_lock bit
	   for train result */
	reg = FDI_RX_IMR(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_RX_SYMBOL_LOCK;
	temp &= ~FDI_RX_BIT_LOCK;
	I915_WRITE(reg, temp);
	I915_READ(reg);
	udelay(150);

	/* enable CPU FDI TX and PCH FDI RX */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~(7 << 19);
	temp |= (intel_crtc->fdi_lanes - 1) << 19;
	temp &= ~FDI_LINK_TRAIN_NONE;
	temp |= FDI_LINK_TRAIN_PATTERN_1;
	I915_WRITE(reg, temp | FDI_TX_ENABLE);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_LINK_TRAIN_NONE;
	temp |= FDI_LINK_TRAIN_PATTERN_1;
	I915_WRITE(reg, temp | FDI_RX_ENABLE);

	POSTING_READ(reg);
	udelay(150);

	/* Ironlake workaround, enable clock pointer after FDI enable*/
	I915_WRITE(FDI_RX_CHICKEN(pipe), FDI_RX_PHASE_SYNC_POINTER_OVR);
	I915_WRITE(FDI_RX_CHICKEN(pipe), FDI_RX_PHASE_SYNC_POINTER_OVR |
		   FDI_RX_PHASE_SYNC_POINTER_EN);

	reg = FDI_RX_IIR(pipe);
	for (tries = 0; tries < 5; tries++) {
		temp = I915_READ(reg);
		DRM_DEBUG_KMS("FDI_RX_IIR 0x%x\n", temp);

		if ((temp & FDI_RX_BIT_LOCK)) {
			DRM_DEBUG_KMS("FDI train 1 done.\n");
			I915_WRITE(reg, temp | FDI_RX_BIT_LOCK);
			break;
		}
	}
	if (tries == 5)
		DRM_ERROR("FDI train 1 fail!\n");

	/* Train 2 */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_LINK_TRAIN_NONE;
	temp |= FDI_LINK_TRAIN_PATTERN_2;
	I915_WRITE(reg, temp);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_LINK_TRAIN_NONE;
	temp |= FDI_LINK_TRAIN_PATTERN_2;
	I915_WRITE(reg, temp);

	POSTING_READ(reg);
	udelay(150);

	reg = FDI_RX_IIR(pipe);
	for (tries = 0; tries < 5; tries++) {
		temp = I915_READ(reg);
		DRM_DEBUG_KMS("FDI_RX_IIR 0x%x\n", temp);

		if (temp & FDI_RX_SYMBOL_LOCK) {
			I915_WRITE(reg, temp | FDI_RX_SYMBOL_LOCK);
			DRM_DEBUG_KMS("FDI train 2 done.\n");
			break;
		}
	}
	if (tries == 5)
		DRM_ERROR("FDI train 2 fail!\n");

	DRM_DEBUG_KMS("FDI train done\n");

}

static const int snb_b_fdi_train_param[] = {
	FDI_LINK_TRAIN_400MV_0DB_SNB_B,
	FDI_LINK_TRAIN_400MV_6DB_SNB_B,
	FDI_LINK_TRAIN_600MV_3_5DB_SNB_B,
	FDI_LINK_TRAIN_800MV_0DB_SNB_B,
};

/* The FDI link training functions for SNB/Cougarpoint. */
static void gen6_fdi_link_train(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	u32 reg, temp, i, retry;

	/* Train 1: umask FDI RX Interrupt symbol_lock and bit_lock bit
	   for train result */
	reg = FDI_RX_IMR(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_RX_SYMBOL_LOCK;
	temp &= ~FDI_RX_BIT_LOCK;
	I915_WRITE(reg, temp);

	POSTING_READ(reg);
	udelay(150);

	/* enable CPU FDI TX and PCH FDI RX */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~(7 << 19);
	temp |= (intel_crtc->fdi_lanes - 1) << 19;
	temp &= ~FDI_LINK_TRAIN_NONE;
	temp |= FDI_LINK_TRAIN_PATTERN_1;
	temp &= ~FDI_LINK_TRAIN_VOL_EMP_MASK;
	/* SNB-B */
	temp |= FDI_LINK_TRAIN_400MV_0DB_SNB_B;
	I915_WRITE(reg, temp | FDI_TX_ENABLE);

	I915_WRITE(FDI_RX_MISC(pipe),
		   FDI_RX_TP1_TO_TP2_48 | FDI_RX_FDI_DELAY_90);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	if (HAS_PCH_CPT(dev)) {
		temp &= ~FDI_LINK_TRAIN_PATTERN_MASK_CPT;
		temp |= FDI_LINK_TRAIN_PATTERN_1_CPT;
	} else {
		temp &= ~FDI_LINK_TRAIN_NONE;
		temp |= FDI_LINK_TRAIN_PATTERN_1;
	}
	I915_WRITE(reg, temp | FDI_RX_ENABLE);

	POSTING_READ(reg);
	udelay(150);

	for (i = 0; i < 4; i++) {
		reg = FDI_TX_CTL(pipe);
		temp = I915_READ(reg);
		temp &= ~FDI_LINK_TRAIN_VOL_EMP_MASK;
		temp |= snb_b_fdi_train_param[i];
		I915_WRITE(reg, temp);

		POSTING_READ(reg);
		udelay(500);

		for (retry = 0; retry < 5; retry++) {
			reg = FDI_RX_IIR(pipe);
			temp = I915_READ(reg);
			DRM_DEBUG_KMS("FDI_RX_IIR 0x%x\n", temp);
			if (temp & FDI_RX_BIT_LOCK) {
				I915_WRITE(reg, temp | FDI_RX_BIT_LOCK);
				DRM_DEBUG_KMS("FDI train 1 done.\n");
				break;
			}
			udelay(50);
		}
		if (retry < 5)
			break;
	}
	if (i == 4)
		DRM_ERROR("FDI train 1 fail!\n");

	/* Train 2 */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_LINK_TRAIN_NONE;
	temp |= FDI_LINK_TRAIN_PATTERN_2;
	if (IS_GEN6(dev)) {
		temp &= ~FDI_LINK_TRAIN_VOL_EMP_MASK;
		/* SNB-B */
		temp |= FDI_LINK_TRAIN_400MV_0DB_SNB_B;
	}
	I915_WRITE(reg, temp);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	if (HAS_PCH_CPT(dev)) {
		temp &= ~FDI_LINK_TRAIN_PATTERN_MASK_CPT;
		temp |= FDI_LINK_TRAIN_PATTERN_2_CPT;
	} else {
		temp &= ~FDI_LINK_TRAIN_NONE;
		temp |= FDI_LINK_TRAIN_PATTERN_2;
	}
	I915_WRITE(reg, temp);

	POSTING_READ(reg);
	udelay(150);

	for (i = 0; i < 4; i++) {
		reg = FDI_TX_CTL(pipe);
		temp = I915_READ(reg);
		temp &= ~FDI_LINK_TRAIN_VOL_EMP_MASK;
		temp |= snb_b_fdi_train_param[i];
		I915_WRITE(reg, temp);

		POSTING_READ(reg);
		udelay(500);

		for (retry = 0; retry < 5; retry++) {
			reg = FDI_RX_IIR(pipe);
			temp = I915_READ(reg);
			DRM_DEBUG_KMS("FDI_RX_IIR 0x%x\n", temp);
			if (temp & FDI_RX_SYMBOL_LOCK) {
				I915_WRITE(reg, temp | FDI_RX_SYMBOL_LOCK);
				DRM_DEBUG_KMS("FDI train 2 done.\n");
				break;
			}
			udelay(50);
		}
		if (retry < 5)
			break;
	}
	if (i == 4)
		DRM_ERROR("FDI train 2 fail!\n");

	DRM_DEBUG_KMS("FDI train done.\n");
}

/* Manual link training for Ivy Bridge A0 parts */
static void ivb_manual_fdi_link_train(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	u32 reg, temp, i;

	/* Train 1: umask FDI RX Interrupt symbol_lock and bit_lock bit
	   for train result */
	reg = FDI_RX_IMR(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_RX_SYMBOL_LOCK;
	temp &= ~FDI_RX_BIT_LOCK;
	I915_WRITE(reg, temp);

	POSTING_READ(reg);
	udelay(150);

	DRM_DEBUG_KMS("FDI_RX_IIR before link train 0x%x\n",
		      I915_READ(FDI_RX_IIR(pipe)));

	/* enable CPU FDI TX and PCH FDI RX */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~(7 << 19);
	temp |= (intel_crtc->fdi_lanes - 1) << 19;
	temp &= ~(FDI_LINK_TRAIN_AUTO | FDI_LINK_TRAIN_NONE_IVB);
	temp |= FDI_LINK_TRAIN_PATTERN_1_IVB;
	temp &= ~FDI_LINK_TRAIN_VOL_EMP_MASK;
	temp |= FDI_LINK_TRAIN_400MV_0DB_SNB_B;
	temp |= FDI_COMPOSITE_SYNC;
	I915_WRITE(reg, temp | FDI_TX_ENABLE);

	I915_WRITE(FDI_RX_MISC(pipe),
		   FDI_RX_TP1_TO_TP2_48 | FDI_RX_FDI_DELAY_90);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_LINK_TRAIN_AUTO;
	temp &= ~FDI_LINK_TRAIN_PATTERN_MASK_CPT;
	temp |= FDI_LINK_TRAIN_PATTERN_1_CPT;
	temp |= FDI_COMPOSITE_SYNC;
	I915_WRITE(reg, temp | FDI_RX_ENABLE);

	POSTING_READ(reg);
	udelay(150);

	for (i = 0; i < 4; i++) {
		reg = FDI_TX_CTL(pipe);
		temp = I915_READ(reg);
		temp &= ~FDI_LINK_TRAIN_VOL_EMP_MASK;
		temp |= snb_b_fdi_train_param[i];
		I915_WRITE(reg, temp);

		POSTING_READ(reg);
		udelay(500);

		reg = FDI_RX_IIR(pipe);
		temp = I915_READ(reg);
		DRM_DEBUG_KMS("FDI_RX_IIR 0x%x\n", temp);

		if (temp & FDI_RX_BIT_LOCK ||
		    (I915_READ(reg) & FDI_RX_BIT_LOCK)) {
			I915_WRITE(reg, temp | FDI_RX_BIT_LOCK);
			DRM_DEBUG_KMS("FDI train 1 done, level %i.\n", i);
			break;
		}
	}
	if (i == 4)
		DRM_ERROR("FDI train 1 fail!\n");

	/* Train 2 */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_LINK_TRAIN_NONE_IVB;
	temp |= FDI_LINK_TRAIN_PATTERN_2_IVB;
	temp &= ~FDI_LINK_TRAIN_VOL_EMP_MASK;
	temp |= FDI_LINK_TRAIN_400MV_0DB_SNB_B;
	I915_WRITE(reg, temp);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_LINK_TRAIN_PATTERN_MASK_CPT;
	temp |= FDI_LINK_TRAIN_PATTERN_2_CPT;
	I915_WRITE(reg, temp);

	POSTING_READ(reg);
	udelay(150);

	for (i = 0; i < 4; i++) {
		reg = FDI_TX_CTL(pipe);
		temp = I915_READ(reg);
		temp &= ~FDI_LINK_TRAIN_VOL_EMP_MASK;
		temp |= snb_b_fdi_train_param[i];
		I915_WRITE(reg, temp);

		POSTING_READ(reg);
		udelay(500);

		reg = FDI_RX_IIR(pipe);
		temp = I915_READ(reg);
		DRM_DEBUG_KMS("FDI_RX_IIR 0x%x\n", temp);

		if (temp & FDI_RX_SYMBOL_LOCK) {
			I915_WRITE(reg, temp | FDI_RX_SYMBOL_LOCK);
			DRM_DEBUG_KMS("FDI train 2 done, level %i.\n", i);
			break;
		}
	}
	if (i == 4)
		DRM_ERROR("FDI train 2 fail!\n");

	DRM_DEBUG_KMS("FDI train done.\n");
}

static void ironlake_fdi_pll_enable(struct intel_crtc *intel_crtc)
{
	struct drm_device *dev = intel_crtc->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe = intel_crtc->pipe;
	u32 reg, temp;


	/* enable PCH FDI RX PLL, wait warmup plus DMI latency */
	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~((0x7 << 19) | (0x7 << 16));
	temp |= (intel_crtc->fdi_lanes - 1) << 19;
	temp |= (I915_READ(PIPECONF(pipe)) & PIPE_BPC_MASK) << 11;
	I915_WRITE(reg, temp | FDI_RX_PLL_ENABLE);

	POSTING_READ(reg);
	udelay(200);

	/* Switch from Rawclk to PCDclk */
	temp = I915_READ(reg);
	I915_WRITE(reg, temp | FDI_PCDCLK);

	POSTING_READ(reg);
	udelay(200);

	/* On Haswell, the PLL configuration for ports and pipes is handled
	 * separately, as part of DDI setup */
	if (!IS_HASWELL(dev)) {
		/* Enable CPU FDI TX PLL, always on for Ironlake */
		reg = FDI_TX_CTL(pipe);
		temp = I915_READ(reg);
		if ((temp & FDI_TX_PLL_ENABLE) == 0) {
			I915_WRITE(reg, temp | FDI_TX_PLL_ENABLE);

			POSTING_READ(reg);
			udelay(100);
		}
	}
}

static void ironlake_fdi_disable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	u32 reg, temp;

	/* disable CPU FDI tx and PCH FDI rx */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	I915_WRITE(reg, temp & ~FDI_TX_ENABLE);
	POSTING_READ(reg);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~(0x7 << 16);
	temp |= (I915_READ(PIPECONF(pipe)) & PIPE_BPC_MASK) << 11;
	I915_WRITE(reg, temp & ~FDI_RX_ENABLE);

	POSTING_READ(reg);
	udelay(100);

	/* Ironlake workaround, disable clock pointer after downing FDI */
	if (HAS_PCH_IBX(dev)) {
		I915_WRITE(FDI_RX_CHICKEN(pipe), FDI_RX_PHASE_SYNC_POINTER_OVR);
		I915_WRITE(FDI_RX_CHICKEN(pipe),
			   I915_READ(FDI_RX_CHICKEN(pipe) &
				     ~FDI_RX_PHASE_SYNC_POINTER_EN));
	}

	/* still set train pattern 1 */
	reg = FDI_TX_CTL(pipe);
	temp = I915_READ(reg);
	temp &= ~FDI_LINK_TRAIN_NONE;
	temp |= FDI_LINK_TRAIN_PATTERN_1;
	I915_WRITE(reg, temp);

	reg = FDI_RX_CTL(pipe);
	temp = I915_READ(reg);
	if (HAS_PCH_CPT(dev)) {
		temp &= ~FDI_LINK_TRAIN_PATTERN_MASK_CPT;
		temp |= FDI_LINK_TRAIN_PATTERN_1_CPT;
	} else {
		temp &= ~FDI_LINK_TRAIN_NONE;
		temp |= FDI_LINK_TRAIN_PATTERN_1;
	}
	/* BPC in FDI rx is consistent with that in PIPECONF */
	temp &= ~(0x07 << 16);
	temp |= (I915_READ(PIPECONF(pipe)) & PIPE_BPC_MASK) << 11;
	I915_WRITE(reg, temp);

	POSTING_READ(reg);
	udelay(100);
}

/*
 * When we disable a pipe, we need to clear any pending scanline wait events
 * to avoid hanging the ring, which we assume we are waiting on.
 */
static void intel_clear_scanline_wait(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	u32 tmp;

	if (IS_GEN2(dev))
		/* Can't break the hang on i8xx */
		return;

	ring = LP_RING(dev_priv);
	tmp = I915_READ_CTL(ring);
	if (tmp & RING_WAIT)
		I915_WRITE_CTL(ring, tmp);
}

static bool intel_crtc_has_pending_flip(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;
	bool pending;

	if (atomic_read(&dev_priv->mm.wedged))
		return false;

	spin_lock_irqsave(&dev->event_lock, flags);
	pending = to_intel_crtc(crtc)->unpin_work != NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	return pending;
}

static void intel_crtc_wait_for_pending_flips(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (crtc->fb == NULL)
		return;

	wait_event(dev_priv->pending_flip_queue,
		   !intel_crtc_has_pending_flip(crtc));

	mutex_lock(&dev->struct_mutex);
	intel_finish_fb(crtc->fb);
	mutex_unlock(&dev->struct_mutex);
}

/* Program iCLKIP clock to the desired frequency */
static void lpt_program_iclkip(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 divsel, phaseinc, auxdiv, phasedir = 0;
	u32 temp;

	/* It is necessary to ungate the pixclk gate prior to programming
	 * the divisors, and gate it back when it is done.
	 */
	I915_WRITE(PIXCLK_GATE, PIXCLK_GATE_GATE);

	/* Disable SSCCTL */
	intel_sbi_write(dev_priv, SBI_SSCCTL6,
			intel_sbi_read(dev_priv, SBI_SSCCTL6, SBI_ICLK) |
				SBI_SSCCTL_DISABLE,
			SBI_ICLK);

	/* 20MHz is a corner case which is out of range for the 7-bit divisor */
	if (crtc->mode.clock == 20000) {
		auxdiv = 1;
		divsel = 0x41;
		phaseinc = 0x20;
	} else {
		/* The iCLK virtual clock root frequency is in MHz,
		 * but the crtc->mode.clock in in KHz. To get the divisors,
		 * it is necessary to divide one by another, so we
		 * convert the virtual clock precision to KHz here for higher
		 * precision.
		 */
		u32 iclk_virtual_root_freq = 172800 * 1000;
		u32 iclk_pi_range = 64;
		u32 desired_divisor, msb_divisor_value, pi_value;

		desired_divisor = (iclk_virtual_root_freq / crtc->mode.clock);
		msb_divisor_value = desired_divisor / iclk_pi_range;
		pi_value = desired_divisor % iclk_pi_range;

		auxdiv = 0;
		divsel = msb_divisor_value - 2;
		phaseinc = pi_value;
	}

	/* This should not happen with any sane values */
	WARN_ON(SBI_SSCDIVINTPHASE_DIVSEL(divsel) &
		~SBI_SSCDIVINTPHASE_DIVSEL_MASK);
	WARN_ON(SBI_SSCDIVINTPHASE_DIR(phasedir) &
		~SBI_SSCDIVINTPHASE_INCVAL_MASK);

	DRM_DEBUG_KMS("iCLKIP clock: found settings for %dKHz refresh rate: auxdiv=%x, divsel=%x, phasedir=%x, phaseinc=%x\n",
			crtc->mode.clock,
			auxdiv,
			divsel,
			phasedir,
			phaseinc);

	/* Program SSCDIVINTPHASE6 */
	temp = intel_sbi_read(dev_priv, SBI_SSCDIVINTPHASE6, SBI_ICLK);
	temp &= ~SBI_SSCDIVINTPHASE_DIVSEL_MASK;
	temp |= SBI_SSCDIVINTPHASE_DIVSEL(divsel);
	temp &= ~SBI_SSCDIVINTPHASE_INCVAL_MASK;
	temp |= SBI_SSCDIVINTPHASE_INCVAL(phaseinc);
	temp |= SBI_SSCDIVINTPHASE_DIR(phasedir);
	temp |= SBI_SSCDIVINTPHASE_PROPAGATE;
	intel_sbi_write(dev_priv, SBI_SSCDIVINTPHASE6, temp, SBI_ICLK);

	/* Program SSCAUXDIV */
	temp = intel_sbi_read(dev_priv, SBI_SSCAUXDIV6, SBI_ICLK);
	temp &= ~SBI_SSCAUXDIV_FINALDIV2SEL(1);
	temp |= SBI_SSCAUXDIV_FINALDIV2SEL(auxdiv);
	intel_sbi_write(dev_priv, SBI_SSCAUXDIV6, temp, SBI_ICLK);

	/* Enable modulator and associated divider */
	temp = intel_sbi_read(dev_priv, SBI_SSCCTL6, SBI_ICLK);
	temp &= ~SBI_SSCCTL_DISABLE;
	intel_sbi_write(dev_priv, SBI_SSCCTL6, temp, SBI_ICLK);

	/* Wait for initialization time */
	udelay(24);

	I915_WRITE(PIXCLK_GATE, PIXCLK_GATE_UNGATE);
}

/*
 * Enable PCH resources required for PCH ports:
 *   - PCH PLLs
 *   - FDI training & RX/TX
 *   - update transcoder timings
 *   - DP transcoding bits
 *   - transcoder
 */
static void ironlake_pch_enable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	u32 reg, temp;

	assert_transcoder_disabled(dev_priv, pipe);

	/* Write the TU size bits before fdi link training, so that error
	 * detection works. */
	I915_WRITE(FDI_RX_TUSIZE1(pipe),
		   I915_READ(PIPE_DATA_M1(pipe)) & TU_SIZE_MASK);

	/* For PCH output, training FDI link */
	dev_priv->display.fdi_link_train(crtc);

	/* XXX: pch pll's can be enabled any time before we enable the PCH
	 * transcoder, and we actually should do this to not upset any PCH
	 * transcoder that already use the clock when we share it.
	 *
	 * Note that enable_pch_pll tries to do the right thing, but get_pch_pll
	 * unconditionally resets the pll - we need that to have the right LVDS
	 * enable sequence. */
	ironlake_enable_pch_pll(intel_crtc);

	if (HAS_PCH_CPT(dev)) {
		u32 sel;

		temp = I915_READ(PCH_DPLL_SEL);
		switch (pipe) {
		default:
		case 0:
			temp |= TRANSA_DPLL_ENABLE;
			sel = TRANSA_DPLLB_SEL;
			break;
		case 1:
			temp |= TRANSB_DPLL_ENABLE;
			sel = TRANSB_DPLLB_SEL;
			break;
		case 2:
			temp |= TRANSC_DPLL_ENABLE;
			sel = TRANSC_DPLLB_SEL;
			break;
		}
		if (intel_crtc->pch_pll->pll_reg == _PCH_DPLL_B)
			temp |= sel;
		else
			temp &= ~sel;
		I915_WRITE(PCH_DPLL_SEL, temp);
	}

	/* set transcoder timing, panel must allow it */
	assert_panel_unlocked(dev_priv, pipe);
	I915_WRITE(TRANS_HTOTAL(pipe), I915_READ(HTOTAL(pipe)));
	I915_WRITE(TRANS_HBLANK(pipe), I915_READ(HBLANK(pipe)));
	I915_WRITE(TRANS_HSYNC(pipe),  I915_READ(HSYNC(pipe)));

	I915_WRITE(TRANS_VTOTAL(pipe), I915_READ(VTOTAL(pipe)));
	I915_WRITE(TRANS_VBLANK(pipe), I915_READ(VBLANK(pipe)));
	I915_WRITE(TRANS_VSYNC(pipe),  I915_READ(VSYNC(pipe)));
	I915_WRITE(TRANS_VSYNCSHIFT(pipe),  I915_READ(VSYNCSHIFT(pipe)));

	intel_fdi_normal_train(crtc);

	/* For PCH DP, enable TRANS_DP_CTL */
	if (HAS_PCH_CPT(dev) &&
	    (intel_pipe_has_type(crtc, INTEL_OUTPUT_DISPLAYPORT) ||
	     intel_pipe_has_type(crtc, INTEL_OUTPUT_EDP))) {
		u32 bpc = (I915_READ(PIPECONF(pipe)) & PIPE_BPC_MASK) >> 5;
		reg = TRANS_DP_CTL(pipe);
		temp = I915_READ(reg);
		temp &= ~(TRANS_DP_PORT_SEL_MASK |
			  TRANS_DP_SYNC_MASK |
			  TRANS_DP_BPC_MASK);
		temp |= (TRANS_DP_OUTPUT_ENABLE |
			 TRANS_DP_ENH_FRAMING);
		temp |= bpc << 9; /* same format but at 11:9 */

		if (crtc->mode.flags & DRM_MODE_FLAG_PHSYNC)
			temp |= TRANS_DP_HSYNC_ACTIVE_HIGH;
		if (crtc->mode.flags & DRM_MODE_FLAG_PVSYNC)
			temp |= TRANS_DP_VSYNC_ACTIVE_HIGH;

		switch (intel_trans_dp_port_sel(crtc)) {
		case PCH_DP_B:
			temp |= TRANS_DP_PORT_SEL_B;
			break;
		case PCH_DP_C:
			temp |= TRANS_DP_PORT_SEL_C;
			break;
		case PCH_DP_D:
			temp |= TRANS_DP_PORT_SEL_D;
			break;
		default:
			BUG();
		}

		I915_WRITE(reg, temp);
	}

	ironlake_enable_pch_transcoder(dev_priv, pipe);
}

static void lpt_pch_enable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	enum transcoder cpu_transcoder = intel_crtc->config.cpu_transcoder;

	assert_transcoder_disabled(dev_priv, TRANSCODER_A);

	lpt_program_iclkip(crtc);

	/* Set transcoder timing. */
	I915_WRITE(_TRANS_HTOTAL_A, I915_READ(HTOTAL(cpu_transcoder)));
	I915_WRITE(_TRANS_HBLANK_A, I915_READ(HBLANK(cpu_transcoder)));
	I915_WRITE(_TRANS_HSYNC_A,  I915_READ(HSYNC(cpu_transcoder)));

	I915_WRITE(_TRANS_VTOTAL_A, I915_READ(VTOTAL(cpu_transcoder)));
	I915_WRITE(_TRANS_VBLANK_A, I915_READ(VBLANK(cpu_transcoder)));
	I915_WRITE(_TRANS_VSYNC_A,  I915_READ(VSYNC(cpu_transcoder)));
	I915_WRITE(_TRANS_VSYNCSHIFT_A, I915_READ(VSYNCSHIFT(cpu_transcoder)));

	lpt_enable_pch_transcoder(dev_priv, cpu_transcoder);
}

static void intel_put_pch_pll(struct intel_crtc *intel_crtc)
{
	struct intel_pch_pll *pll = intel_crtc->pch_pll;

	if (pll == NULL)
		return;

	if (pll->refcount == 0) {
		WARN(1, "bad PCH PLL refcount\n");
		return;
	}

	--pll->refcount;
	intel_crtc->pch_pll = NULL;
}

static struct intel_pch_pll *intel_get_pch_pll(struct intel_crtc *intel_crtc, u32 dpll, u32 fp)
{
	struct drm_i915_private *dev_priv = intel_crtc->base.dev->dev_private;
	struct intel_pch_pll *pll;
	int i;

	pll = intel_crtc->pch_pll;
	if (pll) {
		DRM_DEBUG_KMS("CRTC:%d reusing existing PCH PLL %x\n",
			      intel_crtc->base.base.id, pll->pll_reg);
		goto prepare;
	}

	if (HAS_PCH_IBX(dev_priv->dev)) {
		/* Ironlake PCH has a fixed PLL->PCH pipe mapping. */
		i = intel_crtc->pipe;
		pll = &dev_priv->pch_plls[i];

		DRM_DEBUG_KMS("CRTC:%d using pre-allocated PCH PLL %x\n",
			      intel_crtc->base.base.id, pll->pll_reg);

		goto found;
	}

	for (i = 0; i < dev_priv->num_pch_pll; i++) {
		pll = &dev_priv->pch_plls[i];

		/* Only want to check enabled timings first */
		if (pll->refcount == 0)
			continue;

		if (dpll == (I915_READ(pll->pll_reg) & 0x7fffffff) &&
		    fp == I915_READ(pll->fp0_reg)) {
			DRM_DEBUG_KMS("CRTC:%d sharing existing PCH PLL %x (refcount %d, ative %d)\n",
				      intel_crtc->base.base.id,
				      pll->pll_reg, pll->refcount, pll->active);

			goto found;
		}
	}

	/* Ok no matching timings, maybe there's a free one? */
	for (i = 0; i < dev_priv->num_pch_pll; i++) {
		pll = &dev_priv->pch_plls[i];
		if (pll->refcount == 0) {
			DRM_DEBUG_KMS("CRTC:%d allocated PCH PLL %x\n",
				      intel_crtc->base.base.id, pll->pll_reg);
			goto found;
		}
	}

	return NULL;

found:
	intel_crtc->pch_pll = pll;
	pll->refcount++;
	DRM_DEBUG_DRIVER("using pll %d for pipe %d\n", i, intel_crtc->pipe);
prepare: /* separate function? */
	DRM_DEBUG_DRIVER("switching PLL %x off\n", pll->pll_reg);

	/* Wait for the clocks to stabilize before rewriting the regs */
	I915_WRITE(pll->pll_reg, dpll & ~DPLL_VCO_ENABLE);
	POSTING_READ(pll->pll_reg);
	udelay(150);

	I915_WRITE(pll->fp0_reg, fp);
	I915_WRITE(pll->pll_reg, dpll & ~DPLL_VCO_ENABLE);
	pll->on = false;
	return pll;
}

void intel_cpt_verify_modeset(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int dslreg = PIPEDSL(pipe);
	u32 temp;

	temp = I915_READ(dslreg);
	udelay(500);
	if (wait_for(I915_READ(dslreg) != temp, 5)) {
		if (wait_for(I915_READ(dslreg) != temp, 5))
			DRM_ERROR("mode set failed: pipe %d stuck\n", pipe);
	}
}

static void ironlake_crtc_enable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_encoder *encoder;
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	u32 temp;

	WARN_ON(!crtc->enabled);

	if (intel_crtc->active)
		return;

	intel_crtc->active = true;
	intel_update_watermarks(dev);

	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS)) {
		temp = I915_READ(PCH_LVDS);
		if ((temp & LVDS_PORT_EN) == 0)
			I915_WRITE(PCH_LVDS, temp | LVDS_PORT_EN);
	}


	if (intel_crtc->config.has_pch_encoder) {
		/* Note: FDI PLL enabling _must_ be done before we enable the
		 * cpu pipes, hence this is separate from all the other fdi/pch
		 * enabling. */
		ironlake_fdi_pll_enable(intel_crtc);
	} else {
		assert_fdi_tx_disabled(dev_priv, pipe);
		assert_fdi_rx_disabled(dev_priv, pipe);
	}

	for_each_encoder_on_crtc(dev, crtc, encoder)
		if (encoder->pre_enable)
			encoder->pre_enable(encoder);

	/* Enable panel fitting for LVDS */
	if (dev_priv->pch_pf_size &&
	    (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS) || HAS_eDP)) {
		/* Force use of hard-coded filter coefficients
		 * as some pre-programmed values are broken,
		 * e.g. x201.
		 */
		if (IS_IVYBRIDGE(dev))
			I915_WRITE(PF_CTL(pipe), PF_ENABLE | PF_FILTER_MED_3x3 |
						 PF_PIPE_SEL_IVB(pipe));
		else
			I915_WRITE(PF_CTL(pipe), PF_ENABLE | PF_FILTER_MED_3x3);
		I915_WRITE(PF_WIN_POS(pipe), dev_priv->pch_pf_pos);
		I915_WRITE(PF_WIN_SZ(pipe), dev_priv->pch_pf_size);
	}

	/*
	 * On ILK+ LUT must be loaded before the pipe is running but with
	 * clocks enabled
	 */
	intel_crtc_load_lut(crtc);

	intel_enable_pipe(dev_priv, pipe,
			  intel_crtc->config.has_pch_encoder);
	intel_enable_plane(dev_priv, plane, pipe);

	if (intel_crtc->config.has_pch_encoder)
		ironlake_pch_enable(crtc);

	mutex_lock(&dev->struct_mutex);
	intel_update_fbc(dev);
	mutex_unlock(&dev->struct_mutex);

	intel_crtc_update_cursor(crtc, true);

	for_each_encoder_on_crtc(dev, crtc, encoder)
		encoder->enable(encoder);

	if (HAS_PCH_CPT(dev))
		intel_cpt_verify_modeset(dev, intel_crtc->pipe);

	/*
	 * There seems to be a race in PCH platform hw (at least on some
	 * outputs) where an enabled pipe still completes any pageflip right
	 * away (as if the pipe is off) instead of waiting for vblank. As soon
	 * as the first vblank happend, everything works as expected. Hence just
	 * wait for one vblank before returning to avoid strange things
	 * happening.
	 */
	intel_wait_for_vblank(dev, intel_crtc->pipe);
}

static void haswell_crtc_enable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_encoder *encoder;
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;

	WARN_ON(!crtc->enabled);

	if (intel_crtc->active)
		return;

	intel_crtc->active = true;
	intel_update_watermarks(dev);

	if (intel_crtc->config.has_pch_encoder)
		dev_priv->display.fdi_link_train(crtc);

	for_each_encoder_on_crtc(dev, crtc, encoder)
		if (encoder->pre_enable)
			encoder->pre_enable(encoder);

	intel_ddi_enable_pipe_clock(intel_crtc);

	/* Enable panel fitting for eDP */
	if (dev_priv->pch_pf_size &&
	    intel_pipe_has_type(crtc, INTEL_OUTPUT_EDP)) {
		/* Force use of hard-coded filter coefficients
		 * as some pre-programmed values are broken,
		 * e.g. x201.
		 */
		I915_WRITE(PF_CTL(pipe), PF_ENABLE | PF_FILTER_MED_3x3 |
					 PF_PIPE_SEL_IVB(pipe));
		I915_WRITE(PF_WIN_POS(pipe), dev_priv->pch_pf_pos);
		I915_WRITE(PF_WIN_SZ(pipe), dev_priv->pch_pf_size);
	}

	/*
	 * On ILK+ LUT must be loaded before the pipe is running but with
	 * clocks enabled
	 */
	intel_crtc_load_lut(crtc);

	intel_ddi_set_pipe_settings(crtc);
	intel_ddi_enable_transcoder_func(crtc);

	intel_enable_pipe(dev_priv, pipe,
			  intel_crtc->config.has_pch_encoder);
	intel_enable_plane(dev_priv, plane, pipe);

	if (intel_crtc->config.has_pch_encoder)
		lpt_pch_enable(crtc);

	mutex_lock(&dev->struct_mutex);
	intel_update_fbc(dev);
	mutex_unlock(&dev->struct_mutex);

	intel_crtc_update_cursor(crtc, true);

	for_each_encoder_on_crtc(dev, crtc, encoder)
		encoder->enable(encoder);

	/*
	 * There seems to be a race in PCH platform hw (at least on some
	 * outputs) where an enabled pipe still completes any pageflip right
	 * away (as if the pipe is off) instead of waiting for vblank. As soon
	 * as the first vblank happend, everything works as expected. Hence just
	 * wait for one vblank before returning to avoid strange things
	 * happening.
	 */
	intel_wait_for_vblank(dev, intel_crtc->pipe);
}

static void ironlake_crtc_disable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_encoder *encoder;
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	u32 reg, temp;


	if (!intel_crtc->active)
		return;

	for_each_encoder_on_crtc(dev, crtc, encoder)
		encoder->disable(encoder);

	intel_crtc_wait_for_pending_flips(crtc);
	drm_vblank_off(dev, pipe);
	intel_crtc_update_cursor(crtc, false);

	intel_disable_plane(dev_priv, plane, pipe);

	if (dev_priv->cfb_plane == plane)
		intel_disable_fbc(dev);

	intel_disable_pipe(dev_priv, pipe);

	/* Disable PF */
	I915_WRITE(PF_CTL(pipe), 0);
	I915_WRITE(PF_WIN_SZ(pipe), 0);

	for_each_encoder_on_crtc(dev, crtc, encoder)
		if (encoder->post_disable)
			encoder->post_disable(encoder);

	ironlake_fdi_disable(crtc);

	ironlake_disable_pch_transcoder(dev_priv, pipe);

	if (HAS_PCH_CPT(dev)) {
		/* disable TRANS_DP_CTL */
		reg = TRANS_DP_CTL(pipe);
		temp = I915_READ(reg);
		temp &= ~(TRANS_DP_OUTPUT_ENABLE | TRANS_DP_PORT_SEL_MASK);
		temp |= TRANS_DP_PORT_SEL_NONE;
		I915_WRITE(reg, temp);

		/* disable DPLL_SEL */
		temp = I915_READ(PCH_DPLL_SEL);
		switch (pipe) {
		case 0:
			temp &= ~(TRANSA_DPLL_ENABLE | TRANSA_DPLLB_SEL);
			break;
		case 1:
			temp &= ~(TRANSB_DPLL_ENABLE | TRANSB_DPLLB_SEL);
			break;
		case 2:
			/* C shares PLL A or B */
			temp &= ~(TRANSC_DPLL_ENABLE | TRANSC_DPLLB_SEL);
			break;
		default:
			BUG(); /* wtf */
		}
		I915_WRITE(PCH_DPLL_SEL, temp);
	}

	/* disable PCH DPLL */
	intel_disable_pch_pll(intel_crtc);

	ironlake_fdi_pll_disable(intel_crtc);

	intel_crtc->active = false;
	intel_update_watermarks(dev);

	mutex_lock(&dev->struct_mutex);
	intel_update_fbc(dev);
	mutex_unlock(&dev->struct_mutex);
}

static void haswell_crtc_disable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_encoder *encoder;
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	enum transcoder cpu_transcoder = intel_crtc->config.cpu_transcoder;

	if (!intel_crtc->active)
		return;

	for_each_encoder_on_crtc(dev, crtc, encoder)
		encoder->disable(encoder);

	intel_crtc_wait_for_pending_flips(crtc);
	drm_vblank_off(dev, pipe);
	intel_crtc_update_cursor(crtc, false);

	intel_disable_plane(dev_priv, plane, pipe);

	if (dev_priv->cfb_plane == plane)
		intel_disable_fbc(dev);

	intel_disable_pipe(dev_priv, pipe);

	intel_ddi_disable_transcoder_func(dev_priv, cpu_transcoder);

	/* XXX: Once we have proper panel fitter state tracking implemented with
	 * hardware state read/check support we should switch to only disable
	 * the panel fitter when we know it's used. */
	if (intel_using_power_well(dev)) {
		I915_WRITE(PF_CTL(pipe), 0);
		I915_WRITE(PF_WIN_SZ(pipe), 0);
	}

	intel_ddi_disable_pipe_clock(intel_crtc);

	for_each_encoder_on_crtc(dev, crtc, encoder)
		if (encoder->post_disable)
			encoder->post_disable(encoder);

	if (intel_crtc->config.has_pch_encoder) {
		lpt_disable_pch_transcoder(dev_priv);
		intel_ddi_fdi_disable(crtc);
	}

	intel_crtc->active = false;
	intel_update_watermarks(dev);

	mutex_lock(&dev->struct_mutex);
	intel_update_fbc(dev);
	mutex_unlock(&dev->struct_mutex);
}

static void ironlake_crtc_off(struct drm_crtc *crtc)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	intel_put_pch_pll(intel_crtc);
}

static void haswell_crtc_off(struct drm_crtc *crtc)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	/* Stop saying we're using TRANSCODER_EDP because some other CRTC might
	 * start using it. */
	intel_crtc->cpu_transcoder = intel_crtc->pipe;

	intel_ddi_put_crtc_pll(crtc);
}

static void intel_crtc_dpms_overlay(struct intel_crtc *intel_crtc, bool enable)
{
	if (!enable && intel_crtc->overlay) {
		struct drm_device *dev = intel_crtc->base.dev;
		struct drm_i915_private *dev_priv = dev->dev_private;

		mutex_lock(&dev->struct_mutex);
		dev_priv->mm.interruptible = false;
		(void) intel_overlay_switch_off(intel_crtc->overlay);
		dev_priv->mm.interruptible = true;
		mutex_unlock(&dev->struct_mutex);
	}

	/* Let userspace switch the overlay on again. In most cases userspace
	 * has to recompute where to put it anyway.
	 */
}

static void i9xx_crtc_enable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_encoder *encoder;
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;

	WARN_ON(!crtc->enabled);

	if (intel_crtc->active)
		return;

	intel_crtc->active = true;
	intel_update_watermarks(dev);

	intel_enable_pll(dev_priv, pipe);
	intel_enable_pipe(dev_priv, pipe, false);
	intel_enable_plane(dev_priv, plane, pipe);

	intel_crtc_load_lut(crtc);
	intel_update_fbc(dev);

	/* Give the overlay scaler a chance to enable if it's on this pipe */
	intel_crtc_dpms_overlay(intel_crtc, true);
	intel_crtc_update_cursor(crtc, true);

	for_each_encoder_on_crtc(dev, crtc, encoder)
		encoder->enable(encoder);
}

static void i9xx_pfit_disable(struct intel_crtc *crtc)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum pipe pipe;
	uint32_t pctl = I915_READ(PFIT_CONTROL);

	assert_pipe_disabled(dev_priv, crtc->pipe);

	if (INTEL_INFO(dev)->gen >= 4)
		pipe = (pctl & PFIT_PIPE_MASK) >> PFIT_PIPE_SHIFT;
	else
		pipe = PIPE_B;

	if (pipe == crtc->pipe) {
		DRM_DEBUG_DRIVER("disabling pfit, current: 0x%08x\n", pctl);
		I915_WRITE(PFIT_CONTROL, 0);
	}
}

static void i9xx_crtc_disable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	u32 pctl;

	if (!intel_crtc->active)
		return;

	/* Give the overlay scaler a chance to disable if it's on this pipe */
	intel_crtc_wait_for_pending_flips(crtc);
	drm_vblank_off(dev, pipe);
	intel_crtc_dpms_overlay(intel_crtc, false);
	intel_crtc_update_cursor(crtc, false);

	if (dev_priv->cfb_plane == plane)
		intel_disable_fbc(dev);

	intel_disable_plane(dev_priv, plane, pipe);
	intel_disable_pipe(dev_priv, pipe);

	i9xx_pfit_disable(intel_crtc);

	intel_disable_pll(dev_priv, pipe);

	intel_crtc->active = false;
	intel_update_fbc(dev);
	intel_update_watermarks(dev);
}

static void i9xx_crtc_off(struct drm_crtc *crtc)
{
}

static void intel_crtc_update_sarea(struct drm_crtc *crtc,
				    bool enabled)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_master_private *master_priv;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;

	if (!dev->primary->master)
		return;

	master_priv = dev->primary->master->driver_priv;
	if (!master_priv->sarea_priv)
		return;

	switch (pipe) {
	case 0:
		master_priv->sarea_priv->pipeA_w = enabled ? crtc->mode.hdisplay : 0;
		master_priv->sarea_priv->pipeA_h = enabled ? crtc->mode.vdisplay : 0;
		break;
	case 1:
		master_priv->sarea_priv->pipeB_w = enabled ? crtc->mode.hdisplay : 0;
		master_priv->sarea_priv->pipeB_h = enabled ? crtc->mode.vdisplay : 0;
		break;
	default:
		DRM_ERROR("Can't update pipe %c in SAREA\n", pipe_name(pipe));
		break;
	}
}

static void intel_crtc_disable(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	struct drm_device *dev = crtc->dev;

	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_OFF);
	assert_plane_disabled(dev->dev_private, to_intel_crtc(crtc)->plane);
	assert_pipe_disabled(dev->dev_private, to_intel_crtc(crtc)->pipe);

	if (crtc->fb) {
		mutex_lock(&dev->struct_mutex);
		intel_unpin_fb_obj(to_intel_framebuffer(crtc->fb)->obj);
		mutex_unlock(&dev->struct_mutex);
		crtc->fb = NULL;
	}

	/* Update computed state. */
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (!connector->encoder || !connector->encoder->crtc)
			continue;

		if (connector->encoder->crtc != crtc)
			continue;

		connector->dpms = DRM_MODE_DPMS_OFF;
		to_intel_encoder(connector->encoder)->connectors_active = false;
	}
}

void intel_modeset_disable(struct drm_device *dev)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (crtc->enabled)
			intel_crtc_disable(crtc);
	}
}

void intel_encoder_noop(struct drm_encoder *encoder)
{
}

void intel_encoder_destroy(struct drm_encoder *encoder)
{
	struct intel_encoder *intel_encoder = to_intel_encoder(encoder);

	drm_encoder_cleanup(encoder);
	kfree(intel_encoder);
}

/* Simple dpms helper for encodres with just one connector, no cloning and only
 * one kind of off state. It clamps all !ON modes to fully OFF and changes the
 * state of the entire output pipe. */
void intel_encoder_dpms(struct intel_encoder *encoder, int mode)
{
	if (mode == DRM_MODE_DPMS_ON) {
		encoder->connectors_active = true;

		intel_crtc_update_dpms(encoder->base.crtc);
	} else {
		encoder->connectors_active = false;

		intel_crtc_update_dpms(encoder->base.crtc);
	}
}

/* Cross check the actual hw state with our own modeset state tracking (and it's
 * internal consistency). */
static void intel_connector_check_state(struct intel_connector *connector)
{
	if (connector->get_hw_state(connector)) {
		struct intel_encoder *encoder = connector->encoder;
		struct drm_crtc *crtc;
		bool encoder_enabled;
		enum pipe pipe;

		DRM_DEBUG_KMS("[CONNECTOR:%d:%s]\n",
			      connector->base.base.id,
			      drm_get_connector_name(&connector->base));

		WARN(connector->base.dpms == DRM_MODE_DPMS_OFF,
		     "wrong connector dpms state\n");
		WARN(connector->base.encoder != &encoder->base,
		     "active connector not linked to encoder\n");
		WARN(!encoder->connectors_active,
		     "encoder->connectors_active not set\n");

		encoder_enabled = encoder->get_hw_state(encoder, &pipe);
		WARN(!encoder_enabled, "encoder not enabled\n");
		if (WARN_ON(!encoder->base.crtc))
			return;

		crtc = encoder->base.crtc;

		WARN(!crtc->enabled, "crtc not enabled\n");
		WARN(!to_intel_crtc(crtc)->active, "crtc not active\n");
		WARN(pipe != to_intel_crtc(crtc)->pipe,
		     "encoder active on the wrong pipe\n");
	}
}

/* Even simpler default implementation, if there's really no special case to
 * consider. */
void intel_connector_dpms(struct drm_connector *connector, int mode)
{
	struct intel_encoder *encoder = intel_attached_encoder(connector);

	/* All the simple cases only support two dpms states. */
	if (mode != DRM_MODE_DPMS_ON)
		mode = DRM_MODE_DPMS_OFF;

	if (mode == connector->dpms)
		return;

	connector->dpms = mode;

	/* Only need to change hw state when actually enabled */
	if (encoder->base.crtc)
		intel_encoder_dpms(encoder, mode);
	else
		WARN_ON(encoder->connectors_active != false);

	intel_modeset_check_state(connector->dev);
}

/* Simple connector->get_hw_state implementation for encoders that support only
 * one connector and no cloning and hence the encoder state determines the state
 * of the connector. */
bool intel_connector_get_hw_state(struct intel_connector *connector)
{
	enum pipe pipe = 0;
	struct intel_encoder *encoder = connector->encoder;

	return encoder->get_hw_state(encoder, &pipe);
}

static bool intel_crtc_compute_config(struct drm_crtc *crtc,
				      struct intel_crtc_config *pipe_config)
{
	struct drm_device *dev = crtc->dev;
	struct drm_display_mode *adjusted_mode = &pipe_config->adjusted_mode;

	if (HAS_PCH_SPLIT(dev)) {
		/* FDI link clock is fixed at 2.7G */
		if (pipe_config->requested_mode.clock * 3
		    > IRONLAKE_FDI_FREQ * 4)
			return false;
	}

	/* All interlaced capable intel hw wants timings in frames. Note though
	 * that intel_lvds_mode_fixup does some funny tricks with the crtc
	 * timings, so we need to be careful not to clobber these.*/
	if (!pipe_config->timings_set)
		drm_mode_set_crtcinfo(adjusted_mode, 0);

	/* WaPruneModeWithIncorrectHsyncOffset: Cantiga+ cannot handle modes
	 * with a hsync front porch of 0.
	 */
	if ((INTEL_INFO(dev)->gen > 4 || IS_G4X(dev)) &&
		adjusted_mode->hsync_start == adjusted_mode->hdisplay)
		return false;

	if ((IS_G4X(dev) || IS_VALLEYVIEW(dev)) && pipe_config->pipe_bpp > 10*3) {
		pipe_config->pipe_bpp = 10*3; /* 12bpc is gen5+ */
	} else if (INTEL_INFO(dev)->gen <= 4 && pipe_config->pipe_bpp > 8*3) {
		/* only a 8bpc pipe, with 6bpc dither through the panel fitter
		 * for lvds. */
		pipe_config->pipe_bpp = 8*3;
	}

	return true;
}

static int valleyview_get_display_clock_speed(struct drm_device *dev)
{
	return 400000; /* FIXME */
}

static int i945_get_display_clock_speed(struct drm_device *dev)
{
	return 400000;
}

static int i915_get_display_clock_speed(struct drm_device *dev)
{
	return 333000;
}

static int i9xx_misc_get_display_clock_speed(struct drm_device *dev)
{
	return 200000;
}

static int i915gm_get_display_clock_speed(struct drm_device *dev)
{
	u16 gcfgc = 0;

	pci_read_config_word(dev->pdev, GCFGC, &gcfgc);

	if (gcfgc & GC_LOW_FREQUENCY_ENABLE)
		return 133000;
	else {
		switch (gcfgc & GC_DISPLAY_CLOCK_MASK) {
		case GC_DISPLAY_CLOCK_333_MHZ:
			return 333000;
		default:
		case GC_DISPLAY_CLOCK_190_200_MHZ:
			return 190000;
		}
	}
}

static int i865_get_display_clock_speed(struct drm_device *dev)
{
	return 266000;
}

static int i855_get_display_clock_speed(struct drm_device *dev)
{
	u16 hpllcc = 0;
	/* Assume that the hardware is in the high speed state.  This
	 * should be the default.
	 */
	switch (hpllcc & GC_CLOCK_CONTROL_MASK) {
	case GC_CLOCK_133_200:
	case GC_CLOCK_100_200:
		return 200000;
	case GC_CLOCK_166_250:
		return 250000;
	case GC_CLOCK_100_133:
		return 133000;
	}

	/* Shouldn't happen */
	return 0;
}

static int i830_get_display_clock_speed(struct drm_device *dev)
{
	return 133000;
}

struct fdi_m_n {
	u32        tu;
	u32        gmch_m;
	u32        gmch_n;
	u32        link_m;
	u32        link_n;
};

static void
fdi_reduce_ratio(u32 *num, u32 *den)
{
	while (*num > 0xffffff || *den > 0xffffff) {
		*num >>= 1;
		*den >>= 1;
	}
}

static void
ironlake_compute_m_n(int bits_per_pixel, int nlanes, int pixel_clock,
		     int link_clock, struct fdi_m_n *m_n)
{
	m_n->tu = 64; /* default size */

	/* BUG_ON(pixel_clock > INT_MAX / 36); */
	m_n->gmch_m = bits_per_pixel * pixel_clock;
	m_n->gmch_n = link_clock * nlanes * 8;
	fdi_reduce_ratio(&m_n->gmch_m, &m_n->gmch_n);

	m_n->link_m = pixel_clock;
	m_n->link_n = link_clock;
	fdi_reduce_ratio(&m_n->link_m, &m_n->link_n);
}


struct intel_watermark_params {
	unsigned long fifo_size;
	unsigned long max_wm;
	unsigned long default_wm;
	unsigned long guard_size;
	unsigned long cacheline_size;
};

/* Pineview has different values for various configs */
static const struct intel_watermark_params pineview_display_wm = {
	PINEVIEW_DISPLAY_FIFO,
	PINEVIEW_MAX_WM,
	PINEVIEW_DFT_WM,
	PINEVIEW_GUARD_WM,
	PINEVIEW_FIFO_LINE_SIZE
};
static const struct intel_watermark_params pineview_display_hplloff_wm = {
	PINEVIEW_DISPLAY_FIFO,
	PINEVIEW_MAX_WM,
	PINEVIEW_DFT_HPLLOFF_WM,
	PINEVIEW_GUARD_WM,
	PINEVIEW_FIFO_LINE_SIZE
};
static const struct intel_watermark_params pineview_cursor_wm = {
	PINEVIEW_CURSOR_FIFO,
	PINEVIEW_CURSOR_MAX_WM,
	PINEVIEW_CURSOR_DFT_WM,
	PINEVIEW_CURSOR_GUARD_WM,
	PINEVIEW_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params pineview_cursor_hplloff_wm = {
	PINEVIEW_CURSOR_FIFO,
	PINEVIEW_CURSOR_MAX_WM,
	PINEVIEW_CURSOR_DFT_WM,
	PINEVIEW_CURSOR_GUARD_WM,
	PINEVIEW_FIFO_LINE_SIZE
};
static const struct intel_watermark_params g4x_wm_info = {
	G4X_FIFO_SIZE,
	G4X_MAX_WM,
	G4X_MAX_WM,
	2,
	G4X_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params g4x_cursor_wm_info = {
	I965_CURSOR_FIFO,
	I965_CURSOR_MAX_WM,
	I965_CURSOR_DFT_WM,
	2,
	G4X_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params i965_cursor_wm_info = {
	I965_CURSOR_FIFO,
	I965_CURSOR_MAX_WM,
	I965_CURSOR_DFT_WM,
	2,
	I915_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params i945_wm_info = {
	I945_FIFO_SIZE,
	I915_MAX_WM,
	1,
	2,
	I915_FIFO_LINE_SIZE
};
static const struct intel_watermark_params i915_wm_info = {
	I915_FIFO_SIZE,
	I915_MAX_WM,
	1,
	2,
	I915_FIFO_LINE_SIZE
};
static const struct intel_watermark_params i855_wm_info = {
	I855GM_FIFO_SIZE,
	I915_MAX_WM,
	1,
	2,
	I830_FIFO_LINE_SIZE
};
static const struct intel_watermark_params i830_wm_info = {
	I830_FIFO_SIZE,
	I915_MAX_WM,
	1,
	2,
	I830_FIFO_LINE_SIZE
};

static const struct intel_watermark_params ironlake_display_wm_info = {
	ILK_DISPLAY_FIFO,
	ILK_DISPLAY_MAXWM,
	ILK_DISPLAY_DFTWM,
	2,
	ILK_FIFO_LINE_SIZE
};
static const struct intel_watermark_params ironlake_cursor_wm_info = {
	ILK_CURSOR_FIFO,
	ILK_CURSOR_MAXWM,
	ILK_CURSOR_DFTWM,
	2,
	ILK_FIFO_LINE_SIZE
};
static const struct intel_watermark_params ironlake_display_srwm_info = {
	ILK_DISPLAY_SR_FIFO,
	ILK_DISPLAY_MAX_SRWM,
	ILK_DISPLAY_DFT_SRWM,
	2,
	ILK_FIFO_LINE_SIZE
};
static const struct intel_watermark_params ironlake_cursor_srwm_info = {
	ILK_CURSOR_SR_FIFO,
	ILK_CURSOR_MAX_SRWM,
	ILK_CURSOR_DFT_SRWM,
	2,
	ILK_FIFO_LINE_SIZE
};

static const struct intel_watermark_params sandybridge_display_wm_info = {
	SNB_DISPLAY_FIFO,
	SNB_DISPLAY_MAXWM,
	SNB_DISPLAY_DFTWM,
	2,
	SNB_FIFO_LINE_SIZE
};
static const struct intel_watermark_params sandybridge_cursor_wm_info = {
	SNB_CURSOR_FIFO,
	SNB_CURSOR_MAXWM,
	SNB_CURSOR_DFTWM,
	2,
	SNB_FIFO_LINE_SIZE
};
static const struct intel_watermark_params sandybridge_display_srwm_info = {
	SNB_DISPLAY_SR_FIFO,
	SNB_DISPLAY_MAX_SRWM,
	SNB_DISPLAY_DFT_SRWM,
	2,
	SNB_FIFO_LINE_SIZE
};
static const struct intel_watermark_params sandybridge_cursor_srwm_info = {
	SNB_CURSOR_SR_FIFO,
	SNB_CURSOR_MAX_SRWM,
	SNB_CURSOR_DFT_SRWM,
	2,
	SNB_FIFO_LINE_SIZE
};


/**
 * intel_calculate_wm - calculate watermark level
 * @clock_in_khz: pixel clock
 * @wm: chip FIFO params
 * @pixel_size: display pixel size
 * @latency_ns: memory latency for the platform
 *
 * Calculate the watermark level (the level at which the display plane will
 * start fetching from memory again).  Each chip has a different display
 * FIFO size and allocation, so the caller needs to figure that out and pass
 * in the correct intel_watermark_params structure.
 *
 * As the pixel clock runs, the FIFO will be drained at a rate that depends
 * on the pixel size.  When it reaches the watermark level, it'll start
 * fetching FIFO line sized based chunks from memory until the FIFO fills
 * past the watermark point.  If the FIFO drains completely, a FIFO underrun
 * will occur, and a display engine hang could result.
 */
static unsigned long intel_calculate_wm(unsigned long clock_in_khz,
					const struct intel_watermark_params *wm,
					int fifo_size,
					int pixel_size,
					unsigned long latency_ns)
{
	long entries_required, wm_size;

	/*
	 * Note: we need to make sure we don't overflow for various clock &
	 * latency values.
	 * clocks go from a few thousand to several hundred thousand.
	 * latency is usually a few thousand
	 */
	entries_required = ((clock_in_khz / 1000) * pixel_size * latency_ns) /
		1000;
	entries_required = DIV_ROUND_UP(entries_required, wm->cacheline_size);

	DRM_DEBUG_KMS("FIFO entries required for mode: %ld\n", entries_required);

	wm_size = fifo_size - (entries_required + wm->guard_size);

	DRM_DEBUG_KMS("FIFO watermark level: %ld\n", wm_size);

	/* Don't promote wm_size to unsigned... */
	if (wm_size > (long)wm->max_wm)
		wm_size = wm->max_wm;
	if (wm_size <= 0)
		wm_size = wm->default_wm;
	return wm_size;
}

struct cxsr_latency {
	int is_desktop;
	int is_ddr3;
	unsigned long fsb_freq;
	unsigned long mem_freq;
	unsigned long display_sr;
	unsigned long display_hpll_disable;
	unsigned long cursor_sr;
	unsigned long cursor_hpll_disable;
};

static const struct cxsr_latency cxsr_latency_table[] = {
	{1, 0, 800, 400, 3382, 33382, 3983, 33983},    /* DDR2-400 SC */
	{1, 0, 800, 667, 3354, 33354, 3807, 33807},    /* DDR2-667 SC */
	{1, 0, 800, 800, 3347, 33347, 3763, 33763},    /* DDR2-800 SC */
	{1, 1, 800, 667, 6420, 36420, 6873, 36873},    /* DDR3-667 SC */
	{1, 1, 800, 800, 5902, 35902, 6318, 36318},    /* DDR3-800 SC */

	{1, 0, 667, 400, 3400, 33400, 4021, 34021},    /* DDR2-400 SC */
	{1, 0, 667, 667, 3372, 33372, 3845, 33845},    /* DDR2-667 SC */
	{1, 0, 667, 800, 3386, 33386, 3822, 33822},    /* DDR2-800 SC */
	{1, 1, 667, 667, 6438, 36438, 6911, 36911},    /* DDR3-667 SC */
	{1, 1, 667, 800, 5941, 35941, 6377, 36377},    /* DDR3-800 SC */

	{1, 0, 400, 400, 3472, 33472, 4173, 34173},    /* DDR2-400 SC */
	{1, 0, 400, 667, 3443, 33443, 3996, 33996},    /* DDR2-667 SC */
	{1, 0, 400, 800, 3430, 33430, 3946, 33946},    /* DDR2-800 SC */
	{1, 1, 400, 667, 6509, 36509, 7062, 37062},    /* DDR3-667 SC */
	{1, 1, 400, 800, 5985, 35985, 6501, 36501},    /* DDR3-800 SC */

	{0, 0, 800, 400, 3438, 33438, 4065, 34065},    /* DDR2-400 SC */
	{0, 0, 800, 667, 3410, 33410, 3889, 33889},    /* DDR2-667 SC */
	{0, 0, 800, 800, 3403, 33403, 3845, 33845},    /* DDR2-800 SC */
	{0, 1, 800, 667, 6476, 36476, 6955, 36955},    /* DDR3-667 SC */
	{0, 1, 800, 800, 5958, 35958, 6400, 36400},    /* DDR3-800 SC */

	{0, 0, 667, 400, 3456, 33456, 4103, 34106},    /* DDR2-400 SC */
	{0, 0, 667, 667, 3428, 33428, 3927, 33927},    /* DDR2-667 SC */
	{0, 0, 667, 800, 3443, 33443, 3905, 33905},    /* DDR2-800 SC */
	{0, 1, 667, 667, 6494, 36494, 6993, 36993},    /* DDR3-667 SC */
	{0, 1, 667, 800, 5998, 35998, 6460, 36460},    /* DDR3-800 SC */

	{0, 0, 400, 400, 3528, 33528, 4255, 34255},    /* DDR2-400 SC */
	{0, 0, 400, 667, 3500, 33500, 4079, 34079},    /* DDR2-667 SC */
	{0, 0, 400, 800, 3487, 33487, 4029, 34029},    /* DDR2-800 SC */
	{0, 1, 400, 667, 6566, 36566, 7145, 37145},    /* DDR3-667 SC */
	{0, 1, 400, 800, 6042, 36042, 6584, 36584},    /* DDR3-800 SC */
};

static const struct cxsr_latency *intel_get_cxsr_latency(int is_desktop,
							 int is_ddr3,
							 int fsb,
							 int mem)
{
	const struct cxsr_latency *latency;
	int i;

	if (fsb == 0 || mem == 0)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(cxsr_latency_table); i++) {
		latency = &cxsr_latency_table[i];
		if (is_desktop == latency->is_desktop &&
		    is_ddr3 == latency->is_ddr3 &&
		    fsb == latency->fsb_freq && mem == latency->mem_freq)
			return latency;
	}

	DRM_DEBUG_KMS("Unknown FSB/MEM found, disable CxSR\n");

	return NULL;
}

static void pineview_disable_cxsr(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* deactivate cxsr */
	I915_WRITE(DSPFW3, I915_READ(DSPFW3) & ~PINEVIEW_SELF_REFRESH_EN);
}

/*
 * Latency for FIFO fetches is dependent on several factors:
 *   - memory configuration (speed, channels)
 *   - chipset
 *   - current MCH state
 * It can be fairly high in some situations, so here we assume a fairly
 * pessimal value.  It's a tradeoff between extra memory fetches (if we
 * set this value too high, the FIFO will fetch frequently to stay full)
 * and power consumption (set it too low to save power and we might see
 * FIFO underruns and display "flicker").
 *
 * A value of 5us seems to be a good balance; safe for very low end
 * platforms but not overly aggressive on lower latency configs.
 */
static const int latency_ns = 5000;

static int i9xx_get_fifo_size(struct drm_device *dev, int plane)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t dsparb = I915_READ(DSPARB);
	int size;

	size = dsparb & 0x7f;
	if (plane)
		size = ((dsparb >> DSPARB_CSTART_SHIFT) & 0x7f) - size;

	DRM_DEBUG_KMS("FIFO size - (0x%08x) %s: %d\n", dsparb,
		      plane ? "B" : "A", size);

	return size;
}

static int i85x_get_fifo_size(struct drm_device *dev, int plane)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t dsparb = I915_READ(DSPARB);
	int size;

	size = dsparb & 0x1ff;
	if (plane)
		size = ((dsparb >> DSPARB_BEND_SHIFT) & 0x1ff) - size;
	size >>= 1; /* Convert to cachelines */

	DRM_DEBUG_KMS("FIFO size - (0x%08x) %s: %d\n", dsparb,
		      plane ? "B" : "A", size);

	return size;
}

static int i845_get_fifo_size(struct drm_device *dev, int plane)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t dsparb = I915_READ(DSPARB);
	int size;

	size = dsparb & 0x7f;
	size >>= 2; /* Convert to cachelines */

	DRM_DEBUG_KMS("FIFO size - (0x%08x) %s: %d\n", dsparb,
		      plane ? "B" : "A",
		      size);

	return size;
}

static int i830_get_fifo_size(struct drm_device *dev, int plane)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t dsparb = I915_READ(DSPARB);
	int size;

	size = dsparb & 0x7f;
	size >>= 1; /* Convert to cachelines */

	DRM_DEBUG_KMS("FIFO size - (0x%08x) %s: %d\n", dsparb,
		      plane ? "B" : "A", size);

	return size;
}

static struct drm_crtc *single_enabled_crtc(struct drm_device *dev)
{
	struct drm_crtc *crtc, *enabled = NULL;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (crtc->enabled && crtc->fb) {
			if (enabled)
				return NULL;
			enabled = crtc;
		}
	}

	return enabled;
}

static void pineview_update_wm(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;
	const struct cxsr_latency *latency;
	u32 reg;
	unsigned long wm;

	latency = intel_get_cxsr_latency(IS_PINEVIEW_G(dev), dev_priv->is_ddr3,
					 dev_priv->fsb_freq, dev_priv->mem_freq);
	if (!latency) {
		DRM_DEBUG_KMS("Unknown FSB/MEM found, disable CxSR\n");
		pineview_disable_cxsr(dev);
		return;
	}

	crtc = single_enabled_crtc(dev);
	if (crtc) {
		int clock = crtc->mode.clock;
		int pixel_size = crtc->fb->bits_per_pixel / 8;

		/* Display SR */
		wm = intel_calculate_wm(clock, &pineview_display_wm,
					pineview_display_wm.fifo_size,
					pixel_size, latency->display_sr);
		reg = I915_READ(DSPFW1);
		reg &= ~DSPFW_SR_MASK;
		reg |= wm << DSPFW_SR_SHIFT;
		I915_WRITE(DSPFW1, reg);
		DRM_DEBUG_KMS("DSPFW1 register is %x\n", reg);

		/* cursor SR */
		wm = intel_calculate_wm(clock, &pineview_cursor_wm,
					pineview_display_wm.fifo_size,
					pixel_size, latency->cursor_sr);
		reg = I915_READ(DSPFW3);
		reg &= ~DSPFW_CURSOR_SR_MASK;
		reg |= (wm & 0x3f) << DSPFW_CURSOR_SR_SHIFT;
		I915_WRITE(DSPFW3, reg);

		/* Display HPLL off SR */
		wm = intel_calculate_wm(clock, &pineview_display_hplloff_wm,
					pineview_display_hplloff_wm.fifo_size,
					pixel_size, latency->display_hpll_disable);
		reg = I915_READ(DSPFW3);
		reg &= ~DSPFW_HPLL_SR_MASK;
		reg |= wm & DSPFW_HPLL_SR_MASK;
		I915_WRITE(DSPFW3, reg);

		/* cursor HPLL off SR */
		wm = intel_calculate_wm(clock, &pineview_cursor_hplloff_wm,
					pineview_display_hplloff_wm.fifo_size,
					pixel_size, latency->cursor_hpll_disable);
		reg = I915_READ(DSPFW3);
		reg &= ~DSPFW_HPLL_CURSOR_MASK;
		reg |= (wm & 0x3f) << DSPFW_HPLL_CURSOR_SHIFT;
		I915_WRITE(DSPFW3, reg);
		DRM_DEBUG_KMS("DSPFW3 register is %x\n", reg);

		/* activate cxsr */
		I915_WRITE(DSPFW3,
			   I915_READ(DSPFW3) | PINEVIEW_SELF_REFRESH_EN);
		DRM_DEBUG_KMS("Self-refresh is enabled\n");
	} else {
		pineview_disable_cxsr(dev);
		DRM_DEBUG_KMS("Self-refresh is disabled\n");
	}
}

static bool g4x_compute_wm0(struct drm_device *dev,
			    int plane,
			    const struct intel_watermark_params *display,
			    int display_latency_ns,
			    const struct intel_watermark_params *cursor,
			    int cursor_latency_ns,
			    int *plane_wm,
			    int *cursor_wm)
{
	struct drm_crtc *crtc;
	int htotal, hdisplay, clock, pixel_size;
	int line_time_us, line_count;
	int entries, tlb_miss;

	crtc = intel_get_crtc_for_plane(dev, plane);
	if (crtc->fb == NULL || !crtc->enabled) {
		*cursor_wm = cursor->guard_size;
		*plane_wm = display->guard_size;
		return false;
	}

	htotal = crtc->mode.htotal;
	hdisplay = crtc->mode.hdisplay;
	clock = crtc->mode.clock;
	pixel_size = crtc->fb->bits_per_pixel / 8;

	/* Use the small buffer method to calculate plane watermark */
	entries = ((clock * pixel_size / 1000) * display_latency_ns) / 1000;
	tlb_miss = display->fifo_size*display->cacheline_size - hdisplay * 8;
	if (tlb_miss > 0)
		entries += tlb_miss;
	entries = DIV_ROUND_UP(entries, display->cacheline_size);
	*plane_wm = entries + display->guard_size;
	if (*plane_wm > (int)display->max_wm)
		*plane_wm = display->max_wm;

	/* Use the large buffer method to calculate cursor watermark */
	line_time_us = ((htotal * 1000) / clock);
	line_count = (cursor_latency_ns / line_time_us + 1000) / 1000;
	entries = line_count * 64 * pixel_size;
	tlb_miss = cursor->fifo_size*cursor->cacheline_size - hdisplay * 8;
	if (tlb_miss > 0)
		entries += tlb_miss;
	entries = DIV_ROUND_UP(entries, cursor->cacheline_size);
	*cursor_wm = entries + cursor->guard_size;
	if (*cursor_wm > (int)cursor->max_wm)
		*cursor_wm = (int)cursor->max_wm;

	return true;
}

/*
 * Check the wm result.
 *
 * If any calculated watermark values is larger than the maximum value that
 * can be programmed into the associated watermark register, that watermark
 * must be disabled.
 */
static bool g4x_check_srwm(struct drm_device *dev,
			   int display_wm, int cursor_wm,
			   const struct intel_watermark_params *display,
			   const struct intel_watermark_params *cursor)
{
	DRM_DEBUG_KMS("SR watermark: display plane %d, cursor %d\n",
		      display_wm, cursor_wm);

	if (display_wm > display->max_wm) {
		DRM_DEBUG_KMS("display watermark is too large(%d/%ld), disabling\n",
			      display_wm, display->max_wm);
		return false;
	}

	if (cursor_wm > cursor->max_wm) {
		DRM_DEBUG_KMS("cursor watermark is too large(%d/%ld), disabling\n",
			      cursor_wm, cursor->max_wm);
		return false;
	}

	if (!(display_wm || cursor_wm)) {
		DRM_DEBUG_KMS("SR latency is 0, disabling\n");
		return false;
	}

	return true;
}

static bool g4x_compute_srwm(struct drm_device *dev,
			     int plane,
			     int latency_ns,
			     const struct intel_watermark_params *display,
			     const struct intel_watermark_params *cursor,
			     int *display_wm, int *cursor_wm)
{
	struct drm_crtc *crtc;
	int hdisplay, htotal, pixel_size, clock;
	unsigned long line_time_us;
	int line_count, line_size;
	int small, large;
	int entries;

	if (!latency_ns) {
		*display_wm = *cursor_wm = 0;
		return false;
	}

	crtc = intel_get_crtc_for_plane(dev, plane);
	hdisplay = crtc->mode.hdisplay;
	htotal = crtc->mode.htotal;
	clock = crtc->mode.clock;
	pixel_size = crtc->fb->bits_per_pixel / 8;

	line_time_us = (htotal * 1000) / clock;
	line_count = (latency_ns / line_time_us + 1000) / 1000;
	line_size = hdisplay * pixel_size;

	/* Use the minimum of the small and large buffer method for primary */
	small = ((clock * pixel_size / 1000) * latency_ns) / 1000;
	large = line_count * line_size;

	entries = DIV_ROUND_UP(min(small, large), display->cacheline_size);
	*display_wm = entries + display->guard_size;

	/* calculate the self-refresh watermark for display cursor */
	entries = line_count * pixel_size * 64;
	entries = DIV_ROUND_UP(entries, cursor->cacheline_size);
	*cursor_wm = entries + cursor->guard_size;

	return g4x_check_srwm(dev,
			      *display_wm, *cursor_wm,
			      display, cursor);
}

#define single_plane_enabled(mask) is_power_of_2(mask)

static void g4x_update_wm(struct drm_device *dev)
{
	static const int sr_latency_ns = 12000;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int planea_wm, planeb_wm, cursora_wm, cursorb_wm;
	int plane_sr, cursor_sr;
	unsigned int enabled = 0;

	if (g4x_compute_wm0(dev, 0,
			    &g4x_wm_info, latency_ns,
			    &g4x_cursor_wm_info, latency_ns,
			    &planea_wm, &cursora_wm))
		enabled |= 1;

	if (g4x_compute_wm0(dev, 1,
			    &g4x_wm_info, latency_ns,
			    &g4x_cursor_wm_info, latency_ns,
			    &planeb_wm, &cursorb_wm))
		enabled |= 2;

	plane_sr = cursor_sr = 0;
	if (single_plane_enabled(enabled) &&
	    g4x_compute_srwm(dev, ffs(enabled) - 1,
			     sr_latency_ns,
			     &g4x_wm_info,
			     &g4x_cursor_wm_info,
			     &plane_sr, &cursor_sr))
		I915_WRITE(FW_BLC_SELF, FW_BLC_SELF_EN);
	else
		I915_WRITE(FW_BLC_SELF,
			   I915_READ(FW_BLC_SELF) & ~FW_BLC_SELF_EN);

	DRM_DEBUG_KMS("Setting FIFO watermarks - A: plane=%d, cursor=%d, B: plane=%d, cursor=%d, SR: plane=%d, cursor=%d\n",
		      planea_wm, cursora_wm,
		      planeb_wm, cursorb_wm,
		      plane_sr, cursor_sr);

	I915_WRITE(DSPFW1,
		   (plane_sr << DSPFW_SR_SHIFT) |
		   (cursorb_wm << DSPFW_CURSORB_SHIFT) |
		   (planeb_wm << DSPFW_PLANEB_SHIFT) |
		   planea_wm);
	I915_WRITE(DSPFW2,
		   (I915_READ(DSPFW2) & DSPFW_CURSORA_MASK) |
		   (cursora_wm << DSPFW_CURSORA_SHIFT));
	/* HPLL off in SR has some issues on G4x... disable it */
	I915_WRITE(DSPFW3,
		   (I915_READ(DSPFW3) & ~DSPFW_HPLL_SR_EN) |
		   (cursor_sr << DSPFW_CURSOR_SR_SHIFT));
}

static void i965_update_wm(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;
	int srwm = 1;
	int cursor_sr = 16;

	/* Calc sr entries for one plane configs */
	crtc = single_enabled_crtc(dev);
	if (crtc) {
		/* self-refresh has much higher latency */
		static const int sr_latency_ns = 12000;
		int clock = crtc->mode.clock;
		int htotal = crtc->mode.htotal;
		int hdisplay = crtc->mode.hdisplay;
		int pixel_size = crtc->fb->bits_per_pixel / 8;
		unsigned long line_time_us;
		int entries;

		line_time_us = ((htotal * 1000) / clock);

		/* Use ns/us then divide to preserve precision */
		entries = (((sr_latency_ns / line_time_us) + 1000) / 1000) *
			pixel_size * hdisplay;
		entries = DIV_ROUND_UP(entries, I915_FIFO_LINE_SIZE);
		srwm = I965_FIFO_SIZE - entries;
		if (srwm < 0)
			srwm = 1;
		srwm &= 0x1ff;
		DRM_DEBUG_KMS("self-refresh entries: %d, wm: %d\n",
			      entries, srwm);

		entries = (((sr_latency_ns / line_time_us) + 1000) / 1000) *
			pixel_size * 64;
		entries = DIV_ROUND_UP(entries,
					  i965_cursor_wm_info.cacheline_size);
		cursor_sr = i965_cursor_wm_info.fifo_size -
			(entries + i965_cursor_wm_info.guard_size);

		if (cursor_sr > i965_cursor_wm_info.max_wm)
			cursor_sr = i965_cursor_wm_info.max_wm;

		DRM_DEBUG_KMS("self-refresh watermark: display plane %d "
			      "cursor %d\n", srwm, cursor_sr);

		if (IS_CRESTLINE(dev))
			I915_WRITE(FW_BLC_SELF, FW_BLC_SELF_EN);
	} else {
		/* Turn off self refresh if both pipes are enabled */
		if (IS_CRESTLINE(dev))
			I915_WRITE(FW_BLC_SELF, I915_READ(FW_BLC_SELF)
				   & ~FW_BLC_SELF_EN);
	}

	DRM_DEBUG_KMS("Setting FIFO watermarks - A: 8, B: 8, C: 8, SR %d\n",
		      srwm);

	/* 965 has limitations... */
	I915_WRITE(DSPFW1, (srwm << DSPFW_SR_SHIFT) |
		   (8 << 16) | (8 << 8) | (8 << 0));
	I915_WRITE(DSPFW2, (8 << 8) | (8 << 0));
	/* update cursor SR watermark */
	I915_WRITE(DSPFW3, (cursor_sr << DSPFW_CURSOR_SR_SHIFT));
}

static void i9xx_update_wm(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	const struct intel_watermark_params *wm_info;
	uint32_t fwater_lo;
	uint32_t fwater_hi;
	int cwm, srwm = 1;
	int fifo_size;
	int planea_wm, planeb_wm;
	struct drm_crtc *crtc, *enabled = NULL;

	if (IS_I945GM(dev))
		wm_info = &i945_wm_info;
	else if (!IS_GEN2(dev))
		wm_info = &i915_wm_info;
	else
		wm_info = &i855_wm_info;

	fifo_size = dev_priv->display.get_fifo_size(dev, 0);
	crtc = intel_get_crtc_for_plane(dev, 0);
	if (crtc->enabled && crtc->fb) {
		planea_wm = intel_calculate_wm(crtc->mode.clock,
					       wm_info, fifo_size,
					       crtc->fb->bits_per_pixel / 8,
					       latency_ns);
		enabled = crtc;
	} else
		planea_wm = fifo_size - wm_info->guard_size;

	fifo_size = dev_priv->display.get_fifo_size(dev, 1);
	crtc = intel_get_crtc_for_plane(dev, 1);
	if (crtc->enabled && crtc->fb) {
		planeb_wm = intel_calculate_wm(crtc->mode.clock,
					       wm_info, fifo_size,
					       crtc->fb->bits_per_pixel / 8,
					       latency_ns);
		if (enabled == NULL)
			enabled = crtc;
		else
			enabled = NULL;
	} else
		planeb_wm = fifo_size - wm_info->guard_size;

	DRM_DEBUG_KMS("FIFO watermarks - A: %d, B: %d\n", planea_wm, planeb_wm);

	/*
	 * Overlay gets an aggressive default since video jitter is bad.
	 */
	cwm = 2;

	/* Play safe and disable self-refresh before adjusting watermarks. */
	if (IS_I945G(dev) || IS_I945GM(dev))
		I915_WRITE(FW_BLC_SELF, FW_BLC_SELF_EN_MASK | 0);
	else if (IS_I915GM(dev))
		I915_WRITE(INSTPM, I915_READ(INSTPM) & ~INSTPM_SELF_EN);

	/* Calc sr entries for one plane configs */
	if (HAS_FW_BLC(dev) && enabled) {
		/* self-refresh has much higher latency */
		static const int sr_latency_ns = 6000;
		int clock = enabled->mode.clock;
		int htotal = enabled->mode.htotal;
		int hdisplay = enabled->mode.hdisplay;
		int pixel_size = enabled->fb->bits_per_pixel / 8;
		unsigned long line_time_us;
		int entries;

		line_time_us = (htotal * 1000) / clock;

		/* Use ns/us then divide to preserve precision */
		entries = (((sr_latency_ns / line_time_us) + 1000) / 1000) *
			pixel_size * hdisplay;
		entries = DIV_ROUND_UP(entries, wm_info->cacheline_size);
		DRM_DEBUG_KMS("self-refresh entries: %d\n", entries);
		srwm = wm_info->fifo_size - entries;
		if (srwm < 0)
			srwm = 1;

		if (IS_I945G(dev) || IS_I945GM(dev))
			I915_WRITE(FW_BLC_SELF,
				   FW_BLC_SELF_FIFO_MASK | (srwm & 0xff));
		else if (IS_I915GM(dev))
			I915_WRITE(FW_BLC_SELF, srwm & 0x3f);
	}

	DRM_DEBUG_KMS("Setting FIFO watermarks - A: %d, B: %d, C: %d, SR %d\n",
		      planea_wm, planeb_wm, cwm, srwm);

	fwater_lo = ((planeb_wm & 0x3f) << 16) | (planea_wm & 0x3f);
	fwater_hi = (cwm & 0x1f);

	/* Set request length to 8 cachelines per fetch */
	fwater_lo = fwater_lo | (1 << 24) | (1 << 8);
	fwater_hi = fwater_hi | (1 << 8);

	I915_WRITE(FW_BLC, fwater_lo);
	I915_WRITE(FW_BLC2, fwater_hi);

	if (HAS_FW_BLC(dev)) {
		if (enabled) {
			if (IS_I945G(dev) || IS_I945GM(dev))
				I915_WRITE(FW_BLC_SELF,
					   FW_BLC_SELF_EN_MASK | FW_BLC_SELF_EN);
			else if (IS_I915GM(dev))
				I915_WRITE(INSTPM, I915_READ(INSTPM) | INSTPM_SELF_EN);
			DRM_DEBUG_KMS("memory self refresh enabled\n");
		} else
			DRM_DEBUG_KMS("memory self refresh disabled\n");
	}
}

static void i830_update_wm(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;
	uint32_t fwater_lo;
	int planea_wm;

	crtc = single_enabled_crtc(dev);
	if (crtc == NULL)
		return;

	planea_wm = intel_calculate_wm(crtc->mode.clock, &i830_wm_info,
				       dev_priv->display.get_fifo_size(dev, 0),
				       crtc->fb->bits_per_pixel / 8,
				       latency_ns);
	fwater_lo = I915_READ(FW_BLC) & ~0xfff;
	fwater_lo |= (3<<8) | planea_wm;

	DRM_DEBUG_KMS("Setting FIFO watermarks - A: %d\n", planea_wm);

	I915_WRITE(FW_BLC, fwater_lo);
}

#define ILK_LP0_PLANE_LATENCY		700
#define ILK_LP0_CURSOR_LATENCY		1300

/*
 * Check the wm result.
 *
 * If any calculated watermark values is larger than the maximum value that
 * can be programmed into the associated watermark register, that watermark
 * must be disabled.
 */
static bool ironlake_check_srwm(struct drm_device *dev, int level,
				int fbc_wm, int display_wm, int cursor_wm,
				const struct intel_watermark_params *display,
				const struct intel_watermark_params *cursor)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	DRM_DEBUG_KMS("watermark %d: display plane %d, fbc lines %d,"
		      " cursor %d\n", level, display_wm, fbc_wm, cursor_wm);

	if (fbc_wm > SNB_FBC_MAX_SRWM) {
		DRM_DEBUG_KMS("fbc watermark(%d) is too large(%d), disabling wm%d+\n",
			      fbc_wm, SNB_FBC_MAX_SRWM, level);

		/* fbc has it's own way to disable FBC WM */
		I915_WRITE(DISP_ARB_CTL,
			   I915_READ(DISP_ARB_CTL) | DISP_FBC_WM_DIS);
		return false;
	}

	if (display_wm > display->max_wm) {
		DRM_DEBUG_KMS("display watermark(%d) is too large(%d), disabling wm%d+\n",
			      display_wm, SNB_DISPLAY_MAX_SRWM, level);
		return false;
	}

	if (cursor_wm > cursor->max_wm) {
		DRM_DEBUG_KMS("cursor watermark(%d) is too large(%d), disabling wm%d+\n",
			      cursor_wm, SNB_CURSOR_MAX_SRWM, level);
		return false;
	}

	if (!(fbc_wm || display_wm || cursor_wm)) {
		DRM_DEBUG_KMS("latency %d is 0, disabling wm%d+\n", level, level);
		return false;
	}

	return true;
}

/*
 * Compute watermark values of WM[1-3],
 */
static bool ironlake_compute_srwm(struct drm_device *dev, int level, int plane,
				  int latency_ns,
				  const struct intel_watermark_params *display,
				  const struct intel_watermark_params *cursor,
				  int *fbc_wm, int *display_wm, int *cursor_wm)
{
	struct drm_crtc *crtc;
	unsigned long line_time_us;
	int hdisplay, htotal, pixel_size, clock;
	int line_count, line_size;
	int small, large;
	int entries;

	if (!latency_ns) {
		*fbc_wm = *display_wm = *cursor_wm = 0;
		return false;
	}

	crtc = intel_get_crtc_for_plane(dev, plane);
	hdisplay = crtc->mode.hdisplay;
	htotal = crtc->mode.htotal;
	clock = crtc->mode.clock;
	pixel_size = crtc->fb->bits_per_pixel / 8;

	line_time_us = (htotal * 1000) / clock;
	line_count = (latency_ns / line_time_us + 1000) / 1000;
	line_size = hdisplay * pixel_size;

	/* Use the minimum of the small and large buffer method for primary */
	small = ((clock * pixel_size / 1000) * latency_ns) / 1000;
	large = line_count * line_size;

	entries = DIV_ROUND_UP(min(small, large), display->cacheline_size);
	*display_wm = entries + display->guard_size;

	/*
	 * Spec says:
	 * FBC WM = ((Final Primary WM * 64) / number of bytes per line) + 2
	 */
	*fbc_wm = DIV_ROUND_UP(*display_wm * 64, line_size) + 2;

	/* calculate the self-refresh watermark for display cursor */
	entries = line_count * pixel_size * 64;
	entries = DIV_ROUND_UP(entries, cursor->cacheline_size);
	*cursor_wm = entries + cursor->guard_size;

	return ironlake_check_srwm(dev, level,
				   *fbc_wm, *display_wm, *cursor_wm,
				   display, cursor);
}

static void ironlake_update_wm(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int fbc_wm, plane_wm, cursor_wm;
	unsigned int enabled;

	enabled = 0;
	if (g4x_compute_wm0(dev, 0,
			    &ironlake_display_wm_info,
			    ILK_LP0_PLANE_LATENCY,
			    &ironlake_cursor_wm_info,
			    ILK_LP0_CURSOR_LATENCY,
			    &plane_wm, &cursor_wm)) {
		I915_WRITE(WM0_PIPEA_ILK,
			   (plane_wm << WM0_PIPE_PLANE_SHIFT) | cursor_wm);
		DRM_DEBUG_KMS("FIFO watermarks For pipe A -"
			      " plane %d, " "cursor: %d\n",
			      plane_wm, cursor_wm);
		enabled |= 1;
	}

	if (g4x_compute_wm0(dev, 1,
			    &ironlake_display_wm_info,
			    ILK_LP0_PLANE_LATENCY,
			    &ironlake_cursor_wm_info,
			    ILK_LP0_CURSOR_LATENCY,
			    &plane_wm, &cursor_wm)) {
		I915_WRITE(WM0_PIPEB_ILK,
			   (plane_wm << WM0_PIPE_PLANE_SHIFT) | cursor_wm);
		DRM_DEBUG_KMS("FIFO watermarks For pipe B -"
			      " plane %d, cursor: %d\n",
			      plane_wm, cursor_wm);
		enabled |= 2;
	}

	/*
	 * Calculate and update the self-refresh watermark only when one
	 * display plane is used.
	 */
	I915_WRITE(WM3_LP_ILK, 0);
	I915_WRITE(WM2_LP_ILK, 0);
	I915_WRITE(WM1_LP_ILK, 0);

	if (!single_plane_enabled(enabled))
		return;
	enabled = ffs(enabled) - 1;

	/* WM1 */
	if (!ironlake_compute_srwm(dev, 1, enabled,
				   ILK_READ_WM1_LATENCY() * 500,
				   &ironlake_display_srwm_info,
				   &ironlake_cursor_srwm_info,
				   &fbc_wm, &plane_wm, &cursor_wm))
		return;

	I915_WRITE(WM1_LP_ILK,
		   WM1_LP_SR_EN |
		   (ILK_READ_WM1_LATENCY() << WM1_LP_LATENCY_SHIFT) |
		   (fbc_wm << WM1_LP_FBC_SHIFT) |
		   (plane_wm << WM1_LP_SR_SHIFT) |
		   cursor_wm);

	/* WM2 */
	if (!ironlake_compute_srwm(dev, 2, enabled,
				   ILK_READ_WM2_LATENCY() * 500,
				   &ironlake_display_srwm_info,
				   &ironlake_cursor_srwm_info,
				   &fbc_wm, &plane_wm, &cursor_wm))
		return;

	I915_WRITE(WM2_LP_ILK,
		   WM2_LP_EN |
		   (ILK_READ_WM2_LATENCY() << WM1_LP_LATENCY_SHIFT) |
		   (fbc_wm << WM1_LP_FBC_SHIFT) |
		   (plane_wm << WM1_LP_SR_SHIFT) |
		   cursor_wm);

	/*
	 * WM3 is unsupported on ILK, probably because we don't have latency
	 * data for that power state
	 */
}

void sandybridge_update_wm(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int latency = SNB_READ_WM0_LATENCY() * 100;	/* In unit 0.1us */
	u32 val;
	int fbc_wm, plane_wm, cursor_wm;
	unsigned int enabled;

	enabled = 0;
	if (g4x_compute_wm0(dev, 0,
			    &sandybridge_display_wm_info, latency,
			    &sandybridge_cursor_wm_info, latency,
			    &plane_wm, &cursor_wm)) {
		val = I915_READ(WM0_PIPEA_ILK);
		val &= ~(WM0_PIPE_PLANE_MASK | WM0_PIPE_CURSOR_MASK);
		I915_WRITE(WM0_PIPEA_ILK, val |
			   ((plane_wm << WM0_PIPE_PLANE_SHIFT) | cursor_wm));
		DRM_DEBUG_KMS("FIFO watermarks For pipe A -"
			      " plane %d, " "cursor: %d\n",
			      plane_wm, cursor_wm);
		enabled |= 1;
	}

	if (g4x_compute_wm0(dev, 1,
			    &sandybridge_display_wm_info, latency,
			    &sandybridge_cursor_wm_info, latency,
			    &plane_wm, &cursor_wm)) {
		val = I915_READ(WM0_PIPEB_ILK);
		val &= ~(WM0_PIPE_PLANE_MASK | WM0_PIPE_CURSOR_MASK);
		I915_WRITE(WM0_PIPEB_ILK, val |
			   ((plane_wm << WM0_PIPE_PLANE_SHIFT) | cursor_wm));
		DRM_DEBUG_KMS("FIFO watermarks For pipe B -"
			      " plane %d, cursor: %d\n",
			      plane_wm, cursor_wm);
		enabled |= 2;
	}

	/* IVB has 3 pipes */
	if (IS_IVYBRIDGE(dev) &&
	    g4x_compute_wm0(dev, 2,
			    &sandybridge_display_wm_info, latency,
			    &sandybridge_cursor_wm_info, latency,
			    &plane_wm, &cursor_wm)) {
		val = I915_READ(WM0_PIPEC_IVB);
		val &= ~(WM0_PIPE_PLANE_MASK | WM0_PIPE_CURSOR_MASK);
		I915_WRITE(WM0_PIPEC_IVB, val |
			   ((plane_wm << WM0_PIPE_PLANE_SHIFT) | cursor_wm));
		DRM_DEBUG_KMS("FIFO watermarks For pipe C -"
			      " plane %d, cursor: %d\n",
			      plane_wm, cursor_wm);
		enabled |= 3;
	}

	/*
	 * Calculate and update the self-refresh watermark only when one
	 * display plane is used.
	 *
	 * SNB support 3 levels of watermark.
	 *
	 * WM1/WM2/WM2 watermarks have to be enabled in the ascending order,
	 * and disabled in the descending order
	 *
	 */
	I915_WRITE(WM3_LP_ILK, 0);
	I915_WRITE(WM2_LP_ILK, 0);
	I915_WRITE(WM1_LP_ILK, 0);

	if (!single_plane_enabled(enabled) ||
	    dev_priv->sprite_scaling_enabled)
		return;
	enabled = ffs(enabled) - 1;

	/* WM1 */
	if (!ironlake_compute_srwm(dev, 1, enabled,
				   SNB_READ_WM1_LATENCY() * 500,
				   &sandybridge_display_srwm_info,
				   &sandybridge_cursor_srwm_info,
				   &fbc_wm, &plane_wm, &cursor_wm))
		return;

	I915_WRITE(WM1_LP_ILK,
		   WM1_LP_SR_EN |
		   (SNB_READ_WM1_LATENCY() << WM1_LP_LATENCY_SHIFT) |
		   (fbc_wm << WM1_LP_FBC_SHIFT) |
		   (plane_wm << WM1_LP_SR_SHIFT) |
		   cursor_wm);

	/* WM2 */
	if (!ironlake_compute_srwm(dev, 2, enabled,
				   SNB_READ_WM2_LATENCY() * 500,
				   &sandybridge_display_srwm_info,
				   &sandybridge_cursor_srwm_info,
				   &fbc_wm, &plane_wm, &cursor_wm))
		return;

	I915_WRITE(WM2_LP_ILK,
		   WM2_LP_EN |
		   (SNB_READ_WM2_LATENCY() << WM1_LP_LATENCY_SHIFT) |
		   (fbc_wm << WM1_LP_FBC_SHIFT) |
		   (plane_wm << WM1_LP_SR_SHIFT) |
		   cursor_wm);

	/* WM3 */
	if (!ironlake_compute_srwm(dev, 3, enabled,
				   SNB_READ_WM3_LATENCY() * 500,
				   &sandybridge_display_srwm_info,
				   &sandybridge_cursor_srwm_info,
				   &fbc_wm, &plane_wm, &cursor_wm))
		return;

	I915_WRITE(WM3_LP_ILK,
		   WM3_LP_EN |
		   (SNB_READ_WM3_LATENCY() << WM1_LP_LATENCY_SHIFT) |
		   (fbc_wm << WM1_LP_FBC_SHIFT) |
		   (plane_wm << WM1_LP_SR_SHIFT) |
		   cursor_wm);
}

static bool
sandybridge_compute_sprite_wm(struct drm_device *dev, int plane,
			      uint32_t sprite_width, int pixel_size,
			      const struct intel_watermark_params *display,
			      int display_latency_ns, int *sprite_wm)
{
	struct drm_crtc *crtc;
	int clock;
	int entries, tlb_miss;

	crtc = intel_get_crtc_for_plane(dev, plane);
	if (crtc->fb == NULL || !crtc->enabled) {
		*sprite_wm = display->guard_size;
		return false;
	}

	clock = crtc->mode.clock;

	/* Use the small buffer method to calculate the sprite watermark */
	entries = ((clock * pixel_size / 1000) * display_latency_ns) / 1000;
	tlb_miss = display->fifo_size*display->cacheline_size -
		sprite_width * 8;
	if (tlb_miss > 0)
		entries += tlb_miss;
	entries = DIV_ROUND_UP(entries, display->cacheline_size);
	*sprite_wm = entries + display->guard_size;
	if (*sprite_wm > (int)display->max_wm)
		*sprite_wm = display->max_wm;

	return true;
}

static bool
sandybridge_compute_sprite_srwm(struct drm_device *dev, int plane,
				uint32_t sprite_width, int pixel_size,
				const struct intel_watermark_params *display,
				int latency_ns, int *sprite_wm)
{
	struct drm_crtc *crtc;
	unsigned long line_time_us;
	int clock;
	int line_count, line_size;
	int small, large;
	int entries;

	if (!latency_ns) {
		*sprite_wm = 0;
		return false;
	}

	crtc = intel_get_crtc_for_plane(dev, plane);
	clock = crtc->mode.clock;
	if (!clock) {
		*sprite_wm = 0;
		return false;
	}

	line_time_us = (sprite_width * 1000) / clock;
	if (!line_time_us) {
		*sprite_wm = 0;
		return false;
	}

	line_count = (latency_ns / line_time_us + 1000) / 1000;
	line_size = sprite_width * pixel_size;

	/* Use the minimum of the small and large buffer method for primary */
	small = ((clock * pixel_size / 1000) * latency_ns) / 1000;
	large = line_count * line_size;

	entries = DIV_ROUND_UP(min(small, large), display->cacheline_size);
	*sprite_wm = entries + display->guard_size;

	return *sprite_wm > 0x3ff ? false : true;
}

static void sandybridge_update_sprite_wm(struct drm_device *dev, int pipe,
					 uint32_t sprite_width, int pixel_size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int latency = SNB_READ_WM0_LATENCY() * 100;	/* In unit 0.1us */
	u32 val;
	int sprite_wm, reg;
	int ret;

	switch (pipe) {
	case 0:
		reg = WM0_PIPEA_ILK;
		break;
	case 1:
		reg = WM0_PIPEB_ILK;
		break;
	case 2:
		reg = WM0_PIPEC_IVB;
		break;
	default:
		return; /* bad pipe */
	}

	ret = sandybridge_compute_sprite_wm(dev, pipe, sprite_width, pixel_size,
					    &sandybridge_display_wm_info,
					    latency, &sprite_wm);
	if (!ret) {
		DRM_DEBUG_KMS("failed to compute sprite wm for pipe %d\n",
			      pipe);
		return;
	}

	val = I915_READ(reg);
	val &= ~WM0_PIPE_SPRITE_MASK;
	I915_WRITE(reg, val | (sprite_wm << WM0_PIPE_SPRITE_SHIFT));
	DRM_DEBUG_KMS("sprite watermarks For pipe %d - %d\n", pipe, sprite_wm);


	ret = sandybridge_compute_sprite_srwm(dev, pipe, sprite_width,
					      pixel_size,
					      &sandybridge_display_srwm_info,
					      SNB_READ_WM1_LATENCY() * 500,
					      &sprite_wm);
	if (!ret) {
		DRM_DEBUG_KMS("failed to compute sprite lp1 wm on pipe %d\n",
			      pipe);
		return;
	}
	I915_WRITE(WM1S_LP_ILK, sprite_wm);

	/* Only IVB has two more LP watermarks for sprite */
	if (!IS_IVYBRIDGE(dev))
		return;

	ret = sandybridge_compute_sprite_srwm(dev, pipe, sprite_width,
					      pixel_size,
					      &sandybridge_display_srwm_info,
					      SNB_READ_WM2_LATENCY() * 500,
					      &sprite_wm);
	if (!ret) {
		DRM_DEBUG_KMS("failed to compute sprite lp2 wm on pipe %d\n",
			      pipe);
		return;
	}
	I915_WRITE(WM2S_LP_IVB, sprite_wm);

	ret = sandybridge_compute_sprite_srwm(dev, pipe, sprite_width,
					      pixel_size,
					      &sandybridge_display_srwm_info,
					      SNB_READ_WM3_LATENCY() * 500,
					      &sprite_wm);
	if (!ret) {
		DRM_DEBUG_KMS("failed to compute sprite lp3 wm on pipe %d\n",
			      pipe);
		return;
	}
	I915_WRITE(WM3S_LP_IVB, sprite_wm);
}

/**
 * intel_update_watermarks - update FIFO watermark values based on current modes
 *
 * Calculate watermark values for the various WM regs based on current mode
 * and plane configuration.
 *
 * There are several cases to deal with here:
 *   - normal (i.e. non-self-refresh)
 *   - self-refresh (SR) mode
 *   - lines are large relative to FIFO size (buffer can hold up to 2)
 *   - lines are small relative to FIFO size (buffer can hold more than 2
 *     lines), so need to account for TLB latency
 *
 *   The normal calculation is:
 *     watermark = dotclock * bytes per pixel * latency
 *   where latency is platform & configuration dependent (we assume pessimal
 *   values here).
 *
 *   The SR calculation is:
 *     watermark = (trunc(latency/line time)+1) * surface width *
 *       bytes per pixel
 *   where
 *     line time = htotal / dotclock
 *     surface width = hdisplay for normal plane and 64 for cursor
 *   and latency is assumed to be high, as above.
 *
 * The final value programmed to the register should always be rounded up,
 * and include an extra 2 entries to account for clock crossings.
 *
 * We don't use the sprite, so we can ignore that.  And on Crestline we have
 * to set the non-SR watermarks to 8.
 */
static void intel_update_watermarks(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (dev_priv->display.update_wm)
		dev_priv->display.update_wm(dev);
}

void intel_update_sprite_watermarks(struct drm_device *dev, int pipe,
				    uint32_t sprite_width, int pixel_size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (dev_priv->display.update_sprite_wm)
		dev_priv->display.update_sprite_wm(dev, pipe, sprite_width,
						   pixel_size);
}

static inline bool intel_panel_use_ssc(struct drm_i915_private *dev_priv)
{
	if (i915_panel_use_ssc >= 0)
		return i915_panel_use_ssc != 0;
	return dev_priv->lvds_use_ssc
		&& !(dev_priv->quirks & QUIRK_LVDS_SSC_DISABLE);
}

/**
 * intel_choose_pipe_bpp_dither - figure out what color depth the pipe should send
 * @crtc: CRTC structure
 * @mode: requested mode
 *
 * A pipe may be connected to one or more outputs.  Based on the depth of the
 * attached framebuffer, choose a good color depth to use on the pipe.
 *
 * If possible, match the pipe depth to the fb depth.  In some cases, this
 * isn't ideal, because the connected output supports a lesser or restricted
 * set of depths.  Resolve that here:
 *    LVDS typically supports only 6bpc, so clamp down in that case
 *    HDMI supports only 8bpc or 12bpc, so clamp to 8bpc with dither for 10bpc
 *    Displays may support a restricted set as well, check EDID and clamp as
 *      appropriate.
 *    DP may want to dither down to 6bpc to fit larger modes
 *
 * RETURNS:
 * Dithering requirement (i.e. false if display bpc and pipe bpc match,
 * true if they don't match).
 */
static bool intel_choose_pipe_bpp_dither(struct drm_crtc *crtc,
					 struct drm_framebuffer *fb,
					 unsigned int *pipe_bpp,
					 struct drm_display_mode *mode)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_connector *connector;
	struct intel_encoder *intel_encoder;
	unsigned int display_bpc = UINT_MAX, bpc;

	/* Walk the encoders & connectors on this crtc, get min bpc */
	for_each_encoder_on_crtc(dev, crtc, intel_encoder) {

		if (intel_encoder->type == INTEL_OUTPUT_LVDS) {
			unsigned int lvds_bpc;

			if ((I915_READ(PCH_LVDS) & LVDS_A3_POWER_MASK) ==
			    LVDS_A3_POWER_UP)
				lvds_bpc = 8;
			else
				lvds_bpc = 6;

			if (lvds_bpc < display_bpc) {
				DRM_DEBUG_KMS("clamping display bpc (was %d) to LVDS (%d)\n", display_bpc, lvds_bpc);
				display_bpc = lvds_bpc;
			}
			continue;
		}

		/* Not one of the known troublemakers, check the EDID */
		list_for_each_entry(connector, &dev->mode_config.connector_list,
				    head) {
			if (connector->encoder != encoder)
				continue;

			/* Don't use an invalid EDID bpc value */
			if (connector->display_info.bpc &&
			    connector->display_info.bpc < display_bpc) {
				DRM_DEBUG_KMS("clamping display bpc (was %d) to EDID reported max of %d\n", display_bpc, connector->display_info.bpc);
				display_bpc = connector->display_info.bpc;
			}
		}

		if (intel_encoder->type == INTEL_OUTPUT_EDP) {
			/* Use VBT settings if we have an eDP panel */
			unsigned int edp_bpc = dev_priv->edp.bpp / 3;

			if (edp_bpc && edp_bpc < display_bpc) {
				DRM_DEBUG_KMS("clamping display bpc (was %d) to eDP (%d)\n", display_bpc, edp_bpc);
				display_bpc = edp_bpc;
			}
			continue;
		}

		if (intel_encoder->type == INTEL_OUTPUT_EDP) {
			/* Use VBT settings if we have an eDP panel */
			unsigned int edp_bpc = dev_priv->edp.bpp / 3;

			if (edp_bpc && edp_bpc < display_bpc) {
				DRM_DEBUG_KMS("clamping display bpc (was %d) to eDP (%d)\n", display_bpc, edp_bpc);
				display_bpc = edp_bpc;
			}
			continue;
		}

		/*
		 * HDMI is either 12 or 8, so if the display lets 10bpc sneak
		 * through, clamp it down.  (Note: >12bpc will be caught below.)
		 */
		if (intel_encoder->type == INTEL_OUTPUT_HDMI) {
			if (display_bpc > 8 && display_bpc < 12) {
				DRM_DEBUG_KMS("forcing bpc to 12 for HDMI\n");
				display_bpc = 12;
			} else {
				DRM_DEBUG_KMS("forcing bpc to 8 for HDMI\n");
				display_bpc = 8;
			}
		}
	}

	if (mode->private_flags & INTEL_MODE_DP_FORCE_6BPC) {
		DRM_DEBUG_KMS("Dithering DP to 6bpc\n");
		display_bpc = 6;
	}

	/*
	 * We could just drive the pipe at the highest bpc all the time and
	 * enable dithering as needed, but that costs bandwidth.  So choose
	 * the minimum value that expresses the full color range of the fb but
	 * also stays within the max display bpc discovered above.
	 */

	switch (fb->depth) {
	case 8:
		bpc = 8; /* since we go through a colormap */
		break;
	case 15:
	case 16:
		bpc = 6; /* min is 18bpp */
		break;
	case 24:
		bpc = 8;
		break;
	case 30:
		bpc = 10;
		break;
	case 48:
		bpc = 12;
		break;
	default:
		DRM_DEBUG("unsupported depth, assuming 24 bits\n");
		bpc = min((unsigned int)8, display_bpc);
		break;
	}

	display_bpc = min(display_bpc, bpc);

	DRM_DEBUG_KMS("setting pipe bpc to %d (max display bpc %d)\n",
		      bpc, display_bpc);

	*pipe_bpp = display_bpc * 3;

	return display_bpc != bpc;
}

static int vlv_get_refclk(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int refclk = 27000; /* for DP & HDMI */

	return 100000; /* only one validated so far */

	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_ANALOG)) {
		refclk = 96000;
	} else if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS)) {
		if (intel_panel_use_ssc(dev_priv))
			refclk = 100000;
		else
			refclk = 96000;
	} else if (intel_pipe_has_type(crtc, INTEL_OUTPUT_EDP)) {
		refclk = 100000;
	}

	return refclk;
}

static int i9xx_get_refclk(struct drm_crtc *crtc, int num_connectors)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int refclk;

	if (IS_VALLEYVIEW(dev)) {
		refclk = vlv_get_refclk(crtc);
	} else if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS) &&
	    intel_panel_use_ssc(dev_priv) && num_connectors < 2) {
		refclk = dev_priv->lvds_ssc_freq * 1000;
		DRM_DEBUG_KMS("using SSC reference clock of %d MHz\n",
			      refclk / 1000);
	} else if (!IS_GEN2(dev)) {
		refclk = 96000;
	} else {
		refclk = 48000;
	}

	return refclk;
}

static void i9xx_adjust_sdvo_tv_clock(struct intel_crtc *crtc)
{
	unsigned dotclock = crtc->config.adjusted_mode.clock;
	struct dpll *clock = &crtc->config.dpll;

	/* SDVO TV has fixed PLL values depend on its clock range,
	   this mirrors vbios setting. */
	if (dotclock >= 100000 && dotclock < 140500) {
		clock->p1 = 2;
		clock->p2 = 10;
		clock->n = 3;
		clock->m1 = 16;
		clock->m2 = 8;
	} else if (dotclock >= 140500 && dotclock <= 200000) {
		clock->p1 = 1;
		clock->p2 = 10;
		clock->n = 6;
		clock->m1 = 12;
		clock->m2 = 8;
	}

	crtc->config.clock_set = true;
}

static void i9xx_update_pll_dividers(struct intel_crtc *crtc,
				     intel_clock_t *reduced_clock)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe = crtc->pipe;
	u32 fp, fp2 = 0;
	struct dpll *clock = &crtc->config.dpll;

	if (IS_PINEVIEW(dev)) {
		fp = (1 << clock->n) << 16 | clock->m1 << 8 | clock->m2;
		if (reduced_clock)
			fp2 = (1 << reduced_clock->n) << 16 |
				reduced_clock->m1 << 8 | reduced_clock->m2;
	} else {
		fp = clock->n << 16 | clock->m1 << 8 | clock->m2;
		if (reduced_clock)
			fp2 = reduced_clock->n << 16 | reduced_clock->m1 << 8 |
				reduced_clock->m2;
	}

	I915_WRITE(FP0(pipe), fp);

	intel_crtc->lowfreq_avail = false;
	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS) &&
	    reduced_clock && i915_powersave) {
		I915_WRITE(FP1(pipe), fp2);
		intel_crtc->lowfreq_avail = true;
	} else {
		I915_WRITE(FP1(pipe), fp);
	}
}

static int i9xx_crtc_mode_set(struct drm_crtc *crtc,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode,
			      int x, int y,
			      struct drm_framebuffer *fb)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	int refclk, num_connectors = 0;
	intel_clock_t clock, reduced_clock;
	u32 dpll, fp = 0, fp2 = 0, dspcntr, pipeconf;
	bool ok, has_reduced_clock = false, is_sdvo = false;
	bool is_crt = false, is_lvds = false, is_tv = false, is_dp = false;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *encoder, *edp_encoder = NULL;
	const intel_limit_t *limit;
	int ret;
	struct fdi_m_n m_n = {0};
	u32 temp;
	int target_clock, pixel_multiplier, lane, link_bw, factor;
	unsigned int pipe_bpp;
	bool dither;
	bool is_cpu_edp = false, is_pch_edp = false;

	list_for_each_entry(encoder, &mode_config->encoder_list, base.head) {
		if (encoder->base.crtc != crtc)
			continue;

		switch (encoder->type) {
		case INTEL_OUTPUT_LVDS:
			is_lvds = true;
			break;
		case INTEL_OUTPUT_SDVO:
		case INTEL_OUTPUT_HDMI:
			is_sdvo = true;
			if (encoder->needs_tv_clock)
				is_tv = true;
			break;
		case INTEL_OUTPUT_TVOUT:
			is_tv = true;
			break;
		case INTEL_OUTPUT_ANALOG:
			is_crt = true;
			break;
		case INTEL_OUTPUT_DISPLAYPORT:
			is_dp = true;
			break;
		case INTEL_OUTPUT_EDP:
			is_dp = true;
			if (intel_encoder_is_pch_edp(&encoder->base))
				is_pch_edp = true;
			else
				is_cpu_edp = true;
			edp_encoder = encoder;
			break;
		}

		num_connectors++;
	}

	refclk = ironlake_get_refclk(crtc);

	/*
	 * Returns a set of divisors for the desired target clock with the given
	 * refclk, or FALSE.  The returned values represent the clock equation:
	 * reflck * (5 * (m1 + 2) + (m2 + 2)) / (n + 2) / p1 / p2.
	 */
	limit = intel_limit(crtc, refclk);
	ok = limit->find_pll(limit, crtc, adjusted_mode->clock, refclk, NULL,
			     &clock);
	if (!ok) {
		DRM_ERROR("Couldn't find PLL settings for mode!\n");
		return -EINVAL;
	}

	/* Ensure that the cursor is valid for the new mode before changing... */
	intel_crtc_update_cursor(crtc, true);

	if (is_lvds && dev_priv->lvds_downclock_avail) {
		/*
		 * Ensure we match the reduced clock's P to the target clock.
		 * If the clocks don't match, we can't switch the display clock
		 * by using the FP0/FP1. In such case we will disable the LVDS
		 * downclock feature.
		*/
		has_reduced_clock = limit->find_pll(limit, crtc,
						    dev_priv->lvds_downclock,
						    refclk,
						    &clock,
						    &reduced_clock);
	}

	if (is_sdvo && is_tv)
		i9xx_adjust_sdvo_tv_clock(adjusted_mode, &clock);

	i9xx_update_pll_dividers(crtc, &clock, has_reduced_clock ?
				 &reduced_clock : NULL);

	i9xx_update_pll_dividers(crtc, clock, reduced_clock);

	dpll = DPLL_VGA_MODE_DIS;

	if (!IS_GEN2(dev)) {
		if (is_lvds)
			dpll |= DPLLB_MODE_LVDS;
		else
			dpll |= DPLLB_MODE_DAC_SERIAL;
		if (is_sdvo) {
			int pixel_multiplier = intel_mode_get_pixel_multiplier(adjusted_mode);
			if (pixel_multiplier > 1) {
				if (IS_I945G(dev) || IS_I945GM(dev) || IS_G33(dev))
					dpll |= (pixel_multiplier - 1) << SDVO_MULTIPLIER_SHIFT_HIRES;
			}
			dpll |= DPLL_DVO_HIGH_SPEED;
		}
		if (is_dp)
			dpll |= DPLL_DVO_HIGH_SPEED;

		/* compute bitmask from p1 value */
		if (IS_PINEVIEW(dev))
			dpll |= (1 << (clock.p1 - 1)) << DPLL_FPA01_P1_POST_DIV_SHIFT_PINEVIEW;
		else {
			dpll |= (1 << (clock.p1 - 1)) << DPLL_FPA01_P1_POST_DIV_SHIFT;
			if (IS_G4X(dev) && has_reduced_clock)
				dpll |= (1 << (reduced_clock.p1 - 1)) << DPLL_FPA1_P1_POST_DIV_SHIFT;
		}
		switch (clock.p2) {
		case 5:
			dpll |= DPLL_DAC_SERIAL_P2_CLOCK_DIV_5;
			break;
		case 7:
			dpll |= DPLLB_LVDS_P2_CLOCK_DIV_7;
			break;
		case 10:
			dpll |= DPLL_DAC_SERIAL_P2_CLOCK_DIV_10;
			break;
		case 14:
			dpll |= DPLLB_LVDS_P2_CLOCK_DIV_14;
			break;
		}
		if (INTEL_INFO(dev)->gen >= 4)
			dpll |= (6 << PLL_LOAD_PULSE_PHASE_SHIFT);
	} else {
		if (is_lvds) {
			dpll |= (1 << (clock.p1 - 1)) << DPLL_FPA01_P1_POST_DIV_SHIFT;
		} else {
			if (clock.p1 == 2)
				dpll |= PLL_P1_DIVIDE_BY_TWO;
			else
				dpll |= (clock.p1 - 2) << DPLL_FPA01_P1_POST_DIV_SHIFT;
			if (clock.p2 == 4)
				dpll |= PLL_P2_DIVIDE_BY_4;
		}
	}

	if (is_sdvo && is_tv)
		dpll |= PLL_REF_INPUT_TVCLKINBC;
	else if (is_tv)
		/* XXX: just matching BIOS for now */
		/*	dpll |= PLL_REF_INPUT_TVCLKINBC; */
		dpll |= 3;
	else if (is_lvds && intel_panel_use_ssc(dev_priv) && num_connectors < 2)
		dpll |= PLLB_REF_INPUT_SPREADSPECTRUMIN;
	else
		dpll |= PLL_REF_INPUT_DREFCLK;

	/* setup pipeconf */
	pipeconf = I915_READ(PIPECONF(pipe));

	/* Set up the display plane register */
	dspcntr = DISPPLANE_GAMMA_ENABLE;

	if (pipe == 0)
		dspcntr &= ~DISPPLANE_SEL_PIPE_MASK;
	else
		dspcntr |= DISPPLANE_SEL_PIPE_B;

	if (pipe == 0 && INTEL_INFO(dev)->gen < 4) {
		/* Enable pixel doubling when the dot clock is > 90% of the (display)
		 * core speed.
		 *
		 * XXX: No double-wide on 915GM pipe B. Is that the only reason for the
		 * pipe == 0 check?
		 */
		if (mode->clock >
		    dev_priv->display.get_display_clock_speed(dev) * 9 / 10)
			pipeconf |= PIPECONF_DOUBLE_WIDE;
		else
			pipeconf &= ~PIPECONF_DOUBLE_WIDE;
	}

	/* default to 8bpc */
	pipeconf &= ~(PIPECONF_BPP_MASK | PIPECONF_DITHER_EN);
	if (is_dp) {
		if (adjusted_mode->private_flags & INTEL_MODE_DP_FORCE_6BPC) {
			pipeconf |= PIPECONF_BPP_6 |
				    PIPECONF_DITHER_EN |
				    PIPECONF_DITHER_TYPE_SP;
		}
	}

	dpll |= DPLL_VCO_ENABLE;

	if (IS_VALLEYVIEW(dev) && intel_pipe_has_type(crtc, INTEL_OUTPUT_EDP)) {
		if (adjusted_mode->private_flags & INTEL_MODE_DP_FORCE_6BPC) {
			pipeconf |= PIPECONF_BPP_6 |
					PIPECONF_ENABLE |
					I965_PIPECONF_ACTIVE;
		}
	}

	DRM_DEBUG_KMS("Mode for pipe %c:\n", pipe == 0 ? 'A' : 'B');
	drm_mode_debug_printmodeline(mode);

	I915_WRITE(DPLL(pipe), dpll & ~DPLL_VCO_ENABLE);

	POSTING_READ(DPLL(pipe));
	udelay(150);

	/* The LVDS pin pair needs to be on before the DPLLs are enabled.
	 * This is an exception to the general rule that mode_set doesn't turn
	 * things on.
	 */
	if (is_lvds) {
		temp = I915_READ(LVDS);
		temp |= LVDS_PORT_EN | LVDS_A0A2_CLKA_POWER_UP;
		if (pipe == 1) {
			temp |= LVDS_PIPEB_SELECT;
		} else {
			temp &= ~LVDS_PIPEB_SELECT;
		}
		/* set the corresponsding LVDS_BORDER bit */
		temp |= dev_priv->lvds_border_bits;
		/* Set the B0-B3 data pairs corresponding to whether we're going to
		 * set the DPLLs for dual-channel mode or not.
		 */
		if (clock.p2 == 7)
			temp |= LVDS_B0B3_POWER_UP | LVDS_CLKB_POWER_UP;
		else
			temp &= ~(LVDS_B0B3_POWER_UP | LVDS_CLKB_POWER_UP);

		/* It would be nice to set 24 vs 18-bit mode (LVDS_A3_POWER_UP)
		 * appropriately here, but we need to look more thoroughly into how
		 * panels behave in the two modes.
		 */
		/* set the dithering flag on LVDS as needed */
		if (INTEL_INFO(dev)->gen >= 4) {
			if (dev_priv->lvds_dither)
				temp |= LVDS_ENABLE_DITHER;
			else
				temp &= ~LVDS_ENABLE_DITHER;
		}
		if (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC)
			lvds_sync |= LVDS_HSYNC_POLARITY;
		if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
			lvds_sync |= LVDS_VSYNC_POLARITY;
		if ((temp & (LVDS_HSYNC_POLARITY | LVDS_VSYNC_POLARITY))
		    != lvds_sync) {
			char flags[2] = "-+";
			DRM_INFO("Changing LVDS panel from "
				 "(%chsync, %cvsync) to (%chsync, %cvsync)\n",
				 flags[!(temp & LVDS_HSYNC_POLARITY)],
				 flags[!(temp & LVDS_VSYNC_POLARITY)],
				 flags[!(lvds_sync & LVDS_HSYNC_POLARITY)],
				 flags[!(lvds_sync & LVDS_VSYNC_POLARITY)]);
			temp &= ~(LVDS_HSYNC_POLARITY | LVDS_VSYNC_POLARITY);
			temp |= lvds_sync;
		}
		I915_WRITE(LVDS, temp);
	}

	if (is_dp) {
		intel_dp_set_m_n(crtc, mode, adjusted_mode);
	}

	I915_WRITE(DPLL(pipe), dpll);

	/* Wait for the clocks to stabilize. */
	POSTING_READ(DPLL(pipe));
	udelay(150);

	if (INTEL_INFO(dev)->gen >= 4) {
		temp = 0;
		if (is_sdvo) {
			temp = 0;
			if (crtc->config.pixel_multiplier > 1) {
				temp = (crtc->config.pixel_multiplier - 1)
					<< DPLL_MD_UDI_MULTIPLIER_SHIFT;
			}
		}
		I915_WRITE(DPLL_MD(pipe), temp);
	} else {
		/* The pixel multiplier can only be updated once the
		 * DPLL is enabled and the clocks are stable.
		 *
		 * So write it again.
		 */
		I915_WRITE(DPLL(pipe), dpll);
	}

	if (HAS_PIPE_CXSR(dev)) {
		if (intel_crtc->lowfreq_avail) {
			DRM_DEBUG_KMS("enabling CxSR downclocking\n");
			pipeconf |= PIPECONF_CXSR_DOWNCLOCK;
		} else {
			DRM_DEBUG_KMS("disabling CxSR downclocking\n");
			pipeconf &= ~PIPECONF_CXSR_DOWNCLOCK;
		}
	}

	pipeconf &= ~PIPECONF_INTERLACE_MASK;
	if (!IS_GEN2(dev) &&
	    adjusted_mode->flags & DRM_MODE_FLAG_INTERLACE)
		pipeconf |= PIPECONF_INTERLACE_W_FIELD_INDICATION;
	else
		pipeconf |= PIPECONF_PROGRESSIVE;

	intel_set_pipe_timings(intel_crtc, mode, adjusted_mode);

	/* pipesrc and dspsize control the size that is scaled from,
	 * which should always be the user's requested size.
	 */
	I915_WRITE(DSPSIZE(plane),
		   ((mode->vdisplay - 1) << 16) |
		   (mode->hdisplay - 1));
	I915_WRITE(DSPPOS(plane), 0);

	i9xx_set_pipeconf(intel_crtc);

	intel_enable_pipe(dev_priv, pipe, false);

	intel_wait_for_vblank(dev, pipe);

	I915_WRITE(DSPCNTR(plane), dspcntr);
	POSTING_READ(DSPCNTR(plane));
	intel_enable_plane(dev_priv, plane, pipe);

	ret = intel_pipe_set_base(crtc, x, y, old_fb);

	intel_update_watermarks(dev);

	return ret;
}

static bool i9xx_get_pipe_config(struct intel_crtc *crtc,
				 struct intel_crtc_config *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t tmp;

	tmp = I915_READ(PIPECONF(crtc->pipe));
	if (!(tmp & PIPECONF_ENABLE))
		return false;

	return true;
}

static void ironlake_init_pch_refclk(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *encoder;
	u32 val, final;
	bool has_lvds = false;
	bool has_cpu_edp = false;
	bool has_pch_edp = false;
	bool has_panel = false;
	bool has_ck505 = false;
	bool can_ssc = false;

	/* We need to take the global config into account */
	list_for_each_entry(encoder, &mode_config->encoder_list,
			    base.head) {
		switch (encoder->type) {
		case INTEL_OUTPUT_LVDS:
			has_panel = true;
			has_lvds = true;
			break;
		case INTEL_OUTPUT_EDP:
			has_panel = true;
			if (intel_encoder_is_pch_edp(&encoder->base))
				has_pch_edp = true;
			else
				has_cpu_edp = true;
			break;
		}
	}

	if (HAS_PCH_IBX(dev)) {
		has_ck505 = dev_priv->display_clock_mode;
		can_ssc = has_ck505;
	} else {
		has_ck505 = false;
		can_ssc = true;
	}

	DRM_DEBUG_KMS("has_panel %d has_lvds %d has_pch_edp %d has_cpu_edp %d has_ck505 %d\n",
		      has_panel, has_lvds, has_pch_edp, has_cpu_edp,
		      has_ck505);

	/* Ironlake: try to setup display ref clock before DPLL
	 * enabling. This is only under driver's control after
	 * PCH B stepping, previous chipset stepping should be
	 * ignoring this setting.
	 */
	val = I915_READ(PCH_DREF_CONTROL);

	/* As we must carefully and slowly disable/enable each source in turn,
	 * compute the final state we want first and check if we need to
	 * make any changes at all.
	 */
	final = val;
	final &= ~DREF_NONSPREAD_SOURCE_MASK;
	if (has_ck505)
		final |= DREF_NONSPREAD_CK505_ENABLE;
	else
		final |= DREF_NONSPREAD_SOURCE_ENABLE;

	final &= ~DREF_SSC_SOURCE_MASK;
	final &= ~DREF_CPU_SOURCE_OUTPUT_MASK;
	final &= ~DREF_SSC1_ENABLE;

	if (has_panel) {
		final |= DREF_SSC_SOURCE_ENABLE;

		if (intel_panel_use_ssc(dev_priv) && can_ssc)
			final |= DREF_SSC1_ENABLE;

		if (has_cpu_edp) {
			if (intel_panel_use_ssc(dev_priv) && can_ssc)
				final |= DREF_CPU_SOURCE_OUTPUT_DOWNSPREAD;
			else
				final |= DREF_CPU_SOURCE_OUTPUT_NONSPREAD;
		} else
			final |= DREF_CPU_SOURCE_OUTPUT_DISABLE;
	} else {
		final |= DREF_SSC_SOURCE_DISABLE;
		final |= DREF_CPU_SOURCE_OUTPUT_DISABLE;
	}

	if (final == val)
		return;

	/* Always enable nonspread source */
	val &= ~DREF_NONSPREAD_SOURCE_MASK;

	if (has_ck505)
		val |= DREF_NONSPREAD_CK505_ENABLE;
	else
		val |= DREF_NONSPREAD_SOURCE_ENABLE;

	if (has_panel) {
		val &= ~DREF_SSC_SOURCE_MASK;
		val |= DREF_SSC_SOURCE_ENABLE;

		/* SSC must be turned on before enabling the CPU output  */
		if (intel_panel_use_ssc(dev_priv) && can_ssc) {
			DRM_DEBUG_KMS("Using SSC on panel\n");
			val |= DREF_SSC1_ENABLE;
		} else
			val &= ~DREF_SSC1_ENABLE;

		/* Get SSC going before enabling the outputs */
		I915_WRITE(PCH_DREF_CONTROL, val);
		POSTING_READ(PCH_DREF_CONTROL);
		udelay(200);

		val &= ~DREF_CPU_SOURCE_OUTPUT_MASK;

		/* Enable CPU source on CPU attached eDP */
		if (has_cpu_edp) {
			if (intel_panel_use_ssc(dev_priv) && can_ssc) {
				DRM_DEBUG_KMS("Using SSC on eDP\n");
				val |= DREF_CPU_SOURCE_OUTPUT_DOWNSPREAD;
			}
			else
				val |= DREF_CPU_SOURCE_OUTPUT_NONSPREAD;
		} else
			val |= DREF_CPU_SOURCE_OUTPUT_DISABLE;

		I915_WRITE(PCH_DREF_CONTROL, val);
		POSTING_READ(PCH_DREF_CONTROL);
		udelay(200);
	} else {
		DRM_DEBUG_KMS("Disabling SSC entirely\n");

		val &= ~DREF_CPU_SOURCE_OUTPUT_MASK;

		/* Turn off CPU output */
		val |= DREF_CPU_SOURCE_OUTPUT_DISABLE;

		I915_WRITE(PCH_DREF_CONTROL, val);
		POSTING_READ(PCH_DREF_CONTROL);
		udelay(200);

		/* Turn off the SSC source */
		val &= ~DREF_SSC_SOURCE_MASK;
		val |= DREF_SSC_SOURCE_DISABLE;

		/* Turn off SSC1 */
		val &= ~DREF_SSC1_ENABLE;

		I915_WRITE(PCH_DREF_CONTROL, val);
		POSTING_READ(PCH_DREF_CONTROL);
		udelay(200);
	}

	BUG_ON(val != final);
}

/* Sequence to enable CLKOUT_DP for FDI usage and configure PCH FDI I/O. */
static void lpt_init_pch_refclk(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *encoder;
	bool has_vga = false;
	bool is_sdv = false;
	u32 tmp;

	list_for_each_entry(encoder, &mode_config->encoder_list, base.head) {
		switch (encoder->type) {
		case INTEL_OUTPUT_ANALOG:
			has_vga = true;
			break;
		}
	}

	if (!has_vga)
		return;

	/* XXX: Rip out SDV support once Haswell ships for real. */
	if (IS_HASWELL(dev) && (dev->pci_device & 0xFF00) == 0x0C00)
		is_sdv = true;

	tmp = intel_sbi_read(dev_priv, SBI_SSCCTL, SBI_ICLK);
	tmp &= ~SBI_SSCCTL_DISABLE;
	tmp |= SBI_SSCCTL_PATHALT;
	intel_sbi_write(dev_priv, SBI_SSCCTL, tmp, SBI_ICLK);

	udelay(24);

	tmp = intel_sbi_read(dev_priv, SBI_SSCCTL, SBI_ICLK);
	tmp &= ~SBI_SSCCTL_PATHALT;
	intel_sbi_write(dev_priv, SBI_SSCCTL, tmp, SBI_ICLK);

	if (!is_sdv) {
		tmp = I915_READ(SOUTH_CHICKEN2);
		tmp |= FDI_MPHY_IOSFSB_RESET_CTL;
		I915_WRITE(SOUTH_CHICKEN2, tmp);

		if (wait_for_atomic_us(I915_READ(SOUTH_CHICKEN2) &
				       FDI_MPHY_IOSFSB_RESET_STATUS, 100))
			DRM_ERROR("FDI mPHY reset assert timeout\n");

		tmp = I915_READ(SOUTH_CHICKEN2);
		tmp &= ~FDI_MPHY_IOSFSB_RESET_CTL;
		I915_WRITE(SOUTH_CHICKEN2, tmp);

		if (wait_for_atomic_us((I915_READ(SOUTH_CHICKEN2) &
				        FDI_MPHY_IOSFSB_RESET_STATUS) == 0,
				       100))
			DRM_ERROR("FDI mPHY reset de-assert timeout\n");
	}

	tmp = intel_sbi_read(dev_priv, 0x8008, SBI_MPHY);
	tmp &= ~(0xFF << 24);
	tmp |= (0x12 << 24);
	intel_sbi_write(dev_priv, 0x8008, tmp, SBI_MPHY);

	if (is_sdv) {
		tmp = intel_sbi_read(dev_priv, 0x800C, SBI_MPHY);
		tmp |= 0x7FFF;
		intel_sbi_write(dev_priv, 0x800C, tmp, SBI_MPHY);
	}

	tmp = intel_sbi_read(dev_priv, 0x2008, SBI_MPHY);
	tmp |= (1 << 11);
	intel_sbi_write(dev_priv, 0x2008, tmp, SBI_MPHY);

	tmp = intel_sbi_read(dev_priv, 0x2108, SBI_MPHY);
	tmp |= (1 << 11);
	intel_sbi_write(dev_priv, 0x2108, tmp, SBI_MPHY);

	if (is_sdv) {
		tmp = intel_sbi_read(dev_priv, 0x2038, SBI_MPHY);
		tmp |= (0x3F << 24) | (0xF << 20) | (0xF << 16);
		intel_sbi_write(dev_priv, 0x2038, tmp, SBI_MPHY);

		tmp = intel_sbi_read(dev_priv, 0x2138, SBI_MPHY);
		tmp |= (0x3F << 24) | (0xF << 20) | (0xF << 16);
		intel_sbi_write(dev_priv, 0x2138, tmp, SBI_MPHY);

		tmp = intel_sbi_read(dev_priv, 0x203C, SBI_MPHY);
		tmp |= (0x3F << 8);
		intel_sbi_write(dev_priv, 0x203C, tmp, SBI_MPHY);

		tmp = intel_sbi_read(dev_priv, 0x213C, SBI_MPHY);
		tmp |= (0x3F << 8);
		intel_sbi_write(dev_priv, 0x213C, tmp, SBI_MPHY);
	}

	tmp = intel_sbi_read(dev_priv, 0x206C, SBI_MPHY);
	tmp |= (1 << 24) | (1 << 21) | (1 << 18);
	intel_sbi_write(dev_priv, 0x206C, tmp, SBI_MPHY);

	tmp = intel_sbi_read(dev_priv, 0x216C, SBI_MPHY);
	tmp |= (1 << 24) | (1 << 21) | (1 << 18);
	intel_sbi_write(dev_priv, 0x216C, tmp, SBI_MPHY);

	if (!is_sdv) {
		tmp = intel_sbi_read(dev_priv, 0x2080, SBI_MPHY);
		tmp &= ~(7 << 13);
		tmp |= (5 << 13);
		intel_sbi_write(dev_priv, 0x2080, tmp, SBI_MPHY);

		tmp = intel_sbi_read(dev_priv, 0x2180, SBI_MPHY);
		tmp &= ~(7 << 13);
		tmp |= (5 << 13);
		intel_sbi_write(dev_priv, 0x2180, tmp, SBI_MPHY);
	}

	tmp = intel_sbi_read(dev_priv, 0x208C, SBI_MPHY);
	tmp &= ~0xFF;
	tmp |= 0x1C;
	intel_sbi_write(dev_priv, 0x208C, tmp, SBI_MPHY);

	tmp = intel_sbi_read(dev_priv, 0x218C, SBI_MPHY);
	tmp &= ~0xFF;
	tmp |= 0x1C;
	intel_sbi_write(dev_priv, 0x218C, tmp, SBI_MPHY);

	tmp = intel_sbi_read(dev_priv, 0x2098, SBI_MPHY);
	tmp &= ~(0xFF << 16);
	tmp |= (0x1C << 16);
	intel_sbi_write(dev_priv, 0x2098, tmp, SBI_MPHY);

	tmp = intel_sbi_read(dev_priv, 0x2198, SBI_MPHY);
	tmp &= ~(0xFF << 16);
	tmp |= (0x1C << 16);
	intel_sbi_write(dev_priv, 0x2198, tmp, SBI_MPHY);

	if (!is_sdv) {
		tmp = intel_sbi_read(dev_priv, 0x20C4, SBI_MPHY);
		tmp |= (1 << 27);
		intel_sbi_write(dev_priv, 0x20C4, tmp, SBI_MPHY);

		tmp = intel_sbi_read(dev_priv, 0x21C4, SBI_MPHY);
		tmp |= (1 << 27);
		intel_sbi_write(dev_priv, 0x21C4, tmp, SBI_MPHY);

		tmp = intel_sbi_read(dev_priv, 0x20EC, SBI_MPHY);
		tmp &= ~(0xF << 28);
		tmp |= (4 << 28);
		intel_sbi_write(dev_priv, 0x20EC, tmp, SBI_MPHY);

		tmp = intel_sbi_read(dev_priv, 0x21EC, SBI_MPHY);
		tmp &= ~(0xF << 28);
		tmp |= (4 << 28);
		intel_sbi_write(dev_priv, 0x21EC, tmp, SBI_MPHY);
	}

	/* ULT uses SBI_GEN0, but ULT doesn't have VGA, so we don't care. */
	tmp = intel_sbi_read(dev_priv, SBI_DBUFF0, SBI_ICLK);
	tmp |= SBI_DBUFF0_ENABLE;
	intel_sbi_write(dev_priv, SBI_DBUFF0, tmp, SBI_ICLK);
}

/*
 * Initialize reference clocks when the driver loads
 */
void intel_init_pch_refclk(struct drm_device *dev)
{
	if (HAS_PCH_IBX(dev) || HAS_PCH_CPT(dev))
		ironlake_init_pch_refclk(dev);
	else if (HAS_PCH_LPT(dev))
		lpt_init_pch_refclk(dev);
}

static int ironlake_get_refclk(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_encoder *encoder;
	struct intel_encoder *edp_encoder = NULL;
	int num_connectors = 0;
	bool is_lvds = false;

	for_each_encoder_on_crtc(dev, crtc, encoder) {
		switch (encoder->type) {
		case INTEL_OUTPUT_LVDS:
			is_lvds = true;
			break;
		case INTEL_OUTPUT_EDP:
			edp_encoder = encoder;
			break;
		}
		num_connectors++;
	}

	if (is_lvds && intel_panel_use_ssc(dev_priv) && num_connectors < 2) {
		DRM_DEBUG_KMS("using SSC reference clock of %d MHz\n",
			      dev_priv->lvds_ssc_freq);
		return dev_priv->lvds_ssc_freq * 1000;
	}

	return 120000;
}

static int ironlake_crtc_mode_set(struct drm_crtc *crtc,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode,
				  int x, int y,
				  struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	int refclk, num_connectors = 0;
	intel_clock_t clock, reduced_clock;
	u32 dpll, fp = 0, fp2 = 0, dspcntr, pipeconf;
	bool ok, has_reduced_clock = false, is_sdvo = false;
	bool is_crt = false, is_lvds = false, is_tv = false, is_dp = false;
	struct intel_encoder *has_edp_encoder = NULL;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *encoder;
	const intel_limit_t *limit;
	int ret;
	struct fdi_m_n m_n = {0};
	u32 temp;
	u32 lvds_sync = 0;
	int target_clock, pixel_multiplier, lane, link_bw, factor;
	unsigned int pipe_bpp;
	bool dither;

	for_each_encoder_on_crtc(dev, crtc, encoder) {
		switch (encoder->type) {
		case INTEL_OUTPUT_LVDS:
			is_lvds = true;
			break;
		case INTEL_OUTPUT_SDVO:
		case INTEL_OUTPUT_HDMI:
			is_sdvo = true;
			if (encoder->needs_tv_clock)
				is_tv = true;
			break;
		case INTEL_OUTPUT_TVOUT:
			is_tv = true;
			break;
		case INTEL_OUTPUT_ANALOG:
			is_crt = true;
			break;
		case INTEL_OUTPUT_DISPLAYPORT:
			is_dp = true;
			break;
		case INTEL_OUTPUT_EDP:
			has_edp_encoder = encoder;
			break;
		}

		num_connectors++;
	}

	refclk = ironlake_get_refclk(crtc);

	/*
	 * Returns a set of divisors for the desired target clock with the given
	 * refclk, or FALSE.  The returned values represent the clock equation:
	 * reflck * (5 * (m1 + 2) + (m2 + 2)) / (n + 2) / p1 / p2.
	 */
	limit = intel_limit(crtc, refclk);
	ok = limit->find_pll(limit, crtc, adjusted_mode->clock, refclk, NULL,
			     &clock);
	if (!ok) {
		DRM_ERROR("Couldn't find PLL settings for mode!\n");
		return -EINVAL;
	}

	/* Ensure that the cursor is valid for the new mode before changing... */
	intel_crtc_update_cursor(crtc, true);

	if (is_lvds && dev_priv->lvds_downclock_avail) {
		/*
		 * Ensure we match the reduced clock's P to the target clock.
		 * If the clocks don't match, we can't switch the display clock
		 * by using the FP0/FP1. In such case we will disable the LVDS
		 * downclock feature.
		*/
		has_reduced_clock = limit->find_pll(limit, crtc,
						    dev_priv->lvds_downclock,
						    refclk,
						    &clock,
						    &reduced_clock);
	}

	if (is_sdvo && is_tv)
		i9xx_adjust_sdvo_tv_clock(adjusted_mode, &clock);


	/* FDI link */
	pixel_multiplier = intel_mode_get_pixel_multiplier(adjusted_mode);
	lane = 0;
	/* CPU eDP doesn't require FDI link, so just set DP M/N
	   according to current link config */
	if (has_edp_encoder &&
	    !intel_encoder_is_pch_edp(&has_edp_encoder->base)) {
		target_clock = mode->clock;
		intel_edp_link_config(has_edp_encoder,
				      &lane, &link_bw);
	} else {
		/* [e]DP over FDI requires target mode clock
		   instead of link clock */
		if (is_dp || intel_encoder_is_pch_edp(&has_edp_encoder->base))
			target_clock = mode->clock;
		else
			target_clock = adjusted_mode->clock;

		/* FDI is a binary signal running at ~2.7GHz, encoding
		 * each output octet as 10 bits. The actual frequency
		 * is stored as a divider into a 100MHz clock, and the
		 * mode pixel clock is stored in units of 1KHz.
		 * Hence the bw of each lane in terms of the mode signal
		 * is:
		 */
		link_bw = intel_fdi_link_freq(dev) * MHz(100)/KHz(1)/10;
	}

	/* [e]DP over FDI requires target mode clock instead of link clock. */
	if (edp_encoder)
		target_clock = intel_edp_target_clock(edp_encoder, mode);
	else if (is_dp)
		target_clock = mode->clock;
	else
		target_clock = adjusted_mode->clock;

	/* determine panel color depth */
	temp = I915_READ(PIPECONF(pipe));
	temp &= ~PIPE_BPC_MASK;
	dither = intel_choose_pipe_bpp_dither(crtc, &pipe_bpp, adjusted_mode);
	switch (pipe_bpp) {
	case 18:
		temp |= PIPE_6BPC;
		break;
	case 24:
		temp |= PIPE_8BPC;
		break;
	case 30:
		temp |= PIPE_10BPC;
		break;
	case 36:
		temp |= PIPE_12BPC;
		break;
	default:
		WARN(1, "intel_choose_pipe_bpp returned invalid value %d\n",
			pipe_bpp);
		temp |= PIPE_8BPC;
		pipe_bpp = 24;
		break;
	}

	intel_crtc->bpp = pipe_bpp;
	I915_WRITE(PIPECONF(pipe), temp);

	if (!lane) {
		/*
		 * Account for spread spectrum to avoid
		 * oversubscribing the link. Max center spread
		 * is 2.5%; use 5% for safety's sake.
		 */
		u32 bps = target_clock * intel_crtc->bpp * 21 / 20;
		lane = bps / (link_bw * 8) + 1;
	}

	intel_crtc->fdi_lanes = lane;

	if (pixel_multiplier > 1)
		link_bw *= pixel_multiplier;
	ironlake_compute_m_n(intel_crtc->bpp, lane, target_clock, link_bw,
			     &m_n);

	I915_WRITE(PIPE_DATA_M1(cpu_transcoder), TU_SIZE(m_n.tu) | m_n.gmch_m);
	I915_WRITE(PIPE_DATA_N1(cpu_transcoder), m_n.gmch_n);
	I915_WRITE(PIPE_LINK_M1(cpu_transcoder), m_n.link_m);
	I915_WRITE(PIPE_LINK_N1(cpu_transcoder), m_n.link_n);
}

static uint32_t ironlake_compute_dpll(struct intel_crtc *intel_crtc,
				      struct drm_display_mode *adjusted_mode,
				      intel_clock_t *clock, u32 fp)
{
	struct drm_crtc *crtc = &intel_crtc->base;
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_encoder *intel_encoder;
	uint32_t dpll;
	int factor, pixel_multiplier, num_connectors = 0;
	bool is_lvds = false, is_sdvo = false, is_tv = false;
	bool is_dp = false, is_cpu_edp = false;

	for_each_encoder_on_crtc(dev, crtc, intel_encoder) {
		switch (intel_encoder->type) {
		case INTEL_OUTPUT_LVDS:
			is_lvds = true;
			break;
		case INTEL_OUTPUT_SDVO:
		case INTEL_OUTPUT_HDMI:
			is_sdvo = true;
			if (intel_encoder->needs_tv_clock)
				is_tv = true;
			break;
		case INTEL_OUTPUT_TVOUT:
			is_tv = true;
			break;
		}

		num_connectors++;
	}

	/* Enable autotuning of the PLL clock (if permissible) */
	factor = 21;
	if (is_lvds) {
		if ((intel_panel_use_ssc(dev_priv) &&
		     dev_priv->lvds_ssc_freq == 100) ||
		    (I915_READ(PCH_LVDS) & LVDS_CLKB_POWER_MASK) == LVDS_CLKB_POWER_UP)
			factor = 25;
	} else if (is_sdvo && is_tv)
		factor = 20;

	if (clock->m < factor * clock->n)
		fp |= FP_CB_TUNE;

	dpll = 0;

	if (is_lvds)
		dpll |= DPLLB_MODE_LVDS;
	else
		dpll |= DPLLB_MODE_DAC_SERIAL;
	if (is_sdvo) {
		pixel_multiplier = intel_mode_get_pixel_multiplier(adjusted_mode);
		if (pixel_multiplier > 1) {
			dpll |= (pixel_multiplier - 1) << PLL_REF_SDVO_HDMI_MULTIPLIER_SHIFT;
		}
		dpll |= DPLL_DVO_HIGH_SPEED;
	}
	if (is_dp || intel_encoder_is_pch_edp(&has_edp_encoder->base))
		dpll |= DPLL_DVO_HIGH_SPEED;

	/* compute bitmask from p1 value */
	dpll |= (1 << (clock->p1 - 1)) << DPLL_FPA01_P1_POST_DIV_SHIFT;
	/* also FPA1 */
	dpll |= (1 << (clock->p1 - 1)) << DPLL_FPA1_P1_POST_DIV_SHIFT;

	switch (clock->p2) {
	case 5:
		dpll |= DPLL_DAC_SERIAL_P2_CLOCK_DIV_5;
		break;
	case 7:
		dpll |= DPLLB_LVDS_P2_CLOCK_DIV_7;
		break;
	case 10:
		dpll |= DPLL_DAC_SERIAL_P2_CLOCK_DIV_10;
		break;
	case 14:
		dpll |= DPLLB_LVDS_P2_CLOCK_DIV_14;
		break;
	}

	if (is_sdvo && is_tv)
		dpll |= PLL_REF_INPUT_TVCLKINBC;
	else if (is_tv)
		/* XXX: just matching BIOS for now */
		/*	dpll |= PLL_REF_INPUT_TVCLKINBC; */
		dpll |= 3;
	else if (is_lvds && intel_panel_use_ssc(dev_priv) && num_connectors < 2)
		dpll |= PLLB_REF_INPUT_SPREADSPECTRUMIN;
	else
		dpll |= PLL_REF_INPUT_DREFCLK;

	/* setup pipeconf */
	pipeconf = I915_READ(PIPECONF(pipe));

	/* Set up the display plane register */
	dspcntr = DISPPLANE_GAMMA_ENABLE;

	DRM_DEBUG_KMS("Mode for pipe %d:\n", pipe);
	drm_mode_debug_printmodeline(mode);

	/* PCH eDP needs FDI, but CPU eDP does not */
	if (!intel_crtc->no_pll) {
		if (!has_edp_encoder ||
		    intel_encoder_is_pch_edp(&has_edp_encoder->base)) {
			I915_WRITE(PCH_FP0(pipe), fp);
			I915_WRITE(PCH_DPLL(pipe), dpll & ~DPLL_VCO_ENABLE);

			POSTING_READ(PCH_DPLL(pipe));
			udelay(150);
		}
	} else {
		if (dpll == (I915_READ(PCH_DPLL(0)) & 0x7fffffff) &&
		    fp == I915_READ(PCH_FP0(0))) {
			intel_crtc->use_pll_a = true;
			DRM_DEBUG_KMS("using pipe a dpll\n");
		} else if (dpll == (I915_READ(PCH_DPLL(1)) & 0x7fffffff) &&
			   fp == I915_READ(PCH_FP0(1))) {
			intel_crtc->use_pll_a = false;
			DRM_DEBUG_KMS("using pipe b dpll\n");
		} else {
			DRM_DEBUG_KMS("no matching PLL configuration for pipe 2\n");
			return -EINVAL;
		}
	}

	/* The LVDS pin pair needs to be on before the DPLLs are enabled.
	 * This is an exception to the general rule that mode_set doesn't turn
	 * things on.
	 */
	if (is_lvds) {
		temp = I915_READ(PCH_LVDS);
		temp |= LVDS_PORT_EN | LVDS_A0A2_CLKA_POWER_UP;
		if (HAS_PCH_CPT(dev)) {
			temp &= ~PORT_TRANS_SEL_MASK;
			temp |= PORT_TRANS_SEL_CPT(pipe);
		} else {
			if (pipe == 1)
				temp |= LVDS_PIPEB_SELECT;
			else
				temp &= ~LVDS_PIPEB_SELECT;
		}

		/* set the corresponsding LVDS_BORDER bit */
		temp |= dev_priv->lvds_border_bits;
		/* Set the B0-B3 data pairs corresponding to whether we're going to
		 * set the DPLLs for dual-channel mode or not.
		 */
		if (clock.p2 == 7)
			temp |= LVDS_B0B3_POWER_UP | LVDS_CLKB_POWER_UP;
		else
			temp &= ~(LVDS_B0B3_POWER_UP | LVDS_CLKB_POWER_UP);

		/* It would be nice to set 24 vs 18-bit mode (LVDS_A3_POWER_UP)
		 * appropriately here, but we need to look more thoroughly into how
		 * panels behave in the two modes.
		 */
		if (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC)
			lvds_sync |= LVDS_HSYNC_POLARITY;
		if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
			lvds_sync |= LVDS_VSYNC_POLARITY;
		if ((temp & (LVDS_HSYNC_POLARITY | LVDS_VSYNC_POLARITY))
		    != lvds_sync) {
			char flags[2] = "-+";
			DRM_INFO("Changing LVDS panel from "
				 "(%chsync, %cvsync) to (%chsync, %cvsync)\n",
				 flags[!(temp & LVDS_HSYNC_POLARITY)],
				 flags[!(temp & LVDS_VSYNC_POLARITY)],
				 flags[!(lvds_sync & LVDS_HSYNC_POLARITY)],
				 flags[!(lvds_sync & LVDS_VSYNC_POLARITY)]);
			temp &= ~(LVDS_HSYNC_POLARITY | LVDS_VSYNC_POLARITY);
			temp |= lvds_sync;
		}
		I915_WRITE(PCH_LVDS, temp);
	}

	pipeconf &= ~PIPECONF_DITHER_EN;
	pipeconf &= ~PIPECONF_DITHER_TYPE_MASK;
	if ((is_lvds && dev_priv->lvds_dither) || dither) {
		pipeconf |= PIPECONF_DITHER_EN;
		pipeconf |= PIPECONF_DITHER_TYPE_SP;
	}
	if (is_dp || intel_encoder_is_pch_edp(&has_edp_encoder->base)) {
		intel_dp_set_m_n(crtc, mode, adjusted_mode);
	} else {
		/* For non-DP output, clear any trans DP clock recovery setting.*/
		I915_WRITE(TRANSDATA_M1(pipe), 0);
		I915_WRITE(TRANSDATA_N1(pipe), 0);
		I915_WRITE(TRANSDPLINK_M1(pipe), 0);
		I915_WRITE(TRANSDPLINK_N1(pipe), 0);
	}

	if (!intel_crtc->no_pll &&
	    (!has_edp_encoder ||
	     intel_encoder_is_pch_edp(&has_edp_encoder->base))) {
		I915_WRITE(PCH_DPLL(pipe), dpll);

		/* Wait for the clocks to stabilize. */
		POSTING_READ(PCH_DPLL(pipe));
		udelay(150);

		/* The pixel multiplier can only be updated once the
		 * DPLL is enabled and the clocks are stable.
		 *
		 * So write it again.
		 */
		I915_WRITE(PCH_DPLL(pipe), dpll);
	}

	intel_crtc->lowfreq_avail = false;
	if (!intel_crtc->no_pll) {
		if (is_lvds && has_reduced_clock && i915_powersave) {
			I915_WRITE(PCH_FP1(pipe), fp2);
			intel_crtc->lowfreq_avail = true;
			if (HAS_PIPE_CXSR(dev)) {
				DRM_DEBUG_KMS("enabling CxSR downclocking\n");
				pipeconf |= PIPECONF_CXSR_DOWNCLOCK;
			}
		} else {
			I915_WRITE(PCH_FP1(pipe), fp);
			if (HAS_PIPE_CXSR(dev)) {
				DRM_DEBUG_KMS("disabling CxSR downclocking\n");
				pipeconf &= ~PIPECONF_CXSR_DOWNCLOCK;
			}
		}
	}

	pipeconf &= ~PIPECONF_INTERLACE_MASK;
	if (adjusted_mode->flags & DRM_MODE_FLAG_INTERLACE) {
		pipeconf |= PIPECONF_INTERLACED_ILK;
		/* the chip adds 2 halflines automatically */
		adjusted_mode->crtc_vtotal -= 1;
		adjusted_mode->crtc_vblank_end -= 1;
		I915_WRITE(VSYNCSHIFT(pipe),
			   adjusted_mode->crtc_hsync_start
			   - adjusted_mode->crtc_htotal/2);
	} else {
		pipeconf |= PIPECONF_PROGRESSIVE;
		I915_WRITE(VSYNCSHIFT(pipe), 0);
	}

	I915_WRITE(HTOTAL(pipe),
		   (adjusted_mode->crtc_hdisplay - 1) |
		   ((adjusted_mode->crtc_htotal - 1) << 16));
	I915_WRITE(HBLANK(pipe),
		   (adjusted_mode->crtc_hblank_start - 1) |
		   ((adjusted_mode->crtc_hblank_end - 1) << 16));
	I915_WRITE(HSYNC(pipe),
		   (adjusted_mode->crtc_hsync_start - 1) |
		   ((adjusted_mode->crtc_hsync_end - 1) << 16));

	I915_WRITE(VTOTAL(pipe),
		   (adjusted_mode->crtc_vdisplay - 1) |
		   ((adjusted_mode->crtc_vtotal - 1) << 16));
	I915_WRITE(VBLANK(pipe),
		   (adjusted_mode->crtc_vblank_start - 1) |
		   ((adjusted_mode->crtc_vblank_end - 1) << 16));
	I915_WRITE(VSYNC(pipe),
		   (adjusted_mode->crtc_vsync_start - 1) |
		   ((adjusted_mode->crtc_vsync_end - 1) << 16));

	/* pipesrc controls the size that is scaled from, which should
	 * always be the user's requested size.
	 */
	I915_WRITE(PIPESRC(pipe),
		   ((mode->hdisplay - 1) << 16) | (mode->vdisplay - 1));

	I915_WRITE(PIPE_DATA_M1(pipe), TU_SIZE(m_n.tu) | m_n.gmch_m);
	I915_WRITE(PIPE_DATA_N1(pipe), m_n.gmch_n);
	I915_WRITE(PIPE_LINK_M1(pipe), m_n.link_m);
	I915_WRITE(PIPE_LINK_N1(pipe), m_n.link_n);

	if (has_edp_encoder &&
	    !intel_encoder_is_pch_edp(&has_edp_encoder->base)) {
		ironlake_set_pll_edp(crtc, adjusted_mode->clock);
	}

	I915_WRITE(PIPECONF(pipe), pipeconf);
	POSTING_READ(PIPECONF(pipe));

	intel_wait_for_vblank(dev, pipe);

	I915_WRITE(DSPCNTR(plane), dspcntr);
	POSTING_READ(DSPCNTR(plane));

	ret = intel_pipe_set_base(crtc, x, y, fb);

	intel_update_watermarks(dev);

	return fdi_config_ok ? ret : -EINVAL;
}

static int haswell_crtc_mode_set(struct drm_crtc *crtc,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode,
				 int x, int y,
				 struct drm_framebuffer *fb)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	int num_connectors = 0;
	intel_clock_t clock, reduced_clock;
	u32 dpll = 0, fp = 0, fp2 = 0;
	bool ok, has_reduced_clock = false;
	bool is_lvds = false, is_dp = false, is_cpu_edp = false;
	struct intel_encoder *encoder;
	u32 temp;
	int ret;
	bool dither;

	for_each_encoder_on_crtc(dev, crtc, encoder) {
		switch (encoder->type) {
		case INTEL_OUTPUT_LVDS:
			is_lvds = true;
			break;
		}

		num_connectors++;
	}

	if (is_cpu_edp)
		intel_crtc->config.cpu_transcoder = TRANSCODER_EDP;
	else
		intel_crtc->config.cpu_transcoder = pipe;

	/* We are not sure yet this won't happen. */
	WARN(!HAS_PCH_LPT(dev), "Unexpected PCH type %d\n",
	     INTEL_PCH_TYPE(dev));

	WARN(num_connectors != 1, "%d connectors attached to pipe %c\n",
	     num_connectors, pipe_name(pipe));

	WARN_ON(I915_READ(PIPECONF(intel_crtc->config.cpu_transcoder)) &
		(PIPECONF_ENABLE | I965_PIPECONF_ACTIVE));

	WARN_ON(I915_READ(DSPCNTR(plane)) & DISPLAY_PLANE_ENABLE);

	if (!intel_ddi_pll_mode_set(crtc, adjusted_mode->clock))
		return -EINVAL;

	if (HAS_PCH_IBX(dev) || HAS_PCH_CPT(dev)) {
		ok = ironlake_compute_clocks(crtc, adjusted_mode, &clock,
					     &has_reduced_clock,
					     &reduced_clock);
		if (!ok) {
			DRM_ERROR("Couldn't find PLL settings for mode!\n");
			return -EINVAL;
		}
	}

	/* Ensure that the cursor is valid for the new mode before changing... */
	intel_crtc_update_cursor(crtc, true);

	/* determine panel color depth */
	dither = intel_choose_pipe_bpp_dither(crtc, fb, &intel_crtc->bpp,
					      adjusted_mode);
	if (is_lvds && dev_priv->lvds_dither)
		dither = true;

	DRM_DEBUG_KMS("Mode for pipe %d:\n", pipe);
	drm_mode_debug_printmodeline(mode);

	if (HAS_PCH_IBX(dev) || HAS_PCH_CPT(dev)) {
		fp = clock.n << 16 | clock.m1 << 8 | clock.m2;
		if (has_reduced_clock)
			fp2 = reduced_clock.n << 16 | reduced_clock.m1 << 8 |
			      reduced_clock.m2;

		dpll = ironlake_compute_dpll(intel_crtc, adjusted_mode, &clock,
					     fp);

		/* CPU eDP is the only output that doesn't need a PCH PLL of its
		 * own on pre-Haswell/LPT generation */
		if (!is_cpu_edp) {
			struct intel_pch_pll *pll;

			pll = intel_get_pch_pll(intel_crtc, dpll, fp);
			if (pll == NULL) {
				DRM_DEBUG_DRIVER("failed to find PLL for pipe %d\n",
						 pipe);
				return -EINVAL;
			}
		} else
			intel_put_pch_pll(intel_crtc);

		/* The LVDS pin pair needs to be on before the DPLLs are
		 * enabled.  This is an exception to the general rule that
		 * mode_set doesn't turn things on.
		 */
		if (is_lvds) {
			temp = I915_READ(PCH_LVDS);
			temp |= LVDS_PORT_EN | LVDS_A0A2_CLKA_POWER_UP;
			if (HAS_PCH_CPT(dev)) {
				temp &= ~PORT_TRANS_SEL_MASK;
				temp |= PORT_TRANS_SEL_CPT(pipe);
			} else {
				if (pipe == 1)
					temp |= LVDS_PIPEB_SELECT;
				else
					temp &= ~LVDS_PIPEB_SELECT;
			}

			/* set the corresponsding LVDS_BORDER bit */
			temp |= dev_priv->lvds_border_bits;
			/* Set the B0-B3 data pairs corresponding to whether
			 * we're going to set the DPLLs for dual-channel mode or
			 * not.
			 */
			if (clock.p2 == 7)
				temp |= LVDS_B0B3_POWER_UP | LVDS_CLKB_POWER_UP;
			else
				temp &= ~(LVDS_B0B3_POWER_UP |
					  LVDS_CLKB_POWER_UP);

			/* It would be nice to set 24 vs 18-bit mode
			 * (LVDS_A3_POWER_UP) appropriately here, but we need to
			 * look more thoroughly into how panels behave in the
			 * two modes.
			 */
			temp &= ~(LVDS_HSYNC_POLARITY | LVDS_VSYNC_POLARITY);
			if (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC)
				temp |= LVDS_HSYNC_POLARITY;
			if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
				temp |= LVDS_VSYNC_POLARITY;
			I915_WRITE(PCH_LVDS, temp);
		}
	}

	if (is_dp && !is_cpu_edp) {
		intel_dp_set_m_n(crtc, mode, adjusted_mode);
	} else {
		if (HAS_PCH_IBX(dev) || HAS_PCH_CPT(dev)) {
			/* For non-DP output, clear any trans DP clock recovery
			 * setting.*/
			I915_WRITE(TRANSDATA_M1(pipe), 0);
			I915_WRITE(TRANSDATA_N1(pipe), 0);
			I915_WRITE(TRANSDPLINK_M1(pipe), 0);
			I915_WRITE(TRANSDPLINK_N1(pipe), 0);
		}
	}

	intel_crtc->lowfreq_avail = false;
	if (HAS_PCH_IBX(dev) || HAS_PCH_CPT(dev)) {
		if (intel_crtc->pch_pll) {
			I915_WRITE(intel_crtc->pch_pll->pll_reg, dpll);

			/* Wait for the clocks to stabilize. */
			POSTING_READ(intel_crtc->pch_pll->pll_reg);
			udelay(150);

			/* The pixel multiplier can only be updated once the
			 * DPLL is enabled and the clocks are stable.
			 *
			 * So write it again.
			 */
			I915_WRITE(intel_crtc->pch_pll->pll_reg, dpll);
		}

		if (intel_crtc->pch_pll) {
			if (is_lvds && has_reduced_clock && i915_powersave) {
				I915_WRITE(intel_crtc->pch_pll->fp1_reg, fp2);
				intel_crtc->lowfreq_avail = true;
			} else {
				I915_WRITE(intel_crtc->pch_pll->fp1_reg, fp);
			}
		}
	}

	intel_set_pipe_timings(intel_crtc, mode, adjusted_mode);

	if (!is_dp || is_cpu_edp)
		ironlake_set_m_n(crtc, mode, adjusted_mode);

	if (HAS_PCH_IBX(dev) || HAS_PCH_CPT(dev))
		if (is_cpu_edp)
			ironlake_set_pll_edp(crtc, adjusted_mode->clock);

	haswell_set_pipeconf(crtc, adjusted_mode, dither);

	/* Set up the display plane register */
	I915_WRITE(DSPCNTR(plane), DISPPLANE_GAMMA_ENABLE);
	POSTING_READ(DSPCNTR(plane));

	ret = intel_pipe_set_base(crtc, x, y, fb);

	intel_update_watermarks(dev);

	intel_update_linetime_watermarks(dev, pipe, adjusted_mode);

	return ret;
}

static bool haswell_get_pipe_config(struct intel_crtc *crtc,
				    struct intel_crtc_config *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t tmp;

	tmp = I915_READ(PIPECONF(crtc->config.cpu_transcoder));
	if (!(tmp & PIPECONF_ENABLE))
		return false;

	/*
	 * aswell has only FDI/PCH transcoder A. It is which is connected to
	 * DDI E. So just check whether this pipe is wired to DDI E and whether
	 * the PCH transcoder is on.
	 */
	tmp = I915_READ(TRANS_DDI_FUNC_CTL(crtc->pipe));
	if ((tmp & TRANS_DDI_PORT_MASK) == TRANS_DDI_SELECT_PORT(PORT_E) &&
	    I915_READ(TRANSCONF(PIPE_A)) & TRANS_ENABLE)
		pipe_config->has_pch_encoder = true;


	return true;
}

static int intel_crtc_mode_set(struct drm_crtc *crtc,
			       int x, int y,
			       struct drm_framebuffer *fb)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_encoder_helper_funcs *encoder_funcs;
	struct intel_encoder *encoder;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct drm_display_mode *adjusted_mode =
		&intel_crtc->config.adjusted_mode;
	struct drm_display_mode *mode = &intel_crtc->config.requested_mode;
	int pipe = intel_crtc->pipe;
	int ret;

	drm_vblank_pre_modeset(dev, pipe);

	ret = dev_priv->display.crtc_mode_set(crtc, x, y, fb);

	drm_vblank_post_modeset(dev, pipe);

	if (ret != 0)
		return ret;

	for_each_encoder_on_crtc(dev, crtc, encoder) {
		DRM_DEBUG_KMS("[ENCODER:%d:%s] set [MODE:%d:%s]\n",
			encoder->base.base.id,
			drm_get_encoder_name(&encoder->base),
			mode->base.id, mode->name);
		if (encoder->mode_set) {
			encoder->mode_set(encoder);
		} else {
			encoder_funcs = encoder->base.helper_private;
			encoder_funcs->mode_set(&encoder->base, mode, adjusted_mode);
		}
	}

	return 0;
}

static bool intel_eld_uptodate(struct drm_connector *connector,
			       int reg_eldv, uint32_t bits_eldv,
			       int reg_elda, uint32_t bits_elda,
			       int reg_edid)
{
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	uint8_t *eld = connector->eld;
	uint32_t i;

	i = I915_READ(reg_eldv);
	i &= bits_eldv;

	if (!eld[0])
		return !i;

	if (!i)
		return false;

	i = I915_READ(reg_elda);
	i &= ~bits_elda;
	I915_WRITE(reg_elda, i);

	for (i = 0; i < eld[2]; i++)
		if (I915_READ(reg_edid) != *((uint32_t *)eld + i))
			return false;

	return true;
}

static void g4x_write_eld(struct drm_connector *connector,
			  struct drm_crtc *crtc)
{
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	uint8_t *eld = connector->eld;
	uint32_t eldv;
	uint32_t len;
	uint32_t i;

	i = I915_READ(G4X_AUD_VID_DID);

	if (i == INTEL_AUDIO_DEVBLC || i == INTEL_AUDIO_DEVCL)
		eldv = G4X_ELDV_DEVCL_DEVBLC;
	else
		eldv = G4X_ELDV_DEVCTG;

	if (intel_eld_uptodate(connector,
			       G4X_AUD_CNTL_ST, eldv,
			       G4X_AUD_CNTL_ST, G4X_ELD_ADDR,
			       G4X_HDMIW_HDMIEDID))
		return;

	i = I915_READ(G4X_AUD_CNTL_ST);
	i &= ~(eldv | G4X_ELD_ADDR);
	len = (i >> 9) & 0x1f;		/* ELD buffer size */
	I915_WRITE(G4X_AUD_CNTL_ST, i);

	if (!eld[0])
		return;

	len = min_t(uint8_t, eld[2], len);
	DRM_DEBUG_DRIVER("ELD size %d\n", len);
	for (i = 0; i < len; i++)
		I915_WRITE(G4X_HDMIW_HDMIEDID, *((uint32_t *)eld + i));

	i = I915_READ(G4X_AUD_CNTL_ST);
	i |= eldv;
	I915_WRITE(G4X_AUD_CNTL_ST, i);
}

static void haswell_write_eld(struct drm_connector *connector,
				     struct drm_crtc *crtc)
{
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	uint8_t *eld = connector->eld;
	struct drm_device *dev = crtc->dev;
	uint32_t eldv;
	uint32_t i;
	int len;
	int pipe = to_intel_crtc(crtc)->pipe;
	int tmp;

	int hdmiw_hdmiedid = HSW_AUD_EDID_DATA(pipe);
	int aud_cntl_st = HSW_AUD_DIP_ELD_CTRL(pipe);
	int aud_config = HSW_AUD_CFG(pipe);
	int aud_cntrl_st2 = HSW_AUD_PIN_ELD_CP_VLD;


	DRM_DEBUG_DRIVER("HDMI: Haswell Audio initialize....\n");

	/* Audio output enable */
	DRM_DEBUG_DRIVER("HDMI audio: enable codec\n");
	tmp = I915_READ(aud_cntrl_st2);
	tmp |= (AUDIO_OUTPUT_ENABLE_A << (pipe * 4));
	I915_WRITE(aud_cntrl_st2, tmp);

	/* Wait for 1 vertical blank */
	intel_wait_for_vblank(dev, pipe);

	/* Set ELD valid state */
	tmp = I915_READ(aud_cntrl_st2);
	DRM_DEBUG_DRIVER("HDMI audio: pin eld vld status=0x%8x\n", tmp);
	tmp |= (AUDIO_ELD_VALID_A << (pipe * 4));
	I915_WRITE(aud_cntrl_st2, tmp);
	tmp = I915_READ(aud_cntrl_st2);
	DRM_DEBUG_DRIVER("HDMI audio: eld vld status=0x%8x\n", tmp);

	/* Enable HDMI mode */
	tmp = I915_READ(aud_config);
	DRM_DEBUG_DRIVER("HDMI audio: audio conf: 0x%8x\n", tmp);
	/* clear N_programing_enable and N_value_index */
	tmp &= ~(AUD_CONFIG_N_VALUE_INDEX | AUD_CONFIG_N_PROG_ENABLE);
	I915_WRITE(aud_config, tmp);

	DRM_DEBUG_DRIVER("ELD on pipe %c\n", pipe_name(pipe));

	eldv = AUDIO_ELD_VALID_A << (pipe * 4);

	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_DISPLAYPORT)) {
		DRM_DEBUG_DRIVER("ELD: DisplayPort detected\n");
		eld[5] |= (1 << 2);	/* Conn_Type, 0x1 = DisplayPort */
		I915_WRITE(aud_config, AUD_CONFIG_N_VALUE_INDEX); /* 0x1 = DP */
	} else
		I915_WRITE(aud_config, 0);

	if (intel_eld_uptodate(connector,
			       aud_cntrl_st2, eldv,
			       aud_cntl_st, IBX_ELD_ADDRESS,
			       hdmiw_hdmiedid))
		return;

	i = I915_READ(aud_cntrl_st2);
	i &= ~eldv;
	I915_WRITE(aud_cntrl_st2, i);

	if (!eld[0])
		return;

	i = I915_READ(aud_cntl_st);
	i &= ~IBX_ELD_ADDRESS;
	I915_WRITE(aud_cntl_st, i);
	i = (i >> 29) & DIP_PORT_SEL_MASK;		/* DIP_Port_Select, 0x1 = PortB */
	DRM_DEBUG_DRIVER("port num:%d\n", i);

	len = min_t(uint8_t, eld[2], 21);	/* 84 bytes of hw ELD buffer */
	DRM_DEBUG_DRIVER("ELD size %d\n", len);
	for (i = 0; i < len; i++)
		I915_WRITE(hdmiw_hdmiedid, *((uint32_t *)eld + i));

	i = I915_READ(aud_cntrl_st2);
	i |= eldv;
	I915_WRITE(aud_cntrl_st2, i);

}

static void ironlake_write_eld(struct drm_connector *connector,
				     struct drm_crtc *crtc)
{
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	uint8_t *eld = connector->eld;
	uint32_t eldv;
	uint32_t i;
	int len;
	int hdmiw_hdmiedid;
	int aud_config;
	int aud_cntl_st;
	int aud_cntrl_st2;
	int pipe = to_intel_crtc(crtc)->pipe;

	if (HAS_PCH_IBX(connector->dev)) {
		hdmiw_hdmiedid = IBX_HDMIW_HDMIEDID(pipe);
		aud_config = IBX_AUD_CFG(pipe);
		aud_cntl_st = IBX_AUD_CNTL_ST(pipe);
		aud_cntrl_st2 = IBX_AUD_CNTL_ST2;
	} else {
		hdmiw_hdmiedid = CPT_HDMIW_HDMIEDID(pipe);
		aud_config = CPT_AUD_CFG(pipe);
		aud_cntl_st = CPT_AUD_CNTL_ST(pipe);
		aud_cntrl_st2 = CPT_AUD_CNTRL_ST2;
	}

	DRM_DEBUG_DRIVER("ELD on pipe %c\n", pipe_name(pipe));

	i = I915_READ(aud_cntl_st);
	i = (i >> 29) & DIP_PORT_SEL_MASK;		/* DIP_Port_Select, 0x1 = PortB */
	if (!i) {
		DRM_DEBUG_DRIVER("Audio directed to unknown port\n");
		/* operate blindly on all ports */
		eldv = IBX_ELD_VALIDB;
		eldv |= IBX_ELD_VALIDB << 4;
		eldv |= IBX_ELD_VALIDB << 8;
	} else {
		DRM_DEBUG_DRIVER("ELD on port %c\n", 'A' + i);
		eldv = IBX_ELD_VALIDB << ((i - 1) * 4);
	}

	if (intel_pipe_has_type(crtc, INTEL_OUTPUT_DISPLAYPORT)) {
		DRM_DEBUG_DRIVER("ELD: DisplayPort detected\n");
		eld[5] |= (1 << 2);	/* Conn_Type, 0x1 = DisplayPort */
		I915_WRITE(aud_config, AUD_CONFIG_N_VALUE_INDEX); /* 0x1 = DP */
	} else
		I915_WRITE(aud_config, 0);

	if (intel_eld_uptodate(connector,
			       aud_cntrl_st2, eldv,
			       aud_cntl_st, IBX_ELD_ADDRESS,
			       hdmiw_hdmiedid))
		return;

	i = I915_READ(aud_cntrl_st2);
	i &= ~eldv;
	I915_WRITE(aud_cntrl_st2, i);

	if (!eld[0])
		return;

	i = I915_READ(aud_cntl_st);
	i &= ~IBX_ELD_ADDRESS;
	I915_WRITE(aud_cntl_st, i);

	len = min_t(uint8_t, eld[2], 21);	/* 84 bytes of hw ELD buffer */
	DRM_DEBUG_DRIVER("ELD size %d\n", len);
	for (i = 0; i < len; i++)
		I915_WRITE(hdmiw_hdmiedid, *((uint32_t *)eld + i));

	i = I915_READ(aud_cntrl_st2);
	i |= eldv;
	I915_WRITE(aud_cntrl_st2, i);
}

void intel_write_eld(struct drm_encoder *encoder,
		     struct drm_display_mode *mode)
{
	struct drm_crtc *crtc = encoder->crtc;
	struct drm_connector *connector;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	connector = drm_select_eld(encoder, mode);
	if (!connector)
		return;

	DRM_DEBUG_DRIVER("ELD on [CONNECTOR:%d:%s], [ENCODER:%d:%s]\n",
			 connector->base.id,
			 drm_get_connector_name(connector),
			 connector->encoder->base.id,
			 drm_get_encoder_name(connector->encoder));

	connector->eld[6] = drm_av_sync_delay(connector, mode) / 2;

	if (dev_priv->display.write_eld)
		dev_priv->display.write_eld(connector, crtc);
}

/** Loads the palette/gamma unit for the CRTC with the prepared values */
void intel_crtc_load_lut(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int palreg = PALETTE(intel_crtc->pipe);
	int i;

	/* The clocks have to be on to load the palette. */
	if (!crtc->enabled || !intel_crtc->active)
		return;

	/* use legacy palette for Ironlake */
	if (HAS_PCH_SPLIT(dev))
		palreg = LGC_PALETTE(intel_crtc->pipe);

	for (i = 0; i < 256; i++) {
		I915_WRITE(palreg + 4 * i,
			   (intel_crtc->lut_r[i] << 16) |
			   (intel_crtc->lut_g[i] << 8) |
			   intel_crtc->lut_b[i]);
	}
}

static void i845_update_cursor(struct drm_crtc *crtc, u32 base)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	bool visible = base != 0;
	u32 cntl;

	if (intel_crtc->cursor_visible == visible)
		return;

	cntl = I915_READ(_CURACNTR);
	if (visible) {
		/* On these chipsets we can only modify the base whilst
		 * the cursor is disabled.
		 */
		I915_WRITE(_CURABASE, base);

		cntl &= ~(CURSOR_FORMAT_MASK);
		/* XXX width must be 64, stride 256 => 0x00 << 28 */
		cntl |= CURSOR_ENABLE |
			CURSOR_GAMMA_ENABLE |
			CURSOR_FORMAT_ARGB;
	} else
		cntl &= ~(CURSOR_ENABLE | CURSOR_GAMMA_ENABLE);
	I915_WRITE(_CURACNTR, cntl);

	intel_crtc->cursor_visible = visible;
}

static void i9xx_update_cursor(struct drm_crtc *crtc, u32 base)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	bool visible = base != 0;

	if (intel_crtc->cursor_visible != visible) {
		uint32_t cntl = I915_READ(CURCNTR(pipe));
		if (base) {
			cntl &= ~(CURSOR_MODE | MCURSOR_PIPE_SELECT);
			cntl |= CURSOR_MODE_64_ARGB_AX | MCURSOR_GAMMA_ENABLE;
			cntl |= pipe << 28; /* Connect to correct pipe */
		} else {
			cntl &= ~(CURSOR_MODE | MCURSOR_GAMMA_ENABLE);
			cntl |= CURSOR_MODE_DISABLE;
		}
		I915_WRITE(CURCNTR(pipe), cntl);

		intel_crtc->cursor_visible = visible;
	}
	/* and commit changes on next vblank */
	POSTING_READ(CURCNTR(pipe));
	I915_WRITE(CURBASE(pipe), base);
	POSTING_READ(CURBASE(pipe));
}

static void ivb_update_cursor(struct drm_crtc *crtc, u32 base)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	bool visible = base != 0;

	if (intel_crtc->cursor_visible != visible) {
		uint32_t cntl = I915_READ(CURCNTR_IVB(pipe));
		if (base) {
			cntl &= ~CURSOR_MODE;
			cntl |= CURSOR_MODE_64_ARGB_AX | MCURSOR_GAMMA_ENABLE;
		} else {
			cntl &= ~(CURSOR_MODE | MCURSOR_GAMMA_ENABLE);
			cntl |= CURSOR_MODE_DISABLE;
		}
		I915_WRITE(CURCNTR_IVB(pipe), cntl);

		intel_crtc->cursor_visible = visible;
	}
	/* and commit changes on next vblank */
	POSTING_READ(CURCNTR_IVB(pipe));
	I915_WRITE(CURBASE_IVB(pipe), base);
	POSTING_READ(CURBASE_IVB(pipe));
}

/* If no-part of the cursor is visible on the framebuffer, then the GPU may hang... */
static void intel_crtc_update_cursor(struct drm_crtc *crtc,
				     bool on)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	int x = intel_crtc->cursor_x;
	int y = intel_crtc->cursor_y;
	u32 base, pos;
	bool visible;

	pos = 0;

	if (on && crtc->enabled && crtc->fb) {
		base = intel_crtc->cursor_addr;
		if (x > (int) crtc->fb->width)
			base = 0;

		if (y > (int) crtc->fb->height)
			base = 0;
	} else
		base = 0;

	if (x < 0) {
		if (x + intel_crtc->cursor_width < 0)
			base = 0;

		pos |= CURSOR_POS_SIGN << CURSOR_X_SHIFT;
		x = -x;
	}
	pos |= x << CURSOR_X_SHIFT;

	if (y < 0) {
		if (y + intel_crtc->cursor_height < 0)
			base = 0;

		pos |= CURSOR_POS_SIGN << CURSOR_Y_SHIFT;
		y = -y;
	}
	pos |= y << CURSOR_Y_SHIFT;

	visible = base != 0;
	if (!visible && !intel_crtc->cursor_visible)
		return;

	if (IS_IVYBRIDGE(dev)) {
		I915_WRITE(CURPOS_IVB(pipe), pos);
		ivb_update_cursor(crtc, base);
	} else {
		I915_WRITE(CURPOS(pipe), pos);
		if (IS_845G(dev) || IS_I865G(dev))
			i845_update_cursor(crtc, base);
		else
			i9xx_update_cursor(crtc, base);
	}

	if (visible)
		intel_mark_busy(dev, to_intel_framebuffer(crtc->fb)->obj);
}

static int intel_crtc_cursor_set(struct drm_crtc *crtc,
				 struct drm_file *file,
				 uint32_t handle,
				 uint32_t width, uint32_t height)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct drm_i915_gem_object *obj;
	uint32_t addr;
	int ret;

	/* if we want to turn off the cursor ignore width and height */
	if (!handle) {
		DRM_DEBUG_KMS("cursor off\n");
		addr = 0;
		obj = NULL;
		mutex_lock(&dev->struct_mutex);
		goto finish;
	}

	/* Currently we only support 64x64 cursors */
	if (width != 64 || height != 64) {
		DRM_ERROR("we currently only support 64x64 cursors\n");
		return -EINVAL;
	}

	obj = to_intel_bo(drm_gem_object_lookup(dev, file, handle));
	if (&obj->base == NULL)
		return -ENOENT;

	if (obj->base.size < width * height * 4) {
		DRM_ERROR("buffer is to small\n");
		ret = -ENOMEM;
		goto fail;
	}

	/* we only need to pin inside GTT if cursor is non-phy */
	mutex_lock(&dev->struct_mutex);
	if (!dev_priv->info->cursor_needs_physical) {
		unsigned alignment;

		if (obj->tiling_mode) {
			DRM_ERROR("cursor cannot be tiled\n");
			ret = -EINVAL;
			goto fail_locked;
		}

		/* Note that the w/a also requires 2 PTE of padding following
		 * the bo. We currently fill all unused PTE with the shadow
		 * page and so we should always have valid PTE following the
		 * cursor preventing the VT-d warning.
		 */
		alignment = 0;
		if (need_vtd_wa(dev))
			alignment = 64*1024;

		ret = i915_gem_object_pin_to_display_plane(obj, alignment, NULL);
		if (ret) {
			DRM_ERROR("failed to move cursor bo into the GTT\n");
			goto fail_locked;
		}

		ret = i915_gem_object_put_fence(obj);
		if (ret) {
			DRM_ERROR("failed to release fence for cursor");
			goto fail_unpin;
		}

		addr = obj->gtt_offset;
	} else {
		int align = IS_I830(dev) ? 16 * 1024 : 256;
		ret = i915_gem_attach_phys_object(dev, obj,
						  (intel_crtc->pipe == 0) ? I915_GEM_PHYS_CURSOR_0 : I915_GEM_PHYS_CURSOR_1,
						  align);
		if (ret) {
			DRM_ERROR("failed to attach phys object\n");
			goto fail_locked;
		}
		addr = obj->phys_obj->handle->busaddr;
	}

	if (IS_GEN2(dev))
		I915_WRITE(CURSIZE, (height << 12) | width);

 finish:
	if (intel_crtc->cursor_bo) {
		if (dev_priv->info->cursor_needs_physical) {
			if (intel_crtc->cursor_bo != obj)
				i915_gem_detach_phys_object(dev, intel_crtc->cursor_bo);
		} else
			i915_gem_object_unpin(intel_crtc->cursor_bo);
		drm_gem_object_unreference(&intel_crtc->cursor_bo->base);
	}

	mutex_unlock(&dev->struct_mutex);

	intel_crtc->cursor_addr = addr;
	intel_crtc->cursor_bo = obj;
	intel_crtc->cursor_width = width;
	intel_crtc->cursor_height = height;

	intel_crtc_update_cursor(crtc, true);

	return 0;
fail_unpin:
	i915_gem_object_unpin(obj);
fail_locked:
	mutex_unlock(&dev->struct_mutex);
fail:
	drm_gem_object_unreference_unlocked(&obj->base);
	return ret;
}

static int intel_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	intel_crtc->cursor_x = x;
	intel_crtc->cursor_y = y;

	intel_crtc_update_cursor(crtc, true);

	return 0;
}

/** Sets the color ramps on behalf of RandR */
void intel_crtc_fb_gamma_set(struct drm_crtc *crtc, u16 red, u16 green,
				 u16 blue, int regno)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	intel_crtc->lut_r[regno] = red >> 8;
	intel_crtc->lut_g[regno] = green >> 8;
	intel_crtc->lut_b[regno] = blue >> 8;
}

void intel_crtc_fb_gamma_get(struct drm_crtc *crtc, u16 *red, u16 *green,
			     u16 *blue, int regno)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	*red = intel_crtc->lut_r[regno] << 8;
	*green = intel_crtc->lut_g[regno] << 8;
	*blue = intel_crtc->lut_b[regno] << 8;
}

static void intel_crtc_gamma_set(struct drm_crtc *crtc, u16 *red, u16 *green,
				 u16 *blue, uint32_t start, uint32_t size)
{
	int end = (start + size > 256) ? 256 : start + size, i;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	for (i = start; i < end; i++) {
		intel_crtc->lut_r[i] = red[i] >> 8;
		intel_crtc->lut_g[i] = green[i] >> 8;
		intel_crtc->lut_b[i] = blue[i] >> 8;
	}

	intel_crtc_load_lut(crtc);
}

/* VESA 640x480x72Hz mode to set on the pipe */
static struct drm_display_mode load_detect_mode = {
	DRM_MODE("640x480", DRM_MODE_TYPE_DEFAULT, 31500, 640, 664,
		 704, 832, 0, 480, 489, 491, 520, 0, DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
};

static struct drm_framebuffer *
intel_framebuffer_create(struct drm_device *dev,
			 struct drm_mode_fb_cmd2 *mode_cmd,
			 struct drm_i915_gem_object *obj)
{
	struct intel_framebuffer *intel_fb;
	int ret;

	intel_fb = kzalloc(sizeof(*intel_fb), GFP_KERNEL);
	if (!intel_fb) {
		drm_gem_object_unreference_unlocked(&obj->base);
		return ERR_PTR(-ENOMEM);
	}

	ret = intel_framebuffer_init(dev, intel_fb, mode_cmd, obj);
	if (ret) {
		drm_gem_object_unreference_unlocked(&obj->base);
		kfree(intel_fb);
		return ERR_PTR(ret);
	}

	return &intel_fb->base;
}

static u32
intel_framebuffer_pitch_for_width(int width, int bpp)
{
	u32 pitch = DIV_ROUND_UP(width * bpp, 8);
	return ALIGN(pitch, 64);
}

static u32
intel_framebuffer_size_for_mode(struct drm_display_mode *mode, int bpp)
{
	u32 pitch = intel_framebuffer_pitch_for_width(mode->hdisplay, bpp);
	return ALIGN(pitch * mode->vdisplay, PAGE_SIZE);
}

static struct drm_framebuffer *
intel_framebuffer_create_for_mode(struct drm_device *dev,
				  struct drm_display_mode *mode,
				  int depth, int bpp)
{
	struct drm_i915_gem_object *obj;
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };

	obj = i915_gem_alloc_object(dev,
				    intel_framebuffer_size_for_mode(mode, bpp));
	if (obj == NULL)
		return ERR_PTR(-ENOMEM);

	mode_cmd.width = mode->hdisplay;
	mode_cmd.height = mode->vdisplay;
	mode_cmd.pitches[0] = intel_framebuffer_pitch_for_width(mode_cmd.width,
								bpp);
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(bpp, depth);

	return intel_framebuffer_create(dev, &mode_cmd, obj);
}

static struct drm_framebuffer *
mode_fits_in_fbdev(struct drm_device *dev,
		   struct drm_display_mode *mode)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	struct drm_framebuffer *fb;

	if (dev_priv->fbdev == NULL)
		return NULL;

	obj = dev_priv->fbdev->ifb.obj;
	if (obj == NULL)
		return NULL;

	fb = &dev_priv->fbdev->ifb.base;
	if (fb->pitches[0] < intel_framebuffer_pitch_for_width(mode->hdisplay,
							       fb->bits_per_pixel))
		return NULL;

	if (obj->base.size < mode->vdisplay * fb->pitches[0])
		return NULL;

	return fb;
}

bool intel_get_load_detect_pipe(struct drm_connector *connector,
				struct drm_display_mode *mode,
				struct intel_load_detect_pipe *old)
{
	struct intel_crtc *intel_crtc;
	struct intel_encoder *intel_encoder =
		intel_attached_encoder(connector);
	struct drm_crtc *possible_crtc;
	struct drm_encoder *encoder = &intel_encoder->base;
	struct drm_crtc *crtc = NULL;
	struct drm_device *dev = encoder->dev;
	struct drm_framebuffer *fb;
	int i = -1;

	DRM_DEBUG_KMS("[CONNECTOR:%d:%s], [ENCODER:%d:%s]\n",
		      connector->base.id, drm_get_connector_name(connector),
		      encoder->base.id, drm_get_encoder_name(encoder));

	/*
	 * Algorithm gets a little messy:
	 *
	 *   - if the connector already has an assigned crtc, use it (but make
	 *     sure it's on first)
	 *
	 *   - try to find the first unused crtc that can drive this connector,
	 *     and use that if we find one
	 */

	/* See if we already have a CRTC for this connector */
	if (encoder->crtc) {
		crtc = encoder->crtc;

		old->dpms_mode = connector->dpms;
		old->load_detect_temp = false;

		/* Make sure the crtc and connector are running */
		if (connector->dpms != DRM_MODE_DPMS_ON)
			connector->funcs->dpms(connector, DRM_MODE_DPMS_ON);

		return true;
	}

	/* Find an unused one (if possible) */
	list_for_each_entry(possible_crtc, &dev->mode_config.crtc_list, head) {
		i++;
		if (!(encoder->possible_crtcs & (1 << i)))
			continue;
		if (!possible_crtc->enabled) {
			crtc = possible_crtc;
			break;
		}
	}

	/*
	 * If we didn't find an unused CRTC, don't use any.
	 */
	if (!crtc) {
		DRM_DEBUG_KMS("no pipe available for load-detect\n");
		return false;
	}

	intel_encoder->new_crtc = to_intel_crtc(crtc);
	to_intel_connector(connector)->new_encoder = intel_encoder;

	intel_crtc = to_intel_crtc(crtc);
	old->dpms_mode = connector->dpms;
	old->load_detect_temp = true;
	old->release_fb = NULL;

	if (!mode)
		mode = &load_detect_mode;

	/* We need a framebuffer large enough to accommodate all accesses
	 * that the plane may generate whilst we perform load detection.
	 * We can not rely on the fbcon either being present (we get called
	 * during its initialisation to detect all boot displays, or it may
	 * not even exist) or that it is large enough to satisfy the
	 * requested mode.
	 */
	fb = mode_fits_in_fbdev(dev, mode);
	if (fb == NULL) {
		DRM_DEBUG_KMS("creating tmp fb for load-detection\n");
		fb = intel_framebuffer_create_for_mode(dev, mode, 24, 32);
		old->release_fb = fb;
	} else
		DRM_DEBUG_KMS("reusing fbdev for load-detection framebuffer\n");
	if (IS_ERR(fb)) {
		DRM_DEBUG_KMS("failed to allocate framebuffer for load-detection\n");
		return false;
	}

	if (!intel_set_mode(crtc, mode, 0, 0, fb)) {
		DRM_DEBUG_KMS("failed to set mode on load-detect pipe\n");
		if (old->release_fb)
			old->release_fb->funcs->destroy(old->release_fb);
		return false;
	}

	/* let the connector get through one full cycle before testing */
	intel_wait_for_vblank(dev, intel_crtc->pipe);
	return true;
}

void intel_release_load_detect_pipe(struct drm_connector *connector,
				    struct intel_load_detect_pipe *old)
{
	struct intel_encoder *intel_encoder =
		intel_attached_encoder(connector);
	struct drm_encoder *encoder = &intel_encoder->base;

	DRM_DEBUG_KMS("[CONNECTOR:%d:%s], [ENCODER:%d:%s]\n",
		      connector->base.id, drm_get_connector_name(connector),
		      encoder->base.id, drm_get_encoder_name(encoder));

	if (old->load_detect_temp) {
		struct drm_crtc *crtc = encoder->crtc;

		to_intel_connector(connector)->new_encoder = NULL;
		intel_encoder->new_crtc = NULL;
		intel_set_mode(crtc, NULL, 0, 0, NULL);

		if (old->release_fb)
			old->release_fb->funcs->destroy(old->release_fb);

		return;
	}

	/* Switch crtc and encoder back off if necessary */
	if (old->dpms_mode != DRM_MODE_DPMS_ON)
		connector->funcs->dpms(connector, old->dpms_mode);
}

/* Returns the clock of the currently programmed mode of the given pipe. */
static int intel_crtc_clock_get(struct drm_device *dev, struct drm_crtc *crtc)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	u32 dpll = I915_READ(DPLL(pipe));
	u32 fp;
	intel_clock_t clock;

	if ((dpll & DISPLAY_RATE_SELECT_FPA1) == 0)
		fp = I915_READ(FP0(pipe));
	else
		fp = I915_READ(FP1(pipe));

	clock.m1 = (fp & FP_M1_DIV_MASK) >> FP_M1_DIV_SHIFT;
	if (IS_PINEVIEW(dev)) {
		clock.n = ffs((fp & FP_N_PINEVIEW_DIV_MASK) >> FP_N_DIV_SHIFT) - 1;
		clock.m2 = (fp & FP_M2_PINEVIEW_DIV_MASK) >> FP_M2_DIV_SHIFT;
	} else {
		clock.n = (fp & FP_N_DIV_MASK) >> FP_N_DIV_SHIFT;
		clock.m2 = (fp & FP_M2_DIV_MASK) >> FP_M2_DIV_SHIFT;
	}

	if (!IS_GEN2(dev)) {
		if (IS_PINEVIEW(dev))
			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK_PINEVIEW) >>
				DPLL_FPA01_P1_POST_DIV_SHIFT_PINEVIEW);
		else
			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK) >>
			       DPLL_FPA01_P1_POST_DIV_SHIFT);

		switch (dpll & DPLL_MODE_MASK) {
		case DPLLB_MODE_DAC_SERIAL:
			clock.p2 = dpll & DPLL_DAC_SERIAL_P2_CLOCK_DIV_5 ?
				5 : 10;
			break;
		case DPLLB_MODE_LVDS:
			clock.p2 = dpll & DPLLB_LVDS_P2_CLOCK_DIV_7 ?
				7 : 14;
			break;
		default:
			DRM_DEBUG_KMS("Unknown DPLL mode %08x in programmed "
				  "mode\n", (int)(dpll & DPLL_MODE_MASK));
			return 0;
		}

		/* XXX: Handle the 100Mhz refclk */
		intel_clock(dev, 96000, &clock);
	} else {
		bool is_lvds = (pipe == 1) && (I915_READ(LVDS) & LVDS_PORT_EN);

		if (is_lvds) {
			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK_I830_LVDS) >>
				       DPLL_FPA01_P1_POST_DIV_SHIFT);
			clock.p2 = 14;

			if ((dpll & PLL_REF_INPUT_MASK) ==
			    PLLB_REF_INPUT_SPREADSPECTRUMIN) {
				/* XXX: might not be 66MHz */
				intel_clock(dev, 66000, &clock);
			} else
				intel_clock(dev, 48000, &clock);
		} else {
			if (dpll & PLL_P1_DIVIDE_BY_TWO)
				clock.p1 = 2;
			else {
				clock.p1 = ((dpll & DPLL_FPA01_P1_POST_DIV_MASK_I830) >>
					    DPLL_FPA01_P1_POST_DIV_SHIFT) + 2;
			}
			if (dpll & PLL_P2_DIVIDE_BY_4)
				clock.p2 = 4;
			else
				clock.p2 = 2;

			intel_clock(dev, 48000, &clock);
		}
	}

	/* XXX: It would be nice to validate the clocks, but we can't reuse
	 * i830PllIsValid() because it relies on the xf86_config connector
	 * configuration being accurate, which it isn't necessarily.
	 */

	return clock.dot;
}

/** Returns the currently programmed mode of the given pipe. */
struct drm_display_mode *intel_crtc_mode_get(struct drm_device *dev,
					     struct drm_crtc *crtc)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	enum transcoder cpu_transcoder = intel_crtc->config.cpu_transcoder;
	struct drm_display_mode *mode;
	int htot = I915_READ(HTOTAL(cpu_transcoder));
	int hsync = I915_READ(HSYNC(cpu_transcoder));
	int vtot = I915_READ(VTOTAL(cpu_transcoder));
	int vsync = I915_READ(VSYNC(cpu_transcoder));

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->clock = intel_crtc_clock_get(dev, crtc);
	mode->hdisplay = (htot & 0xffff) + 1;
	mode->htotal = ((htot & 0xffff0000) >> 16) + 1;
	mode->hsync_start = (hsync & 0xffff) + 1;
	mode->hsync_end = ((hsync & 0xffff0000) >> 16) + 1;
	mode->vdisplay = (vtot & 0xffff) + 1;
	mode->vtotal = ((vtot & 0xffff0000) >> 16) + 1;
	mode->vsync_start = (vsync & 0xffff) + 1;
	mode->vsync_end = ((vsync & 0xffff0000) >> 16) + 1;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void intel_increase_pllclock(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int pipe = intel_crtc->pipe;
	int dpll_reg = DPLL(pipe);
	int dpll;

	if (HAS_PCH_SPLIT(dev))
		return;

	if (!dev_priv->lvds_downclock_avail)
		return;

	dpll = I915_READ(dpll_reg);
	if (!HAS_PIPE_CXSR(dev) && (dpll & DISPLAY_RATE_SELECT_FPA1)) {
		DRM_DEBUG_DRIVER("upclocking LVDS\n");

		assert_panel_unlocked(dev_priv, pipe);

		dpll &= ~DISPLAY_RATE_SELECT_FPA1;
		I915_WRITE(dpll_reg, dpll);
		intel_wait_for_vblank(dev, pipe);

		dpll = I915_READ(dpll_reg);
		if (dpll & DISPLAY_RATE_SELECT_FPA1)
			DRM_DEBUG_DRIVER("failed to upclock LVDS!\n");
	}
}

static void intel_decrease_pllclock(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	if (HAS_PCH_SPLIT(dev))
		return;

	if (!dev_priv->lvds_downclock_avail)
		return;

	/*
	 * Since this is called by a timer, we should never get here in
	 * the manual case.
	 */
	if (!HAS_PIPE_CXSR(dev) && intel_crtc->lowfreq_avail) {
		int pipe = intel_crtc->pipe;
		int dpll_reg = DPLL(pipe);
		u32 dpll;

		DRM_DEBUG_DRIVER("downclocking LVDS\n");

		assert_panel_unlocked(dev_priv, pipe);

		dpll = I915_READ(dpll_reg);
		dpll |= DISPLAY_RATE_SELECT_FPA1;
		I915_WRITE(dpll_reg, dpll);
		intel_wait_for_vblank(dev, pipe);
		dpll = I915_READ(dpll_reg);
		if (!(dpll & DISPLAY_RATE_SELECT_FPA1))
			DRM_DEBUG_DRIVER("failed to downclock LVDS!\n");
	}
}

/**
 * intel_idle_update - adjust clocks for idleness
 * @work: work struct
 *
 * Either the GPU or display (or both) went idle.  Check the busy status
 * here and adjust the CRTC and GPU clocks as necessary.
 */
static void intel_idle_update(struct work_struct *work)
{
	drm_i915_private_t *dev_priv = container_of(work, drm_i915_private_t,
						    idle_work);
	struct drm_device *dev = dev_priv->dev;
	struct drm_crtc *crtc;
	struct intel_crtc *intel_crtc;

	if (!i915_powersave)
		return;

	mutex_lock(&dev->struct_mutex);

	i915_update_gfx_val(dev_priv);

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		/* Skip inactive CRTCs */
		if (!crtc->fb)
			continue;

		intel_crtc = to_intel_crtc(crtc);
		if (!intel_crtc->busy)
			intel_decrease_pllclock(crtc);
	}


	mutex_unlock(&dev->struct_mutex);
}

/**
 * intel_mark_busy - mark the GPU and possibly the display busy
 * @dev: drm device
 * @obj: object we're operating on
 *
 * Callers can use this function to indicate that the GPU is busy processing
 * commands.  If @obj matches one of the CRTC objects (i.e. it's a scanout
 * buffer), we'll also mark the display as busy, so we know to increase its
 * clock frequency.
 */
void intel_mark_busy(struct drm_device *dev, struct drm_i915_gem_object *obj)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = NULL;
	struct intel_framebuffer *intel_fb;
	struct intel_crtc *intel_crtc;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return;

	if (!dev_priv->busy)
		dev_priv->busy = true;
	else
		mod_timer(&dev_priv->idle_timer, jiffies +
			  msecs_to_jiffies(GPU_IDLE_TIMEOUT));

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (!crtc->fb)
			continue;

		intel_crtc = to_intel_crtc(crtc);
		intel_fb = to_intel_framebuffer(crtc->fb);
		if (intel_fb->obj == obj) {
			if (!intel_crtc->busy) {
				/* Non-busy -> busy, upclock */
				intel_increase_pllclock(crtc);
				intel_crtc->busy = true;
			} else {
				/* Busy -> busy, put off timer */
				mod_timer(&intel_crtc->idle_timer, jiffies +
					  msecs_to_jiffies(CRTC_IDLE_TIMEOUT));
			}
		}
	}
}

static void intel_crtc_destroy(struct drm_crtc *crtc)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct intel_unpin_work *work;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	work = intel_crtc->unpin_work;
	intel_crtc->unpin_work = NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (work) {
		cancel_work_sync(&work->work);
		kfree(work);
	}

	drm_crtc_cleanup(crtc);

	kfree(intel_crtc);
}

static void intel_unpin_work_fn(struct work_struct *__work)
{
	struct intel_unpin_work *work =
		container_of(__work, struct intel_unpin_work, work);
	struct drm_device *dev = work->crtc->dev;

	mutex_lock(&dev->struct_mutex);
	intel_unpin_fb_obj(work->old_fb_obj);
	drm_gem_object_unreference(&work->pending_flip_obj->base);
	drm_gem_object_unreference(&work->old_fb_obj->base);

	intel_update_fbc(dev);
	mutex_unlock(&dev->struct_mutex);

	BUG_ON(atomic_read(&to_intel_crtc(work->crtc)->unpin_work_count) == 0);
	atomic_dec(&to_intel_crtc(work->crtc)->unpin_work_count);

	kfree(work);
}

static void do_intel_finish_page_flip(struct drm_device *dev,
				      struct drm_crtc *crtc)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_unpin_work *work;
	unsigned long flags;

	/* Ignore early vblank irqs */
	if (intel_crtc == NULL)
		return;

	spin_lock_irqsave(&dev->event_lock, flags);
	work = intel_crtc->unpin_work;

	/* Ensure we don't miss a work->pending update ... */
	smp_rmb();

	if (work == NULL || atomic_read(&work->pending) < INTEL_FLIP_COMPLETE) {
		spin_unlock_irqrestore(&dev->event_lock, flags);
		return;
	}

	/* and that the unpin work is consistent wrt ->pending. */
	smp_rmb();

	intel_crtc->unpin_work = NULL;

	if (work->event) {
		e = work->event;
		e->event.sequence = drm_vblank_count_and_time(dev, intel_crtc->pipe, &tvbl);

		e->event.tv_sec = tvbl.tv_sec;
		e->event.tv_usec = tvbl.tv_usec;

		list_add_tail(&e->base.link,
			      &e->base.file_priv->event_list);
		wake_up_interruptible(&e->base.file_priv->event_wait);
	}

	drm_vblank_put(dev, intel_crtc->pipe);

	spin_unlock_irqrestore(&dev->event_lock, flags);

	obj = work->old_fb_obj;

	atomic_clear_mask(1 << intel_crtc->plane,
			  &obj->pending_flip.counter);
	wake_up(&dev_priv->pending_flip_queue);

	queue_work(dev_priv->wq, &work->work);

	trace_i915_flip_complete(intel_crtc->plane, work->pending_flip_obj);
}

void intel_finish_page_flip(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];

	do_intel_finish_page_flip(dev, crtc);
}

void intel_finish_page_flip_plane(struct drm_device *dev, int plane)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->plane_to_crtc_mapping[plane];

	do_intel_finish_page_flip(dev, crtc);
}

void intel_prepare_page_flip(struct drm_device *dev, int plane)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(dev_priv->plane_to_crtc_mapping[plane]);
	unsigned long flags;

	/* NB: An MMIO update of the plane base pointer will also
	 * generate a page-flip completion irq, i.e. every modeset
	 * is also accompanied by a spurious intel_prepare_page_flip().
	 */
	spin_lock_irqsave(&dev->event_lock, flags);
	if (intel_crtc->unpin_work)
		atomic_inc_not_zero(&intel_crtc->unpin_work->pending);
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

inline static void intel_mark_page_flip_active(struct intel_crtc *intel_crtc)
{
	/* Ensure that the work item is consistent when activating it ... */
	smp_wmb();
	atomic_set(&intel_crtc->unpin_work->pending, INTEL_FLIP_PENDING);
	/* and that it is marked active as soon as the irq could fire. */
	smp_wmb();
}

static int intel_gen2_queue_flip(struct drm_device *dev,
				 struct drm_crtc *crtc,
				 struct drm_framebuffer *fb,
				 struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	u32 flip_mask;
	int ret;

	ret = intel_pin_and_fence_fb_obj(dev, obj, LP_RING(dev_priv));
	if (ret)
		goto err;

	/* Offset into the new buffer for cases of shared fbs between CRTCs */
	offset = crtc->y * fb->pitches[0] + crtc->x * fb->bits_per_pixel/8;

	ret = BEGIN_LP_RING(6);
	if (ret)
		goto err_unpin;

	/* Can't queue multiple flips, so wait for the previous
	 * one to finish before executing the next.
	 */
	if (intel_crtc->plane)
		flip_mask = MI_WAIT_FOR_PLANE_B_FLIP;
	else
		flip_mask = MI_WAIT_FOR_PLANE_A_FLIP;
	OUT_RING(MI_WAIT_FOR_EVENT | flip_mask);
	OUT_RING(MI_NOOP);
	OUT_RING(MI_DISPLAY_FLIP |
		 MI_DISPLAY_FLIP_PLANE(intel_crtc->plane));
	OUT_RING(fb->pitches[0]);
	OUT_RING(obj->gtt_offset + offset);
	OUT_RING(0); /* aux display base address, unused */

	intel_mark_page_flip_active(intel_crtc);
	ADVANCE_LP_RING();
	return 0;

err_unpin:
	intel_unpin_fb_obj(obj);
err:
	return ret;
}

static int intel_gen3_queue_flip(struct drm_device *dev,
				 struct drm_crtc *crtc,
				 struct drm_framebuffer *fb,
				 struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	u32 flip_mask;
	int ret;

	ret = intel_pin_and_fence_fb_obj(dev, obj, LP_RING(dev_priv));
	if (ret)
		goto err;

	/* Offset into the new buffer for cases of shared fbs between CRTCs */
	offset = crtc->y * fb->pitches[0] + crtc->x * fb->bits_per_pixel/8;

	ret = BEGIN_LP_RING(6);
	if (ret)
		goto err_unpin;

	if (intel_crtc->plane)
		flip_mask = MI_WAIT_FOR_PLANE_B_FLIP;
	else
		flip_mask = MI_WAIT_FOR_PLANE_A_FLIP;
	OUT_RING(MI_WAIT_FOR_EVENT | flip_mask);
	OUT_RING(MI_NOOP);
	OUT_RING(MI_DISPLAY_FLIP_I915 |
		 MI_DISPLAY_FLIP_PLANE(intel_crtc->plane));
	OUT_RING(fb->pitches[0]);
	OUT_RING(obj->gtt_offset + offset);
	OUT_RING(MI_NOOP);

	intel_mark_page_flip_active(intel_crtc);
	ADVANCE_LP_RING();
	return 0;

err_unpin:
	intel_unpin_fb_obj(obj);
err:
	return ret;
}

static int intel_gen4_queue_flip(struct drm_device *dev,
				 struct drm_crtc *crtc,
				 struct drm_framebuffer *fb,
				 struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	uint32_t pf, pipesrc;
	int ret;

	ret = intel_pin_and_fence_fb_obj(dev, obj, LP_RING(dev_priv));
	if (ret)
		goto err;

	ret = BEGIN_LP_RING(4);
	if (ret)
		goto err_unpin;

	/* i965+ uses the linear or tiled offsets from the
	 * Display Registers (which do not change across a page-flip)
	 * so we need only reprogram the base address.
	 */
	OUT_RING(MI_DISPLAY_FLIP |
		 MI_DISPLAY_FLIP_PLANE(intel_crtc->plane));
	OUT_RING(fb->pitches[0]);
	OUT_RING(obj->gtt_offset | obj->tiling_mode);

	/* XXX Enabling the panel-fitter across page-flip is so far
	 * untested on non-native modes, so ignore it for now.
	 * pf = I915_READ(pipe == 0 ? PFA_CTL_1 : PFB_CTL_1) & PF_ENABLE;
	 */
	pf = 0;
	pipesrc = I915_READ(PIPESRC(intel_crtc->pipe)) & 0x0fff0fff;
	OUT_RING(pf | pipesrc);

	intel_mark_page_flip_active(intel_crtc);
	ADVANCE_LP_RING();
	return 0;

err_unpin:
	intel_unpin_fb_obj(obj);
err:
	return ret;
}

static int intel_gen6_queue_flip(struct drm_device *dev,
				 struct drm_crtc *crtc,
				 struct drm_framebuffer *fb,
				 struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	uint32_t pf, pipesrc;
	int ret;

	ret = intel_pin_and_fence_fb_obj(dev, obj, LP_RING(dev_priv));
	if (ret)
		goto err;

	ret = BEGIN_LP_RING(4);
	if (ret)
		goto err_unpin;

	OUT_RING(MI_DISPLAY_FLIP |
		 MI_DISPLAY_FLIP_PLANE(intel_crtc->plane));
	OUT_RING(fb->pitches[0] | obj->tiling_mode);
	OUT_RING(obj->gtt_offset);

	/* Contrary to the suggestions in the documentation,
	 * "Enable Panel Fitter" does not seem to be required when page
	 * flipping with a non-native mode, and worse causes a normal
	 * modeset to fail.
	 * pf = I915_READ(PF_CTL(intel_crtc->pipe)) & PF_ENABLE;
	 */
	pf = 0;
	pipesrc = I915_READ(PIPESRC(intel_crtc->pipe)) & 0x0fff0fff;
	OUT_RING(pf | pipesrc);

	intel_mark_page_flip_active(intel_crtc);
	ADVANCE_LP_RING();
	return 0;

err_unpin:
	intel_unpin_fb_obj(obj);
err:
	return ret;
}

/*
 * On gen7 we currently use the blit ring because (in early silicon at least)
 * the render ring doesn't give us interrpts for page flip completion, which
 * means clients will hang after the first flip is queued.  Fortunately the
 * blit ring generates interrupts properly, so use it instead.
 */
static int intel_gen7_queue_flip(struct drm_device *dev,
				 struct drm_crtc *crtc,
				 struct drm_framebuffer *fb,
				 struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_ring_buffer *ring = &dev_priv->ring[BCS];
	uint32_t plane_bit = 0;
	int ret;

	ret = intel_pin_and_fence_fb_obj(dev, obj, ring);
	if (ret)
		goto err;

	switch(intel_crtc->plane) {
	case PLANE_A:
		plane_bit = MI_DISPLAY_FLIP_IVB_PLANE_A;
		break;
	case PLANE_B:
		plane_bit = MI_DISPLAY_FLIP_IVB_PLANE_B;
		break;
	case PLANE_C:
		plane_bit = MI_DISPLAY_FLIP_IVB_PLANE_C;
		break;
	default:
		WARN_ONCE(1, "unknown plane in flip command\n");
		ret = -ENODEV;
		goto err_unpin;
	}

	ret = intel_ring_begin(ring, 4);
	if (ret)
		goto err_unpin;

	intel_ring_emit(ring, MI_DISPLAY_FLIP_I915 | plane_bit);
	intel_ring_emit(ring, (fb->pitches[0] | obj->tiling_mode));
	intel_ring_emit(ring, obj->gtt_offset + intel_crtc->dspaddr_offset);
	intel_ring_emit(ring, (MI_NOOP));

	intel_mark_page_flip_active(intel_crtc);
	intel_ring_advance(ring);
	return 0;

err_unpin:
	intel_unpin_fb_obj(obj);
err:
	return ret;
}

static int intel_default_queue_flip(struct drm_device *dev,
				    struct drm_crtc *crtc,
				    struct drm_framebuffer *fb,
				    struct drm_i915_gem_object *obj)
{
	return -ENODEV;
}

static int intel_crtc_page_flip(struct drm_crtc *crtc,
				struct drm_framebuffer *fb,
				struct drm_pending_vblank_event *event)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_framebuffer *old_fb = crtc->fb;
	struct drm_i915_gem_object *obj = to_intel_framebuffer(fb)->obj;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_unpin_work *work;
	unsigned long flags;
	int ret;

	/* Can't change pixel format via MI display flips. */
	if (fb->pixel_format != crtc->fb->pixel_format)
		return -EINVAL;

	/*
	 * TILEOFF/LINOFF registers can't be changed via MI display flips.
	 * Note that pitch changes could also affect these register.
	 */
	if (INTEL_INFO(dev)->gen > 3 &&
	    (fb->offsets[0] != crtc->fb->offsets[0] ||
	     fb->pitches[0] != crtc->fb->pitches[0]))
		return -EINVAL;

	work = kzalloc(sizeof *work, GFP_KERNEL);
	if (work == NULL)
		return -ENOMEM;

	work->event = event;
	work->dev = crtc->dev;
	work->old_fb_obj = to_intel_framebuffer(old_fb)->obj;
	INIT_WORK(&work->work, intel_unpin_work_fn);

	ret = drm_vblank_get(dev, intel_crtc->pipe);
	if (ret)
		goto free_work;

	/* We borrow the event spin lock for protecting unpin_work */
	spin_lock_irqsave(&dev->event_lock, flags);
	if (intel_crtc->unpin_work) {
		spin_unlock_irqrestore(&dev->event_lock, flags);
		kfree(work);
		drm_vblank_put(dev, intel_crtc->pipe);

		DRM_DEBUG_DRIVER("flip queue: crtc already busy\n");
		return -EBUSY;
	}
	intel_crtc->unpin_work = work;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (atomic_read(&intel_crtc->unpin_work_count) >= 2)
		flush_workqueue(dev_priv->wq);

	ret = i915_mutex_lock_interruptible(dev);
	if (ret)
		goto cleanup;

	/* Reference the objects for the scheduled work. */
	drm_gem_object_reference(&work->old_fb_obj->base);
	drm_gem_object_reference(&obj->base);

	crtc->fb = fb;

	work->pending_flip_obj = obj;

	work->enable_stall_check = true;

	/* Block clients from rendering to the new back buffer until
	 * the flip occurs and the object is no longer visible.
	 */
	atomic_add(1 << intel_crtc->plane, &work->old_fb_obj->pending_flip);
	atomic_inc(&intel_crtc->unpin_work_count);

	ret = dev_priv->display.queue_flip(dev, crtc, fb, obj);
	if (ret)
		goto cleanup_pending;

	intel_disable_fbc(dev);
	mutex_unlock(&dev->struct_mutex);

	trace_i915_flip_request(intel_crtc->plane, obj);

	return 0;

cleanup_pending:
	atomic_dec(&intel_crtc->unpin_work_count);
	atomic_sub(1 << intel_crtc->plane, &work->old_fb_obj->pending_flip);
	crtc->fb = old_fb;
	drm_gem_object_unreference(&work->old_fb_obj->base);
	drm_gem_object_unreference(&obj->base);
	mutex_unlock(&dev->struct_mutex);

cleanup:
	spin_lock_irqsave(&dev->event_lock, flags);
	intel_crtc->unpin_work = NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	drm_vblank_put(dev, intel_crtc->pipe);
free_work:
	kfree(work);

	return ret;
}

static struct drm_crtc_helper_funcs intel_helper_funcs = {
	.mode_set_base_atomic = intel_pipe_set_base_atomic,
	.load_lut = intel_crtc_load_lut,
	.disable = intel_crtc_noop,
};

bool intel_encoder_check_is_cloned(struct intel_encoder *encoder)
{
	struct intel_encoder *other_encoder;
	struct drm_crtc *crtc = &encoder->new_crtc->base;

	if (WARN_ON(!crtc))
		return false;

	list_for_each_entry(other_encoder,
			    &crtc->dev->mode_config.encoder_list,
			    base.head) {

		if (&other_encoder->new_crtc->base != crtc ||
		    encoder == other_encoder)
			continue;
		else
			return true;
	}

	return false;
}

static bool intel_encoder_crtc_ok(struct drm_encoder *encoder,
				  struct drm_crtc *crtc)
{
	struct drm_device *dev;
	struct drm_crtc *tmp;
	int crtc_mask = 1;

	WARN(!crtc, "checking null crtc?\n");

	dev = crtc->dev;

	list_for_each_entry(tmp, &dev->mode_config.crtc_list, head) {
		if (tmp == crtc)
			break;
		crtc_mask <<= 1;
	}

	if (encoder->possible_crtcs & crtc_mask)
		return true;
	return false;
}

/**
 * intel_modeset_update_staged_output_state
 *
 * Updates the staged output configuration state, e.g. after we've read out the
 * current hw state.
 */
static void intel_modeset_update_staged_output_state(struct drm_device *dev)
{
	struct intel_encoder *encoder;
	struct intel_connector *connector;

	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    base.head) {
		connector->new_encoder =
			to_intel_encoder(connector->base.encoder);
	}

	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {
		encoder->new_crtc =
			to_intel_crtc(encoder->base.crtc);
	}
}

/**
 * intel_modeset_commit_output_state
 *
 * This function copies the stage display pipe configuration to the real one.
 */
static void intel_modeset_commit_output_state(struct drm_device *dev)
{
	struct intel_encoder *encoder;
	struct intel_connector *connector;

	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    base.head) {
		connector->base.encoder = &connector->new_encoder->base;
	}

	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {
		encoder->base.crtc = &encoder->new_crtc->base;
	}
}

static int
pipe_config_set_bpp(struct drm_crtc *crtc,
		    struct drm_framebuffer *fb,
		    struct intel_crtc_config *pipe_config)
{
	struct drm_device *dev = crtc->dev;
	struct drm_connector *connector;
	int bpp;

	switch (fb->pixel_format) {
	case DRM_FORMAT_C8:
		bpp = 8*3; /* since we go through a colormap */
		break;
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ARGB1555:
		/* checked in intel_framebuffer_init already */
		if (WARN_ON(INTEL_INFO(dev)->gen > 3))
			return -EINVAL;
	case DRM_FORMAT_RGB565:
		bpp = 6*3; /* min is 18bpp */
		break;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		/* checked in intel_framebuffer_init already */
		if (WARN_ON(INTEL_INFO(dev)->gen < 4))
			return -EINVAL;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		bpp = 8*3;
		break;
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_XBGR2101010:
	case DRM_FORMAT_ABGR2101010:
		/* checked in intel_framebuffer_init already */
		if (WARN_ON(INTEL_INFO(dev)->gen < 4))
			return -EINVAL;
		bpp = 10*3;
		break;
	/* TODO: gen4+ supports 16 bpc floating point, too. */
	default:
		DRM_DEBUG_KMS("unsupported depth\n");
		return -EINVAL;
	}

	pipe_config->pipe_bpp = bpp;

	/* Clamp display bpp to EDID value */
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    head) {
		if (connector->encoder && connector->encoder->crtc != crtc)
			continue;

		/* Don't use an invalid EDID bpc value */
		if (connector->display_info.bpc &&
		    connector->display_info.bpc * 3 < bpp) {
			DRM_DEBUG_KMS("clamping display bpp (was %d) to EDID reported max of %d\n",
				      bpp, connector->display_info.bpc*3);
			pipe_config->pipe_bpp = connector->display_info.bpc*3;
		}
	}

	return bpp;
}

static struct intel_crtc_config *
intel_modeset_pipe_config(struct drm_crtc *crtc,
			  struct drm_framebuffer *fb,
			  struct drm_display_mode *mode)
{
	struct drm_device *dev = crtc->dev;
	struct drm_encoder_helper_funcs *encoder_funcs;
	struct intel_encoder *encoder;
	struct intel_crtc_config *pipe_config;
	int plane_bpp;

	pipe_config = kzalloc(sizeof(*pipe_config), GFP_KERNEL);
	if (!pipe_config)
		return ERR_PTR(-ENOMEM);

	drm_mode_copy(&pipe_config->adjusted_mode, mode);
	drm_mode_copy(&pipe_config->requested_mode, mode);

	plane_bpp = pipe_config_set_bpp(crtc, fb, pipe_config);
	if (plane_bpp < 0)
		goto fail;

	/* Pass our mode to the connectors and the CRTC to give them a chance to
	 * adjust it according to limitations or connector properties, and also
	 * a chance to reject the mode entirely.
	 */
	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {

		if (&encoder->new_crtc->base != crtc)
			continue;

		if (encoder->compute_config) {
			if (!(encoder->compute_config(encoder, pipe_config))) {
				DRM_DEBUG_KMS("Encoder config failure\n");
				goto fail;
			}

			continue;
		}

		encoder_funcs = encoder->base.helper_private;
		if (!(encoder_funcs->mode_fixup(&encoder->base,
						&pipe_config->requested_mode,
						&pipe_config->adjusted_mode))) {
			DRM_DEBUG_KMS("Encoder fixup failed\n");
			goto fail;
		}
	}

	if (!(intel_crtc_compute_config(crtc, pipe_config))) {
		DRM_DEBUG_KMS("CRTC fixup failed\n");
		goto fail;
	}
	DRM_DEBUG_KMS("[CRTC:%d]\n", crtc->base.id);

	pipe_config->dither = pipe_config->pipe_bpp != plane_bpp;
	DRM_DEBUG_KMS("plane bpp: %i, pipe bpp: %i, dithering: %i\n",
		      plane_bpp, pipe_config->pipe_bpp, pipe_config->dither);

	return pipe_config;
fail:
	kfree(pipe_config);
	return ERR_PTR(-EINVAL);
}

/* Computes which crtcs are affected and sets the relevant bits in the mask. For
 * simplicity we use the crtc's pipe number (because it's easier to obtain). */
static void
intel_modeset_affected_pipes(struct drm_crtc *crtc, unsigned *modeset_pipes,
			     unsigned *prepare_pipes, unsigned *disable_pipes)
{
	struct intel_crtc *intel_crtc;
	struct drm_device *dev = crtc->dev;
	struct intel_encoder *encoder;
	struct intel_connector *connector;
	struct drm_crtc *tmp_crtc;

	*disable_pipes = *modeset_pipes = *prepare_pipes = 0;

	/* Check which crtcs have changed outputs connected to them, these need
	 * to be part of the prepare_pipes mask. We don't (yet) support global
	 * modeset across multiple crtcs, so modeset_pipes will only have one
	 * bit set at most. */
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    base.head) {
		if (connector->base.encoder == &connector->new_encoder->base)
			continue;

		if (connector->base.encoder) {
			tmp_crtc = connector->base.encoder->crtc;

			*prepare_pipes |= 1 << to_intel_crtc(tmp_crtc)->pipe;
		}

		if (connector->new_encoder)
			*prepare_pipes |=
				1 << connector->new_encoder->new_crtc->pipe;
	}

	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {
		if (encoder->base.crtc == &encoder->new_crtc->base)
			continue;

		if (encoder->base.crtc) {
			tmp_crtc = encoder->base.crtc;

			*prepare_pipes |= 1 << to_intel_crtc(tmp_crtc)->pipe;
		}

		if (encoder->new_crtc)
			*prepare_pipes |= 1 << encoder->new_crtc->pipe;
	}

	/* Check for any pipes that will be fully disabled ... */
	list_for_each_entry(intel_crtc, &dev->mode_config.crtc_list,
			    base.head) {
		bool used = false;

		/* Don't try to disable disabled crtcs. */
		if (!intel_crtc->base.enabled)
			continue;

		list_for_each_entry(encoder, &dev->mode_config.encoder_list,
				    base.head) {
			if (encoder->new_crtc == intel_crtc)
				used = true;
		}

		if (!used)
			*disable_pipes |= 1 << intel_crtc->pipe;
	}


	/* set_mode is also used to update properties on life display pipes. */
	intel_crtc = to_intel_crtc(crtc);
	if (crtc->enabled)
		*prepare_pipes |= 1 << intel_crtc->pipe;

	/*
	 * For simplicity do a full modeset on any pipe where the output routing
	 * changed. We could be more clever, but that would require us to be
	 * more careful with calling the relevant encoder->mode_set functions.
	 */
	if (*prepare_pipes)
		*modeset_pipes = *prepare_pipes;

	/* ... and mask these out. */
	*modeset_pipes &= ~(*disable_pipes);
	*prepare_pipes &= ~(*disable_pipes);

	/*
	 * HACK: We don't (yet) fully support global modesets. intel_set_config
	 * obies this rule, but the modeset restore mode of
	 * intel_modeset_setup_hw_state does not.
	 */
	*modeset_pipes &= 1 << intel_crtc->pipe;
	*prepare_pipes &= 1 << intel_crtc->pipe;
}

static bool intel_crtc_in_use(struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	struct drm_device *dev = crtc->dev;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head)
		if (encoder->crtc == crtc)
			return true;

	return false;
}

static void
intel_modeset_update_state(struct drm_device *dev, unsigned prepare_pipes)
{
	struct intel_encoder *intel_encoder;
	struct intel_crtc *intel_crtc;
	struct drm_connector *connector;

	list_for_each_entry(intel_encoder, &dev->mode_config.encoder_list,
			    base.head) {
		if (!intel_encoder->base.crtc)
			continue;

		intel_crtc = to_intel_crtc(intel_encoder->base.crtc);

		if (prepare_pipes & (1 << intel_crtc->pipe))
			intel_encoder->connectors_active = false;
	}

	intel_modeset_commit_output_state(dev);

	/* Update computed state. */
	list_for_each_entry(intel_crtc, &dev->mode_config.crtc_list,
			    base.head) {
		intel_crtc->base.enabled = intel_crtc_in_use(&intel_crtc->base);
	}

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (!connector->encoder || !connector->encoder->crtc)
			continue;

		intel_crtc = to_intel_crtc(connector->encoder->crtc);

		if (prepare_pipes & (1 << intel_crtc->pipe)) {
			struct drm_property *dpms_property =
				dev->mode_config.dpms_property;

			connector->dpms = DRM_MODE_DPMS_ON;
			drm_object_property_set_value(&connector->base,
							 dpms_property,
							 DRM_MODE_DPMS_ON);

			intel_encoder = to_intel_encoder(connector->encoder);
			intel_encoder->connectors_active = true;
		}
	}

}

#define for_each_intel_crtc_masked(dev, mask, intel_crtc) \
	list_for_each_entry((intel_crtc), \
			    &(dev)->mode_config.crtc_list, \
			    base.head) \
		if (mask & (1 <<(intel_crtc)->pipe)) \

static bool
intel_pipe_config_compare(struct intel_crtc_config *current_config,
			  struct intel_crtc_config *pipe_config)
{
	if (current_config->has_pch_encoder != pipe_config->has_pch_encoder) {
		DRM_ERROR("mismatch in has_pch_encoder "
			  "(expected %i, found %i)\n",
			  current_config->has_pch_encoder,
			  pipe_config->has_pch_encoder);
		return false;
	}

	return true;
}

void
intel_modeset_check_state(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_crtc *crtc;
	struct intel_encoder *encoder;
	struct intel_connector *connector;
	struct intel_crtc_config pipe_config;

	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    base.head) {
		/* This also checks the encoder/connector hw state with the
		 * ->get_hw_state callbacks. */
		intel_connector_check_state(connector);

		WARN(&connector->new_encoder->base != connector->base.encoder,
		     "connector's staged encoder doesn't match current encoder\n");
	}

	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {
		bool enabled = false;
		bool active = false;
		enum pipe pipe, tracked_pipe;

		DRM_DEBUG_KMS("[ENCODER:%d:%s]\n",
			      encoder->base.base.id,
			      drm_get_encoder_name(&encoder->base));

		WARN(&encoder->new_crtc->base != encoder->base.crtc,
		     "encoder's stage crtc doesn't match current crtc\n");
		WARN(encoder->connectors_active && !encoder->base.crtc,
		     "encoder's active_connectors set, but no crtc\n");

		list_for_each_entry(connector, &dev->mode_config.connector_list,
				    base.head) {
			if (connector->base.encoder != &encoder->base)
				continue;
			enabled = true;
			if (connector->base.dpms != DRM_MODE_DPMS_OFF)
				active = true;
		}
		WARN(!!encoder->base.crtc != enabled,
		     "encoder's enabled state mismatch "
		     "(expected %i, found %i)\n",
		     !!encoder->base.crtc, enabled);
		WARN(active && !encoder->base.crtc,
		     "active encoder with no crtc\n");

		WARN(encoder->connectors_active != active,
		     "encoder's computed active state doesn't match tracked active state "
		     "(expected %i, found %i)\n", active, encoder->connectors_active);

		active = encoder->get_hw_state(encoder, &pipe);
		WARN(active != encoder->connectors_active,
		     "encoder's hw state doesn't match sw tracking "
		     "(expected %i, found %i)\n",
		     encoder->connectors_active, active);

		if (!encoder->base.crtc)
			continue;

		tracked_pipe = to_intel_crtc(encoder->base.crtc)->pipe;
		WARN(active && pipe != tracked_pipe,
		     "active encoder's pipe doesn't match"
		     "(expected %i, found %i)\n",
		     tracked_pipe, pipe);

	}

	list_for_each_entry(crtc, &dev->mode_config.crtc_list,
			    base.head) {
		bool enabled = false;
		bool active = false;

		DRM_DEBUG_KMS("[CRTC:%d]\n",
			      crtc->base.base.id);

		WARN(crtc->active && !crtc->base.enabled,
		     "active crtc, but not enabled in sw tracking\n");

		list_for_each_entry(encoder, &dev->mode_config.encoder_list,
				    base.head) {
			if (encoder->base.crtc != &crtc->base)
				continue;
			enabled = true;
			if (encoder->connectors_active)
				active = true;
		}
		WARN(active != crtc->active,
		     "crtc's computed active state doesn't match tracked active state "
		     "(expected %i, found %i)\n", active, crtc->active);
		WARN(enabled != crtc->base.enabled,
		     "crtc's computed enabled state doesn't match tracked enabled state "
		     "(expected %i, found %i)\n", enabled, crtc->base.enabled);

		assert_pipe(dev->dev_private, crtc->pipe, crtc->active);
	}
}

bool intel_set_mode(struct drm_crtc *crtc,
		    struct drm_display_mode *mode,
		    int x, int y, struct drm_framebuffer *fb)
{
	struct drm_device *dev = crtc->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_display_mode *adjusted_mode, saved_mode, saved_hwmode;
	struct intel_crtc *intel_crtc;
	unsigned disable_pipes, prepare_pipes, modeset_pipes;
	bool ret = true;

	intel_modeset_affected_pipes(crtc, &modeset_pipes,
				     &prepare_pipes, &disable_pipes);

	DRM_DEBUG_KMS("set mode pipe masks: modeset: %x, prepare: %x, disable: %x\n",
		      modeset_pipes, prepare_pipes, disable_pipes);

	for_each_intel_crtc_masked(dev, disable_pipes, intel_crtc)
		intel_crtc_disable(&intel_crtc->base);

	saved_hwmode = crtc->hwmode;
	saved_mode = crtc->mode;

	/* Hack: Because we don't (yet) support global modeset on multiple
	 * crtcs, we don't keep track of the new mode for more than one crtc.
	 * Hence simply check whether any bit is set in modeset_pipes in all the
	 * pieces of code that are not yet converted to deal with mutliple crtcs
	 * changing their mode at the same time. */
	adjusted_mode = NULL;
	if (modeset_pipes) {
		adjusted_mode = intel_modeset_adjusted_mode(crtc, mode);
		if (IS_ERR(adjusted_mode)) {
			return false;
		}
	}

	for_each_intel_crtc_masked(dev, prepare_pipes, intel_crtc) {
		if (intel_crtc->base.enabled)
			dev_priv->display.crtc_disable(&intel_crtc->base);
	}

	/* crtc->mode is already used by the ->mode_set callbacks, hence we need
	 * to set it here already despite that we pass it down the callchain.
	 */
	if (modeset_pipes) {
		enum transcoder tmp = to_intel_crtc(crtc)->config.cpu_transcoder;
		crtc->mode = *mode;
		/* mode_set/enable/disable functions rely on a correct pipe
		 * config. */
		to_intel_crtc(crtc)->config = *pipe_config;
		to_intel_crtc(crtc)->config.cpu_transcoder = tmp;
	}

	/* Only after disabling all output pipelines that will be changed can we
	 * update the the output configuration. */
	intel_modeset_update_state(dev, prepare_pipes);

	if (dev_priv->display.modeset_global_resources)
		dev_priv->display.modeset_global_resources(dev);

	/* Set up the DPLL and any encoders state that needs to adjust or depend
	 * on the DPLL.
	 */
	for_each_intel_crtc_masked(dev, modeset_pipes, intel_crtc) {
		ret = !intel_crtc_mode_set(&intel_crtc->base,
					   mode, adjusted_mode,
					   x, y, fb);
		if (!ret)
		    goto done;
	}

	/* Now enable the clocks, plane, pipe, and connectors that we set up. */
	for_each_intel_crtc_masked(dev, prepare_pipes, intel_crtc)
		dev_priv->display.crtc_enable(&intel_crtc->base);

	if (modeset_pipes) {
		/* Store real post-adjustment hardware mode. */
		crtc->hwmode = pipe_config->adjusted_mode;

		/* Calculate and store various constants which
		 * are later needed by vblank and swap-completion
		 * timestamping. They are derived from true hwmode.
		 */
		drm_calc_timestamping_constants(crtc);
	}

	/* FIXME: add subpixel order */
done:
	drm_mode_destroy(dev, adjusted_mode);
	if (!ret && crtc->enabled) {
		crtc->hwmode = saved_hwmode;
		crtc->mode = saved_mode;
	} else {
		intel_modeset_check_state(dev);
	}

	return ret;
}

#undef for_each_intel_crtc_masked

static void intel_set_config_free(struct intel_set_config *config)
{
	if (!config)
		return;

	kfree(config->save_connector_encoders);
	kfree(config->save_encoder_crtcs);
	kfree(config);
}

static int intel_set_config_save_state(struct drm_device *dev,
				       struct intel_set_config *config)
{
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	int count;

	config->save_encoder_crtcs =
		kcalloc(dev->mode_config.num_encoder,
			sizeof(struct drm_crtc *), GFP_KERNEL);
	if (!config->save_encoder_crtcs)
		return -ENOMEM;

	config->save_connector_encoders =
		kcalloc(dev->mode_config.num_connector,
			sizeof(struct drm_encoder *), GFP_KERNEL);
	if (!config->save_connector_encoders)
		return -ENOMEM;

	/* Copy data. Note that driver private data is not affected.
	 * Should anything bad happen only the expected state is
	 * restored, not the drivers personal bookkeeping.
	 */
	count = 0;
	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		config->save_encoder_crtcs[count++] = encoder->crtc;
	}

	count = 0;
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		config->save_connector_encoders[count++] = connector->encoder;
	}

	return 0;
}

static void intel_set_config_restore_state(struct drm_device *dev,
					   struct intel_set_config *config)
{
	struct intel_encoder *encoder;
	struct intel_connector *connector;
	int count;

	count = 0;
	list_for_each_entry(encoder, &dev->mode_config.encoder_list, base.head) {
		encoder->new_crtc =
			to_intel_crtc(config->save_encoder_crtcs[count++]);
	}

	count = 0;
	list_for_each_entry(connector, &dev->mode_config.connector_list, base.head) {
		connector->new_encoder =
			to_intel_encoder(config->save_connector_encoders[count++]);
	}
}

static bool
is_crtc_connector_off(struct drm_crtc *crtc, struct drm_connector *connectors,
		      int num_connectors)
{
	int i;

	for (i = 0; i < num_connectors; i++)
		if (connectors[i].encoder &&
		    connectors[i].encoder->crtc == crtc &&
		    connectors[i].dpms != DRM_MODE_DPMS_ON)
			return true;

	return false;
}

static void
intel_set_config_compute_mode_changes(struct drm_mode_set *set,
				      struct intel_set_config *config)
{

	/* We should be able to check here if the fb has the same properties
	 * and then just flip_or_move it */
	if (set->connectors != NULL &&
	    is_crtc_connector_off(set->crtc, *set->connectors,
				  set->num_connectors)) {
			config->mode_changed = true;
	} else if (set->crtc->fb != set->fb) {
		/* If we have no fb then treat it as a full mode set */
		if (set->crtc->fb == NULL) {
			DRM_DEBUG_KMS("crtc has no fb, full mode set\n");
			config->mode_changed = true;
		} else if (set->fb == NULL) {
			config->mode_changed = true;
		} else if (set->fb->pixel_format !=
			   set->crtc->fb->pixel_format) {
			config->mode_changed = true;
		} else {
			config->fb_changed = true;
		}
	}

	if (set->fb && (set->x != set->crtc->x || set->y != set->crtc->y))
		config->fb_changed = true;

	if (set->mode && !drm_mode_equal(set->mode, &set->crtc->mode)) {
		DRM_DEBUG_KMS("modes are different, full mode set\n");
		drm_mode_debug_printmodeline(&set->crtc->mode);
		drm_mode_debug_printmodeline(set->mode);
		config->mode_changed = true;
	}
}

static int
intel_modeset_stage_output_state(struct drm_device *dev,
				 struct drm_mode_set *set,
				 struct intel_set_config *config)
{
	struct drm_crtc *new_crtc;
	struct intel_connector *connector;
	struct intel_encoder *encoder;
	int count, ro;

	/* The upper layers ensure that we either disabl a crtc or have a list
	 * of connectors. For paranoia, double-check this. */
	WARN_ON(!set->fb && (set->num_connectors != 0));
	WARN_ON(set->fb && (set->num_connectors == 0));

	count = 0;
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    base.head) {
		/* Otherwise traverse passed in connector list and get encoders
		 * for them. */
		for (ro = 0; ro < set->num_connectors; ro++) {
			if (set->connectors[ro] == &connector->base) {
				connector->new_encoder = connector->encoder;
				break;
			}
		}

		/* If we disable the crtc, disable all its connectors. Also, if
		 * the connector is on the changing crtc but not on the new
		 * connector list, disable it. */
		if ((!set->fb || ro == set->num_connectors) &&
		    connector->base.encoder &&
		    connector->base.encoder->crtc == set->crtc) {
			connector->new_encoder = NULL;

			DRM_DEBUG_KMS("[CONNECTOR:%d:%s] to [NOCRTC]\n",
				connector->base.base.id,
				drm_get_connector_name(&connector->base));
		}


		if (&connector->new_encoder->base != connector->base.encoder) {
			DRM_DEBUG_KMS("encoder changed, full mode switch\n");
			config->mode_changed = true;
		}
	}
	/* connector->new_encoder is now updated for all connectors. */

	/* Update crtc of enabled connectors. */
	count = 0;
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    base.head) {
		if (!connector->new_encoder)
			continue;

		new_crtc = connector->new_encoder->base.crtc;

		for (ro = 0; ro < set->num_connectors; ro++) {
			if (set->connectors[ro] == &connector->base)
				new_crtc = set->crtc;
		}

		/* Make sure the new CRTC will work with the encoder */
		if (!intel_encoder_crtc_ok(&connector->new_encoder->base,
					   new_crtc)) {
			return -EINVAL;
		}
		connector->encoder->new_crtc = to_intel_crtc(new_crtc);

		DRM_DEBUG_KMS("[CONNECTOR:%d:%s] to [CRTC:%d]\n",
			connector->base.base.id,
			drm_get_connector_name(&connector->base),
			new_crtc->base.id);
	}

	/* Check for any encoders that needs to be disabled. */
	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {
		list_for_each_entry(connector,
				    &dev->mode_config.connector_list,
				    base.head) {
			if (connector->new_encoder == encoder) {
				WARN_ON(!connector->new_encoder->new_crtc);

				goto next_encoder;
			}
		}
		encoder->new_crtc = NULL;
next_encoder:
		/* Only now check for crtc changes so we don't miss encoders
		 * that will be disabled. */
		if (&encoder->new_crtc->base != encoder->base.crtc) {
			DRM_DEBUG_KMS("crtc changed, full mode switch\n");
			config->mode_changed = true;
		}
	}
	/* Now we've also updated encoder->new_crtc for all encoders. */

	return 0;
}

static int intel_crtc_set_config(struct drm_mode_set *set)
{
	struct drm_device *dev;
	struct drm_mode_set save_set;
	struct intel_set_config *config;
	int ret;

	BUG_ON(!set);
	BUG_ON(!set->crtc);
	BUG_ON(!set->crtc->helper_private);

	if (!set->mode)
		set->fb = NULL;

	/* The fb helper likes to play gross jokes with ->mode_set_config.
	 * Unfortunately the crtc helper doesn't do much at all for this case,
	 * so we have to cope with this madness until the fb helper is fixed up. */
	if (set->fb && set->num_connectors == 0)
		return 0;

	if (set->fb) {
		DRM_DEBUG_KMS("[CRTC:%d] [FB:%d] #connectors=%d (x y) (%i %i)\n",
				set->crtc->base.id, set->fb->base.id,
				(int)set->num_connectors, set->x, set->y);
	} else {
		DRM_DEBUG_KMS("[CRTC:%d] [NOFB]\n", set->crtc->base.id);
	}

	dev = set->crtc->dev;

	ret = -ENOMEM;
	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		goto out_config;

	ret = intel_set_config_save_state(dev, config);
	if (ret)
		goto out_config;

	save_set.crtc = set->crtc;
	save_set.mode = &set->crtc->mode;
	save_set.x = set->crtc->x;
	save_set.y = set->crtc->y;
	save_set.fb = set->crtc->fb;

	/* Compute whether we need a full modeset, only an fb base update or no
	 * change at all. In the future we might also check whether only the
	 * mode changed, e.g. for LVDS where we only change the panel fitter in
	 * such cases. */
	intel_set_config_compute_mode_changes(set, config);

	ret = intel_modeset_stage_output_state(dev, set, config);
	if (ret)
		goto fail;

	if (config->mode_changed) {
		if (set->mode) {
			DRM_DEBUG_KMS("attempting to set mode from"
					" userspace\n");
			drm_mode_debug_printmodeline(set->mode);
		}

		if (!intel_set_mode(set->crtc, set->mode,
				    set->x, set->y, set->fb)) {
			DRM_ERROR("failed to set mode on [CRTC:%d]\n",
				  set->crtc->base.id);
			ret = -EINVAL;
			goto fail;
		}
	} else if (config->fb_changed) {
		ret = intel_pipe_set_base(set->crtc,
					  set->x, set->y, set->fb);
	}

	intel_set_config_free(config);

	return 0;

fail:
	intel_set_config_restore_state(dev, config);

	/* Try to restore the config */
	if (config->mode_changed &&
	    !intel_set_mode(save_set.crtc, save_set.mode,
			    save_set.x, save_set.y, save_set.fb))
		DRM_ERROR("failed to restore config after modeset failure\n");

out_config:
	intel_set_config_free(config);
	return ret;
}

static const struct drm_crtc_funcs intel_crtc_funcs = {
	.cursor_set = intel_crtc_cursor_set,
	.cursor_move = intel_crtc_cursor_move,
	.gamma_set = intel_crtc_gamma_set,
	.set_config = intel_crtc_set_config,
	.destroy = intel_crtc_destroy,
	.page_flip = intel_crtc_page_flip,
};

static void intel_crtc_init(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc;
	int i;

	intel_crtc = kzalloc(sizeof(struct intel_crtc) + (INTELFB_CONN_LIMIT * sizeof(struct drm_connector *)), GFP_KERNEL);
	if (intel_crtc == NULL)
		return;

	drm_crtc_init(dev, &intel_crtc->base, &intel_crtc_funcs);

	drm_mode_crtc_set_gamma_size(&intel_crtc->base, 256);
	for (i = 0; i < 256; i++) {
		intel_crtc->lut_r[i] = i;
		intel_crtc->lut_g[i] = i;
		intel_crtc->lut_b[i] = i;
	}

	/* Swap pipes & planes for FBC on pre-965 */
	intel_crtc->pipe = pipe;
	intel_crtc->plane = pipe;
	intel_crtc->config.cpu_transcoder = pipe;
	if (IS_MOBILE(dev) && IS_GEN3(dev)) {
		DRM_DEBUG_KMS("swapping pipes & planes for FBC\n");
		intel_crtc->plane = !pipe;
	}

	BUG_ON(pipe >= ARRAY_SIZE(dev_priv->plane_to_crtc_mapping) ||
	       dev_priv->plane_to_crtc_mapping[intel_crtc->plane] != NULL);
	dev_priv->plane_to_crtc_mapping[intel_crtc->plane] = &intel_crtc->base;
	dev_priv->pipe_to_crtc_mapping[intel_crtc->pipe] = &intel_crtc->base;

	intel_crtc_reset(&intel_crtc->base);
	intel_crtc->active = true; /* force the pipe off on setup_init_config */
	intel_crtc->bpp = 24; /* default for pre-Ironlake */

	if (HAS_PCH_SPLIT(dev)) {
		if (pipe == 2 && IS_IVYBRIDGE(dev))
			intel_crtc->no_pll = true;
		intel_helper_funcs.prepare = ironlake_crtc_prepare;
		intel_helper_funcs.commit = ironlake_crtc_commit;
	} else {
		intel_helper_funcs.prepare = i9xx_crtc_prepare;
		intel_helper_funcs.commit = i9xx_crtc_commit;
	}

	drm_crtc_helper_add(&intel_crtc->base, &intel_helper_funcs);

	intel_crtc->busy = false;

	setup_timer(&intel_crtc->idle_timer, intel_crtc_idle_timer,
		    (unsigned long)intel_crtc);
}

int intel_get_pipe_from_crtc_id(struct drm_device *dev, void *data,
				struct drm_file *file)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_i915_get_pipe_from_crtc_id *pipe_from_crtc_id = data;
	struct drm_mode_object *drmmode_obj;
	struct intel_crtc *crtc;

	if (!dev_priv) {
		DRM_ERROR("called with no initialization\n");
		return -EINVAL;
	}

	drmmode_obj = drm_mode_object_find(dev, pipe_from_crtc_id->crtc_id,
			DRM_MODE_OBJECT_CRTC);

	if (!drmmode_obj) {
		DRM_ERROR("no such CRTC id\n");
		return -EINVAL;
	}

	crtc = to_intel_crtc(obj_to_crtc(drmmode_obj));
	pipe_from_crtc_id->pipe = crtc->pipe;

	return 0;
}

static int intel_encoder_clones(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct intel_encoder *source_encoder;
	int index_mask = 0;
	int entry = 0;

	list_for_each_entry(source_encoder,
			    &dev->mode_config.encoder_list, base.head) {

		if (encoder == source_encoder)
			index_mask |= (1 << entry);

		/* Intel hw has only one MUX where enocoders could be cloned. */
		if (encoder->cloneable && source_encoder->cloneable)
			index_mask |= (1 << entry);

		entry++;
	}

	return index_mask;
}

static bool has_edp_a(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!IS_MOBILE(dev))
		return false;

	if ((I915_READ(DP_A) & DP_DETECTED) == 0)
		return false;

	if (IS_GEN5(dev) &&
	    (I915_READ(ILK_DISPLAY_CHICKEN_FUSES) & ILK_eDP_A_DISABLE))
		return false;

	return true;
}

static void intel_setup_outputs(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_encoder *encoder;
	bool dpd_is_edp = false;
	bool has_lvds;

	has_lvds = intel_lvds_init(dev);
	if (!has_lvds && !HAS_PCH_SPLIT(dev)) {
		/* disable the panel fitter on everything but LVDS */
		I915_WRITE(PFIT_CONTROL, 0);
	}

	if (!(IS_HASWELL(dev) &&
	      (I915_READ(DDI_BUF_CTL(PORT_A)) & DDI_A_4_LANES)))
		intel_crt_init(dev);

	if (HAS_PCH_SPLIT(dev)) {
		int found;

		if (I915_READ(HDMIB) & PORT_DETECTED) {
			/* PCH SDVOB multiplex with HDMIB */
			found = intel_sdvo_init(dev, PCH_SDVOB);
			if (!found)
				intel_hdmi_init(dev, HDMIB);
			if (!found && (I915_READ(PCH_DP_B) & DP_DETECTED))
				intel_dp_init(dev, PCH_DP_B);
		}

		if (I915_READ(HDMIC) & PORT_DETECTED)
			intel_hdmi_init(dev, HDMIC);

		if (I915_READ(HDMID) & PORT_DETECTED)
			intel_hdmi_init(dev, HDMID);

		if (I915_READ(PCH_DP_C) & DP_DETECTED)
			intel_dp_init(dev, PCH_DP_C);

		if (!dpd_is_edp && (I915_READ(PCH_DP_D) & DP_DETECTED))
			intel_dp_init(dev, PCH_DP_D);
	} else if (IS_VALLEYVIEW(dev)) {
		int found;
		dpd_is_edp = intel_dpd_is_edp(dev);

		if (has_edp_a(dev))
			intel_dp_init(dev, DP_A, PORT_A);

		if (I915_READ(SDVOB) & PORT_DETECTED) {
			/* SDVOB multiplex with HDMIB */
			found = intel_sdvo_init(dev, SDVOB, true);
			if (!found)
				intel_hdmi_init(dev, SDVOB, PORT_B);
			if (!found && (I915_READ(DP_B) & DP_DETECTED))
				intel_dp_init(dev, DP_B, PORT_B);
		}

		if (I915_READ(SDVOC) & PORT_DETECTED)
			intel_hdmi_init(dev, SDVOC, PORT_C);

	} else if (SUPPORTS_DIGITAL_OUTPUTS(dev)) {
		bool found = false;

		if (I915_READ(SDVOB) & SDVO_DETECTED) {
			DRM_DEBUG_KMS("probing SDVOB\n");
			found = intel_sdvo_init(dev, SDVOB);
			if (!found && SUPPORTS_INTEGRATED_HDMI(dev)) {
				DRM_DEBUG_KMS("probing HDMI on SDVOB\n");
				intel_hdmi_init(dev, SDVOB, PORT_B);
			}

			if (!found && SUPPORTS_INTEGRATED_DP(dev)) {
				DRM_DEBUG_KMS("probing DP_B\n");
				intel_dp_init(dev, DP_B, PORT_B);
			}
		}

		/* Before G4X SDVOC doesn't have its own detect register */

		if (I915_READ(SDVOB) & SDVO_DETECTED) {
			DRM_DEBUG_KMS("probing SDVOC\n");
			found = intel_sdvo_init(dev, SDVOC);
		}

		if (!found && (I915_READ(SDVOC) & SDVO_DETECTED)) {

			if (SUPPORTS_INTEGRATED_HDMI(dev)) {
				DRM_DEBUG_KMS("probing HDMI on SDVOC\n");
				intel_hdmi_init(dev, SDVOC, PORT_C);
			}
			if (SUPPORTS_INTEGRATED_DP(dev)) {
				DRM_DEBUG_KMS("probing DP_C\n");
				intel_dp_init(dev, DP_C, PORT_C);
			}
		}

		if (SUPPORTS_INTEGRATED_DP(dev) &&
		    (I915_READ(DP_D) & DP_DETECTED)) {
			DRM_DEBUG_KMS("probing DP_D\n");
			intel_dp_init(dev, DP_D, PORT_D);
		}
	} else if (IS_GEN2(dev))
		intel_dvo_init(dev);

	if (SUPPORTS_TV(dev))
		intel_tv_init(dev);

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, base.head) {
		encoder->base.possible_crtcs = encoder->crtc_mask;
		encoder->base.possible_clones =
			intel_encoder_clones(encoder);
	}

	intel_init_pch_refclk(dev);

	drm_helper_move_panel_connectors_to_head(dev);
}

static void intel_user_framebuffer_destroy(struct drm_framebuffer *fb)
{
	struct intel_framebuffer *intel_fb = to_intel_framebuffer(fb);

	drm_framebuffer_cleanup(fb);
	drm_gem_object_unreference_unlocked(&intel_fb->obj->base);

	kfree(intel_fb);
}

static int intel_user_framebuffer_create_handle(struct drm_framebuffer *fb,
						struct drm_file *file,
						unsigned int *handle)
{
	struct intel_framebuffer *intel_fb = to_intel_framebuffer(fb);
	struct drm_i915_gem_object *obj = intel_fb->obj;

	return drm_gem_handle_create(file, &obj->base, handle);
}

static const struct drm_framebuffer_funcs intel_fb_funcs = {
	.destroy = intel_user_framebuffer_destroy,
	.create_handle = intel_user_framebuffer_create_handle,
};

int intel_framebuffer_init(struct drm_device *dev,
			   struct intel_framebuffer *intel_fb,
			   struct drm_mode_fb_cmd2 *mode_cmd,
			   struct drm_i915_gem_object *obj)
{
	int ret;

	if (obj->tiling_mode == I915_TILING_Y) {
		DRM_DEBUG("hardware does not support tiling Y\n");
		return -EINVAL;
	}

	if (mode_cmd->pitches[0] & 63) {
		DRM_DEBUG("pitch (%d) must be at least 64 byte aligned\n",
			  mode_cmd->pitches[0]);
		return -EINVAL;
	}

	/* FIXME <= Gen4 stride limits are bit unclear */
	if (mode_cmd->pitches[0] > 32768) {
		DRM_DEBUG("pitch (%d) must be at less than 32768\n",
			  mode_cmd->pitches[0]);
		return -EINVAL;
	}

	if (obj->tiling_mode != I915_TILING_NONE &&
	    mode_cmd->pitches[0] != obj->stride) {
		DRM_DEBUG("pitch (%d) must match tiling stride (%d)\n",
			  mode_cmd->pitches[0], obj->stride);
		return -EINVAL;
	}

	/* Reject formats not supported by any plane early. */
	switch (mode_cmd->pixel_format) {
	case DRM_FORMAT_C8:
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		break;
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ARGB1555:
		if (INTEL_INFO(dev)->gen > 3) {
			DRM_DEBUG("invalid format: 0x%08x\n", mode_cmd->pixel_format);
			return -EINVAL;
		}
		break;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_XBGR2101010:
	case DRM_FORMAT_ABGR2101010:
		if (INTEL_INFO(dev)->gen < 4) {
			DRM_DEBUG("invalid format: 0x%08x\n", mode_cmd->pixel_format);
			return -EINVAL;
		}
		break;
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_YVYU:
	case DRM_FORMAT_VYUY:
		if (INTEL_INFO(dev)->gen < 5) {
			DRM_DEBUG("invalid format: 0x%08x\n", mode_cmd->pixel_format);
			return -EINVAL;
		}
		break;
	default:
		DRM_DEBUG("unsupported pixel format 0x%08x\n", mode_cmd->pixel_format);
		return -EINVAL;
	}

	/* FIXME need to adjust LINOFF/TILEOFF accordingly. */
	if (mode_cmd->offsets[0] != 0)
		return -EINVAL;

	ret = drm_framebuffer_init(dev, &intel_fb->base, &intel_fb_funcs);
	if (ret) {
		DRM_ERROR("framebuffer init failed %d\n", ret);
		return ret;
	}

	drm_helper_mode_fill_fb_struct(&intel_fb->base, mode_cmd);
	intel_fb->obj = obj;
	return 0;
}

static struct drm_framebuffer *
intel_user_framebuffer_create(struct drm_device *dev,
			      struct drm_file *filp,
			      struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_i915_gem_object *obj;

	obj = to_intel_bo(drm_gem_object_lookup(dev, filp,
						mode_cmd->handles[0]));
	if (&obj->base == NULL)
		return ERR_PTR(-ENOENT);

	return intel_framebuffer_create(dev, mode_cmd, obj);
}

static const struct drm_mode_config_funcs intel_mode_funcs = {
	.fb_create = intel_user_framebuffer_create,
	.output_poll_changed = intel_fb_output_poll_changed,
};

static struct drm_i915_gem_object *
intel_alloc_context_page(struct drm_device *dev)
{
	struct drm_i915_gem_object *ctx;
	int ret;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	ctx = i915_gem_alloc_object(dev, 4096);
	if (!ctx) {
		DRM_DEBUG("failed to alloc power context, RC6 disabled\n");
		return NULL;
	}

	ret = i915_gem_object_pin(ctx, 4096, true);
	if (ret) {
		DRM_ERROR("failed to pin power context: %d\n", ret);
		goto err_unref;
	}

	ret = i915_gem_object_set_to_gtt_domain(ctx, 1);
	if (ret) {
		DRM_ERROR("failed to set-domain on power context: %d\n", ret);
		goto err_unpin;
	}

	return ctx;

err_unpin:
	i915_gem_object_unpin(ctx);
err_unref:
	drm_gem_object_unreference(&ctx->base);
	mutex_unlock(&dev->struct_mutex);
	return NULL;
}

bool ironlake_set_drps(struct drm_device *dev, u8 val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u16 rgvswctl;

	rgvswctl = I915_READ16(MEMSWCTL);
	if (rgvswctl & MEMCTL_CMD_STS) {
		DRM_DEBUG("gpu busy, RCS change rejected\n");
		return false; /* still busy with another command */
	}

	rgvswctl = (MEMCTL_CMD_CHFREQ << MEMCTL_CMD_SHIFT) |
		(val << MEMCTL_FREQ_SHIFT) | MEMCTL_SFCAVM;
	I915_WRITE16(MEMSWCTL, rgvswctl);
	POSTING_READ16(MEMSWCTL);

	rgvswctl |= MEMCTL_CMD_STS;
	I915_WRITE16(MEMSWCTL, rgvswctl);

	return true;
}

void ironlake_enable_drps(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 rgvmodectl = I915_READ(MEMMODECTL);
	u8 fmax, fmin, fstart, vstart;

	/* Enable temp reporting */
	I915_WRITE16(PMMISC, I915_READ(PMMISC) | MCPPCE_EN);
	I915_WRITE16(TSC1, I915_READ(TSC1) | TSE);

	/* 100ms RC evaluation intervals */
	I915_WRITE(RCUPEI, 100000);
	I915_WRITE(RCDNEI, 100000);

	/* Set max/min thresholds to 90ms and 80ms respectively */
	I915_WRITE(RCBMAXAVG, 90000);
	I915_WRITE(RCBMINAVG, 80000);

	I915_WRITE(MEMIHYST, 1);

	/* Set up min, max, and cur for interrupt handling */
	fmax = (rgvmodectl & MEMMODE_FMAX_MASK) >> MEMMODE_FMAX_SHIFT;
	fmin = (rgvmodectl & MEMMODE_FMIN_MASK);
	fstart = (rgvmodectl & MEMMODE_FSTART_MASK) >>
		MEMMODE_FSTART_SHIFT;

	vstart = (I915_READ(PXVFREQ_BASE + (fstart * 4)) & PXVFREQ_PX_MASK) >>
		PXVFREQ_PX_SHIFT;

	dev_priv->fmax = fmax; /* IPS callback will increase this */
	dev_priv->fstart = fstart;

	dev_priv->max_delay = fstart;
	dev_priv->min_delay = fmin;
	dev_priv->cur_delay = fstart;

	DRM_DEBUG_DRIVER("fmax: %d, fmin: %d, fstart: %d\n",
			 fmax, fmin, fstart);

	I915_WRITE(MEMINTREN, MEMINT_CX_SUPR_EN | MEMINT_EVAL_CHG_EN);

	/*
	 * Interrupts will be enabled in ironlake_irq_postinstall
	 */

	I915_WRITE(VIDSTART, vstart);
	POSTING_READ(VIDSTART);

	rgvmodectl |= MEMMODE_SWMODE_EN;
	I915_WRITE(MEMMODECTL, rgvmodectl);

	if (wait_for((I915_READ(MEMSWCTL) & MEMCTL_CMD_STS) == 0, 10))
		DRM_ERROR("stuck trying to change perf mode\n");
	msleep(1);

	ironlake_set_drps(dev, fstart);

	dev_priv->last_count1 = I915_READ(0x112e4) + I915_READ(0x112e8) +
		I915_READ(0x112e0);
	dev_priv->last_time1 = jiffies_to_msecs(jiffies);
	dev_priv->last_count2 = I915_READ(0x112f4);
	getrawmonotonic(&dev_priv->last_time2);
}

void ironlake_disable_drps(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u16 rgvswctl = I915_READ16(MEMSWCTL);

	/* Ack interrupts, disable EFC interrupt */
	I915_WRITE(MEMINTREN, I915_READ(MEMINTREN) & ~MEMINT_EVAL_CHG_EN);
	I915_WRITE(MEMINTRSTS, MEMINT_EVAL_CHG);
	I915_WRITE(DEIER, I915_READ(DEIER) & ~DE_PCU_EVENT);
	I915_WRITE(DEIIR, DE_PCU_EVENT);
	I915_WRITE(DEIMR, I915_READ(DEIMR) | DE_PCU_EVENT);

	/* Go back to the starting frequency */
	ironlake_set_drps(dev, dev_priv->fstart);
	msleep(1);
	rgvswctl |= MEMCTL_CMD_STS;
	I915_WRITE(MEMSWCTL, rgvswctl);
	msleep(1);

}

void gen6_set_rps(struct drm_device *dev, u8 val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 swreq;

	swreq = (val & 0x3ff) << 25;
	I915_WRITE(GEN6_RPNSWREQ, swreq);
}

void gen6_disable_rps(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE(GEN6_RPNSWREQ, 1 << 31);
	I915_WRITE(GEN6_PMINTRMSK, 0xffffffff);
	I915_WRITE(GEN6_PMIER, 0);
	/* Complete PM interrupt masking here doesn't race with the rps work
	 * item again unmasking PM interrupts because that is using a different
	 * register (PMIMR) to mask PM interrupts. The only risk is in leaving
	 * stale bits in PMIIR and PMIMR which gen6_enable_rps will clean up. */

	spin_lock_irq(&dev_priv->rps_lock);
	dev_priv->pm_iir = 0;
	spin_unlock_irq(&dev_priv->rps_lock);

	I915_WRITE(GEN6_PMIIR, I915_READ(GEN6_PMIIR));
}

static unsigned long intel_pxfreq(u32 vidfreq)
{
	unsigned long freq;
	int div = (vidfreq & 0x3f0000) >> 16;
	int post = (vidfreq & 0x3000) >> 12;
	int pre = (vidfreq & 0x7);

	if (!pre)
		return 0;

	freq = ((div * 133333) / ((1<<post) * pre));

	return freq;
}

void intel_init_emon(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 lcfuse;
	u8 pxw[16];
	int i;

	/* Disable to program */
	I915_WRITE(ECR, 0);
	POSTING_READ(ECR);

	/* Program energy weights for various events */
	I915_WRITE(SDEW, 0x15040d00);
	I915_WRITE(CSIEW0, 0x007f0000);
	I915_WRITE(CSIEW1, 0x1e220004);
	I915_WRITE(CSIEW2, 0x04000004);

	for (i = 0; i < 5; i++)
		I915_WRITE(PEW + (i * 4), 0);
	for (i = 0; i < 3; i++)
		I915_WRITE(DEW + (i * 4), 0);

	/* Program P-state weights to account for frequency power adjustment */
	for (i = 0; i < 16; i++) {
		u32 pxvidfreq = I915_READ(PXVFREQ_BASE + (i * 4));
		unsigned long freq = intel_pxfreq(pxvidfreq);
		unsigned long vid = (pxvidfreq & PXVFREQ_PX_MASK) >>
			PXVFREQ_PX_SHIFT;
		unsigned long val;

		val = vid * vid;
		val *= (freq / 1000);
		val *= 255;
		val /= (127*127*900);
		if (val > 0xff)
			DRM_ERROR("bad pxval: %ld\n", val);
		pxw[i] = val;
	}
	/* Render standby states get 0 weight */
	pxw[14] = 0;
	pxw[15] = 0;

	for (i = 0; i < 4; i++) {
		u32 val = (pxw[i*4] << 24) | (pxw[(i*4)+1] << 16) |
			(pxw[(i*4)+2] << 8) | (pxw[(i*4)+3]);
		I915_WRITE(PXW + (i * 4), val);
	}

	/* Adjust magic regs to magic values (more experimental results) */
	I915_WRITE(OGW0, 0);
	I915_WRITE(OGW1, 0);
	I915_WRITE(EG0, 0x00007f00);
	I915_WRITE(EG1, 0x0000000e);
	I915_WRITE(EG2, 0x000e0000);
	I915_WRITE(EG3, 0x68000300);
	I915_WRITE(EG4, 0x42000000);
	I915_WRITE(EG5, 0x00140031);
	I915_WRITE(EG6, 0);
	I915_WRITE(EG7, 0);

	for (i = 0; i < 8; i++)
		I915_WRITE(PXWL + (i * 4), 0);

	/* Enable PMON + select events */
	I915_WRITE(ECR, 0x80000019);

	lcfuse = I915_READ(LCFUSE02);

	dev_priv->corr = (lcfuse & LCFUSE_HIV_MASK);
}

static int intel_enable_rc6(struct drm_device *dev)
{
	/*
	 * Respect the kernel parameter if it is set
	 */
	if (i915_enable_rc6 >= 0)
		return i915_enable_rc6;

	/*
	 * Disable RC6 on Ironlake
	 */
	if (INTEL_INFO(dev)->gen == 5)
		return 0;

	/*
	 * Disable rc6 on Sandybridge
	 */
	if (INTEL_INFO(dev)->gen == 6) {
		DRM_DEBUG_DRIVER("Sandybridge: deep RC6 disabled\n");
		return INTEL_RC6_ENABLE;
	}
	DRM_DEBUG_DRIVER("RC6 and deep RC6 enabled\n");
	return (INTEL_RC6_ENABLE | INTEL_RC6p_ENABLE);
}

void gen6_enable_rps(struct drm_i915_private *dev_priv)
{
	u32 rp_state_cap = I915_READ(GEN6_RP_STATE_CAP);
	u32 gt_perf_status = I915_READ(GEN6_GT_PERF_STATUS);
	u32 pcu_mbox, rc6_mask = 0;
	u32 gtfifodbg;
	int cur_freq, min_freq, max_freq;
	int rc6_mode;
	int i;

	/* Here begins a magic sequence of register writes to enable
	 * auto-downclocking.
	 *
	 * Perhaps there might be some value in exposing these to
	 * userspace...
	 */
	I915_WRITE(GEN6_RC_STATE, 0);
	mutex_lock(&dev_priv->dev->struct_mutex);

	/* Clear the DBG now so we don't confuse earlier errors */
	if ((gtfifodbg = I915_READ(GTFIFODBG))) {
		DRM_ERROR("GT fifo had a previous error %x\n", gtfifodbg);
		I915_WRITE(GTFIFODBG, gtfifodbg);
	}

	gen6_gt_force_wake_get(dev_priv);

	/* disable the counters and set deterministic thresholds */
	I915_WRITE(GEN6_RC_CONTROL, 0);

	I915_WRITE(GEN6_RC1_WAKE_RATE_LIMIT, 1000 << 16);
	I915_WRITE(GEN6_RC6_WAKE_RATE_LIMIT, 40 << 16 | 30);
	I915_WRITE(GEN6_RC6pp_WAKE_RATE_LIMIT, 30);
	I915_WRITE(GEN6_RC_EVALUATION_INTERVAL, 125000);
	I915_WRITE(GEN6_RC_IDLE_HYSTERSIS, 25);

	for (i = 0; i < I915_NUM_RINGS; i++)
		I915_WRITE(RING_MAX_IDLE(dev_priv->ring[i].mmio_base), 10);

	I915_WRITE(GEN6_RC_SLEEP, 0);
	I915_WRITE(GEN6_RC1e_THRESHOLD, 1000);
	I915_WRITE(GEN6_RC6_THRESHOLD, 50000);
	I915_WRITE(GEN6_RC6p_THRESHOLD, 150000);
	I915_WRITE(GEN6_RC6pp_THRESHOLD, 64000); /* unused */

	rc6_mode = intel_enable_rc6(dev_priv->dev);
	if (rc6_mode & INTEL_RC6_ENABLE)
		rc6_mask |= GEN6_RC_CTL_RC6_ENABLE;

	if (rc6_mode & INTEL_RC6p_ENABLE)
		rc6_mask |= GEN6_RC_CTL_RC6p_ENABLE;

	if (rc6_mode & INTEL_RC6pp_ENABLE)
		rc6_mask |= GEN6_RC_CTL_RC6pp_ENABLE;

	DRM_INFO("Enabling RC6 states: RC6 %s, RC6p %s, RC6pp %s\n",
			(rc6_mode & INTEL_RC6_ENABLE) ? "on" : "off",
			(rc6_mode & INTEL_RC6p_ENABLE) ? "on" : "off",
			(rc6_mode & INTEL_RC6pp_ENABLE) ? "on" : "off");

	I915_WRITE(GEN6_RC_CONTROL,
		   rc6_mask |
		   GEN6_RC_CTL_EI_MODE(1) |
		   GEN6_RC_CTL_HW_ENABLE);

	I915_WRITE(GEN6_RPNSWREQ,
		   GEN6_FREQUENCY(10) |
		   GEN6_OFFSET(0) |
		   GEN6_AGGRESSIVE_TURBO);
	I915_WRITE(GEN6_RC_VIDEO_FREQ,
		   GEN6_FREQUENCY(12));

	I915_WRITE(GEN6_RP_DOWN_TIMEOUT, 1000000);
	I915_WRITE(GEN6_RP_INTERRUPT_LIMITS,
		   18 << 24 |
		   6 << 16);
	I915_WRITE(GEN6_RP_UP_THRESHOLD, 10000);
	I915_WRITE(GEN6_RP_DOWN_THRESHOLD, 1000000);
	I915_WRITE(GEN6_RP_UP_EI, 100000);
	I915_WRITE(GEN6_RP_DOWN_EI, 5000000);
	I915_WRITE(GEN6_RP_IDLE_HYSTERSIS, 10);
	I915_WRITE(GEN6_RP_CONTROL,
		   GEN6_RP_MEDIA_TURBO |
		   GEN6_RP_MEDIA_HW_NORMAL_MODE |
		   GEN6_RP_MEDIA_IS_GFX |
		   GEN6_RP_ENABLE |
		   GEN6_RP_UP_BUSY_AVG |
		   GEN6_RP_DOWN_IDLE_CONT);

	if (wait_for((I915_READ(GEN6_PCODE_MAILBOX) & GEN6_PCODE_READY) == 0,
		     500))
		DRM_ERROR("timeout waiting for pcode mailbox to become idle\n");

	I915_WRITE(GEN6_PCODE_DATA, 0);
	I915_WRITE(GEN6_PCODE_MAILBOX,
		   GEN6_PCODE_READY |
		   GEN6_PCODE_WRITE_MIN_FREQ_TABLE);
	if (wait_for((I915_READ(GEN6_PCODE_MAILBOX) & GEN6_PCODE_READY) == 0,
		     500))
		DRM_ERROR("timeout waiting for pcode mailbox to finish\n");

	min_freq = (rp_state_cap & 0xff0000) >> 16;
	max_freq = rp_state_cap & 0xff;
	cur_freq = (gt_perf_status & 0xff00) >> 8;

	/* Check for overclock support */
	if (wait_for((I915_READ(GEN6_PCODE_MAILBOX) & GEN6_PCODE_READY) == 0,
		     500))
		DRM_ERROR("timeout waiting for pcode mailbox to become idle\n");
	I915_WRITE(GEN6_PCODE_MAILBOX, GEN6_READ_OC_PARAMS);
	pcu_mbox = I915_READ(GEN6_PCODE_DATA);
	if (wait_for((I915_READ(GEN6_PCODE_MAILBOX) & GEN6_PCODE_READY) == 0,
		     500))
		DRM_ERROR("timeout waiting for pcode mailbox to finish\n");
	if (pcu_mbox & (1<<31)) { /* OC supported */
		max_freq = pcu_mbox & 0xff;
		DRM_DEBUG_DRIVER("overclocking supported, adjusting frequency max to %dMHz\n", pcu_mbox * 50);
	}

	/* In units of 100MHz */
	dev_priv->max_delay = max_freq;
	dev_priv->min_delay = min_freq;
	dev_priv->cur_delay = cur_freq;

	/* requires MSI enabled */
	I915_WRITE(GEN6_PMIER,
		   GEN6_PM_MBOX_EVENT |
		   GEN6_PM_THERMAL_EVENT |
		   GEN6_PM_RP_DOWN_TIMEOUT |
		   GEN6_PM_RP_UP_THRESHOLD |
		   GEN6_PM_RP_DOWN_THRESHOLD |
		   GEN6_PM_RP_UP_EI_EXPIRED |
		   GEN6_PM_RP_DOWN_EI_EXPIRED);
	spin_lock_irq(&dev_priv->rps_lock);
	WARN_ON(dev_priv->pm_iir != 0);
	I915_WRITE(GEN6_PMIMR, 0);
	spin_unlock_irq(&dev_priv->rps_lock);
	/* enable all PM interrupts */
	I915_WRITE(GEN6_PMINTRMSK, 0);

	gen6_gt_force_wake_put(dev_priv);
	mutex_unlock(&dev_priv->dev->struct_mutex);
}

void gen6_update_ring_freq(struct drm_i915_private *dev_priv)
{
	int min_freq = 15;
	int gpu_freq, ia_freq, max_ia_freq;
	int scaling_factor = 180;

	max_ia_freq = cpufreq_quick_get_max(0);
	/*
	 * Default to measured freq if none found, PCU will ensure we don't go
	 * over
	 */
	if (!max_ia_freq)
		max_ia_freq = tsc_khz;

	/* Convert from kHz to MHz */
	max_ia_freq /= 1000;

	mutex_lock(&dev_priv->dev->struct_mutex);

	/*
	 * For each potential GPU frequency, load a ring frequency we'd like
	 * to use for memory access.  We do this by specifying the IA frequency
	 * the PCU should use as a reference to determine the ring frequency.
	 */
	for (gpu_freq = dev_priv->max_delay; gpu_freq >= dev_priv->min_delay;
	     gpu_freq--) {
		int diff = dev_priv->max_delay - gpu_freq;

		/*
		 * For GPU frequencies less than 750MHz, just use the lowest
		 * ring freq.
		 */
		if (gpu_freq < min_freq)
			ia_freq = 800;
		else
			ia_freq = max_ia_freq - ((diff * scaling_factor) / 2);
		ia_freq = DIV_ROUND_CLOSEST(ia_freq, 100);

		I915_WRITE(GEN6_PCODE_DATA,
			   (ia_freq << GEN6_PCODE_FREQ_IA_RATIO_SHIFT) |
			   gpu_freq);
		I915_WRITE(GEN6_PCODE_MAILBOX, GEN6_PCODE_READY |
			   GEN6_PCODE_WRITE_MIN_FREQ_TABLE);
		if (wait_for((I915_READ(GEN6_PCODE_MAILBOX) &
			      GEN6_PCODE_READY) == 0, 10)) {
			DRM_ERROR("pcode write of freq table timed out\n");
			continue;
		}
	}

	mutex_unlock(&dev_priv->dev->struct_mutex);
}

static void ironlake_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t dspclk_gate = VRHUNIT_CLOCK_GATE_DISABLE;

	/* Required for FBC */
	dspclk_gate |= DPFCUNIT_CLOCK_GATE_DISABLE |
		DPFCRUNIT_CLOCK_GATE_DISABLE |
		DPFDUNIT_CLOCK_GATE_DISABLE;
	/* Required for CxSR */
	dspclk_gate |= DPARBUNIT_CLOCK_GATE_DISABLE;

	I915_WRITE(PCH_3DCGDIS0,
		   MARIUNIT_CLOCK_GATE_DISABLE |
		   SVSMUNIT_CLOCK_GATE_DISABLE);
	I915_WRITE(PCH_3DCGDIS1,
		   VFMUNIT_CLOCK_GATE_DISABLE);

	I915_WRITE(PCH_DSPCLK_GATE_D, dspclk_gate);

	/*
	 * According to the spec the following bits should be set in
	 * order to enable memory self-refresh
	 * The bit 22/21 of 0x42004
	 * The bit 5 of 0x42020
	 * The bit 15 of 0x45000
	 */
	I915_WRITE(ILK_DISPLAY_CHICKEN2,
		   (I915_READ(ILK_DISPLAY_CHICKEN2) |
		    ILK_DPARB_GATE | ILK_VSDPFD_FULL));
	I915_WRITE(ILK_DSPCLK_GATE,
		   (I915_READ(ILK_DSPCLK_GATE) |
		    ILK_DPARB_CLK_GATE));
	I915_WRITE(DISP_ARB_CTL,
		   (I915_READ(DISP_ARB_CTL) |
		    DISP_FBC_WM_DIS));
	I915_WRITE(WM3_LP_ILK, 0);
	I915_WRITE(WM2_LP_ILK, 0);
	I915_WRITE(WM1_LP_ILK, 0);

	/*
	 * Based on the document from hardware guys the following bits
	 * should be set unconditionally in order to enable FBC.
	 * The bit 22 of 0x42000
	 * The bit 22 of 0x42004
	 * The bit 7,8,9 of 0x42020.
	 */
	if (IS_IRONLAKE_M(dev)) {
		I915_WRITE(ILK_DISPLAY_CHICKEN1,
			   I915_READ(ILK_DISPLAY_CHICKEN1) |
			   ILK_FBCQ_DIS);
		I915_WRITE(ILK_DISPLAY_CHICKEN2,
			   I915_READ(ILK_DISPLAY_CHICKEN2) |
			   ILK_DPARB_GATE);
		I915_WRITE(ILK_DSPCLK_GATE,
			   I915_READ(ILK_DSPCLK_GATE) |
			   ILK_DPFC_DIS1 |
			   ILK_DPFC_DIS2 |
			   ILK_CLK_FBC);
	}

	I915_WRITE(ILK_DISPLAY_CHICKEN2,
		   I915_READ(ILK_DISPLAY_CHICKEN2) |
		   ILK_ELPIN_409_SELECT);
	I915_WRITE(_3D_CHICKEN2,
		   _3D_CHICKEN2_WM_READ_PIPELINED << 16 |
		   _3D_CHICKEN2_WM_READ_PIPELINED);
}

static void gen6_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;
	uint32_t dspclk_gate = VRHUNIT_CLOCK_GATE_DISABLE;

	I915_WRITE(PCH_DSPCLK_GATE_D, dspclk_gate);

	I915_WRITE(ILK_DISPLAY_CHICKEN2,
		   I915_READ(ILK_DISPLAY_CHICKEN2) |
		   ILK_ELPIN_409_SELECT);

	/* WaDisableHiZPlanesWhenMSAAEnabled */
	I915_WRITE(_3D_CHICKEN,
		   _MASKED_BIT_ENABLE(_3D_CHICKEN_HIZ_PLANE_DISABLE_MSAA_4X_SNB));

	I915_WRITE(WM3_LP_ILK, 0);
	I915_WRITE(WM2_LP_ILK, 0);
	I915_WRITE(WM1_LP_ILK, 0);

	I915_WRITE(GEN6_UCGCTL1,
		   I915_READ(GEN6_UCGCTL1) |
		   GEN6_BLBUNIT_CLOCK_GATE_DISABLE);

	/* According to the BSpec vol1g, bit 12 (RCPBUNIT) clock
	 * gating disable must be set.  Failure to set it results in
	 * flickering pixels due to Z write ordering failures after
	 * some amount of runtime in the Mesa "fire" demo, and Unigine
	 * Sanctuary and Tropics, and apparently anything else with
	 * alpha test or pixel discard.
	 *
	 * According to the spec, bit 11 (RCCUNIT) must also be set,
	 * but we didn't debug actual testcases to find it out.
	 */
	I915_WRITE(GEN6_UCGCTL2,
		   GEN6_RCPBUNIT_CLOCK_GATE_DISABLE |
		   GEN6_RCCUNIT_CLOCK_GATE_DISABLE);

	/*
	 * According to the spec the following bits should be
	 * set in order to enable memory self-refresh and fbc:
	 * The bit21 and bit22 of 0x42000
	 * The bit21 and bit22 of 0x42004
	 * The bit5 and bit7 of 0x42020
	 * The bit14 of 0x70180
	 * The bit14 of 0x71180
	 */
	I915_WRITE(ILK_DISPLAY_CHICKEN1,
		   I915_READ(ILK_DISPLAY_CHICKEN1) |
		   ILK_FBCQ_DIS | ILK_PABSTRETCH_DIS);
	I915_WRITE(ILK_DISPLAY_CHICKEN2,
		   I915_READ(ILK_DISPLAY_CHICKEN2) |
		   ILK_DPARB_GATE | ILK_VSDPFD_FULL);
	I915_WRITE(ILK_DSPCLK_GATE,
		   I915_READ(ILK_DSPCLK_GATE) |
		   ILK_DPARB_CLK_GATE  |
		   ILK_DPFD_CLK_GATE);

	for_each_pipe(pipe) {
		I915_WRITE(DSPCNTR(pipe),
			   I915_READ(DSPCNTR(pipe)) |
			   DISPPLANE_TRICKLE_FEED_DISABLE);
		intel_flush_display_plane(dev_priv, pipe);
	}

	/* The default value should be 0x200 according to docs, but the two
	 * platforms I checked have a 0 for this. (Maybe BIOS overrides?) */
	I915_WRITE(GEN6_GT_MODE, 0xffff << 16);
	I915_WRITE(GEN6_GT_MODE, GEN6_GT_MODE_HI << 16 | GEN6_GT_MODE_HI);
}

static void gen7_setup_fixed_func_scheduler(struct drm_i915_private *dev_priv)
{
	uint32_t reg = I915_READ(GEN7_FF_THREAD_MODE);

	reg &= ~GEN7_FF_SCHED_MASK;
	reg |= GEN7_FF_TS_SCHED_HW;
	reg |= GEN7_FF_VS_SCHED_HW;
	reg |= GEN7_FF_DS_SCHED_HW;

	I915_WRITE(GEN7_FF_THREAD_MODE, reg);
}

static void ivybridge_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;
	uint32_t dspclk_gate = VRHUNIT_CLOCK_GATE_DISABLE;

	I915_WRITE(PCH_DSPCLK_GATE_D, dspclk_gate);

	I915_WRITE(WM3_LP_ILK, 0);
	I915_WRITE(WM2_LP_ILK, 0);
	I915_WRITE(WM1_LP_ILK, 0);

	/* According to the spec, bit 13 (RCZUNIT) must be set on IVB.
	 * This implements the WaDisableRCZUnitClockGating workaround.
	 */
	I915_WRITE(GEN6_UCGCTL2, GEN6_RCZUNIT_CLOCK_GATE_DISABLE);

	I915_WRITE(ILK_DSPCLK_GATE, IVB_VRHUNIT_CLK_GATE);

	I915_WRITE(IVB_CHICKEN3,
		   CHICKEN3_DGMG_REQ_OUT_FIX_DISABLE |
		   CHICKEN3_DGMG_DONE_FIX_DISABLE);

	/* Apply the WaDisableRHWOOptimizationForRenderHang workaround. */
	I915_WRITE(GEN7_COMMON_SLICE_CHICKEN1,
		   GEN7_CSC1_RHWO_OPT_DISABLE_IN_RCC);

	/* WaApplyL3ControlAndL3ChickenMode requires those two on Ivy Bridge */
	I915_WRITE(GEN7_L3CNTLREG1,
			GEN7_WA_FOR_GEN7_L3_CONTROL);
	I915_WRITE(GEN7_L3_CHICKEN_MODE_REGISTER,
			GEN7_WA_L3_CHICKEN_MODE);

	/* This is required by WaCatErrorRejectionIssue */
	I915_WRITE(GEN7_SQ_CHICKEN_MBCUNIT_CONFIG,
			I915_READ(GEN7_SQ_CHICKEN_MBCUNIT_CONFIG) |
			GEN7_SQ_CHICKEN_MBCUNIT_SQINTMOB);

	for_each_pipe(pipe) {
		I915_WRITE(DSPCNTR(pipe),
			   I915_READ(DSPCNTR(pipe)) |
			   DISPPLANE_TRICKLE_FEED_DISABLE);
		intel_flush_display_plane(dev_priv, pipe);
	}

	gen7_setup_fixed_func_scheduler(dev_priv);
}

static void g4x_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t dspclk_gate;

	I915_WRITE(RENCLK_GATE_D1, 0);
	I915_WRITE(RENCLK_GATE_D2, VF_UNIT_CLOCK_GATE_DISABLE |
		   GS_UNIT_CLOCK_GATE_DISABLE |
		   CL_UNIT_CLOCK_GATE_DISABLE);
	I915_WRITE(RAMCLK_GATE_D, 0);
	dspclk_gate = VRHUNIT_CLOCK_GATE_DISABLE |
		OVRUNIT_CLOCK_GATE_DISABLE |
		OVCUNIT_CLOCK_GATE_DISABLE;
	if (IS_GM45(dev))
		dspclk_gate |= DSSUNIT_CLOCK_GATE_DISABLE;
	I915_WRITE(DSPCLK_GATE_D, dspclk_gate);
}

static void crestline_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE(RENCLK_GATE_D1, I965_RCC_CLOCK_GATE_DISABLE);
	I915_WRITE(RENCLK_GATE_D2, 0);
	I915_WRITE(DSPCLK_GATE_D, 0);
	I915_WRITE(RAMCLK_GATE_D, 0);
	I915_WRITE16(DEUC, 0);
}

static void broadwater_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE(RENCLK_GATE_D1, I965_RCZ_CLOCK_GATE_DISABLE |
		   I965_RCC_CLOCK_GATE_DISABLE |
		   I965_RCPB_CLOCK_GATE_DISABLE |
		   I965_ISC_CLOCK_GATE_DISABLE |
		   I965_FBC_CLOCK_GATE_DISABLE);
	I915_WRITE(RENCLK_GATE_D2, 0);
}

static void gen3_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 dstate = I915_READ(D_STATE);

	dstate |= DSTATE_PLL_D3_OFF | DSTATE_GFX_CLOCK_GATING |
		DSTATE_DOT_CLOCK_GATING;
	I915_WRITE(D_STATE, dstate);
}

static void i85x_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE(RENCLK_GATE_D1, SV_CLOCK_GATE_DISABLE);
}

static void i830_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE(DSPCLK_GATE_D, OVRUNIT_CLOCK_GATE_DISABLE);
}

static void ibx_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/*
	 * On Ibex Peak and Cougar Point, we need to disable clock
	 * gating for the panel power sequencer or it will fail to
	 * start up when no ports are active.
	 */
	I915_WRITE(SOUTH_DSPCLK_GATE_D, PCH_DPLSUNIT_CLOCK_GATE_DISABLE);
}

static void cpt_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	/*
	 * On Ibex Peak and Cougar Point, we need to disable clock
	 * gating for the panel power sequencer or it will fail to
	 * start up when no ports are active.
	 */
	I915_WRITE(SOUTH_DSPCLK_GATE_D, PCH_DPLSUNIT_CLOCK_GATE_DISABLE);
	I915_WRITE(SOUTH_CHICKEN2, I915_READ(SOUTH_CHICKEN2) |
		   DPLS_EDP_PPS_FIX_DIS);
	/* Without this, mode sets may fail silently on FDI */
	for_each_pipe(pipe)
		I915_WRITE(TRANS_CHICKEN2(pipe), TRANS_AUTOTRAIN_GEN_STALL_DIS);
}

static void ironlake_teardown_rc6(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (dev_priv->renderctx) {
		i915_gem_object_unpin(dev_priv->renderctx);
		drm_gem_object_unreference(&dev_priv->renderctx->base);
		dev_priv->renderctx = NULL;
	}

	if (dev_priv->pwrctx) {
		i915_gem_object_unpin(dev_priv->pwrctx);
		drm_gem_object_unreference(&dev_priv->pwrctx->base);
		dev_priv->pwrctx = NULL;
	}
}

static void ironlake_disable_rc6(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (I915_READ(PWRCTXA)) {
		/* Wake the GPU, prevent RC6, then restore RSTDBYCTL */
		I915_WRITE(RSTDBYCTL, I915_READ(RSTDBYCTL) | RCX_SW_EXIT);
		wait_for(((I915_READ(RSTDBYCTL) & RSX_STATUS_MASK) == RSX_STATUS_ON),
			 50);

		I915_WRITE(PWRCTXA, 0);
		POSTING_READ(PWRCTXA);

		I915_WRITE(RSTDBYCTL, I915_READ(RSTDBYCTL) & ~RCX_SW_EXIT);
		POSTING_READ(RSTDBYCTL);
	}

	ironlake_teardown_rc6(dev);
}

static int ironlake_setup_rc6(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (dev_priv->renderctx == NULL)
		dev_priv->renderctx = intel_alloc_context_page(dev);
	if (!dev_priv->renderctx)
		return -ENOMEM;

	if (dev_priv->pwrctx == NULL)
		dev_priv->pwrctx = intel_alloc_context_page(dev);
	if (!dev_priv->pwrctx) {
		ironlake_teardown_rc6(dev);
		return -ENOMEM;
	}

	return 0;
}

void ironlake_enable_rc6(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	/* rc6 disabled by default due to repeated reports of hanging during
	 * boot and resume.
	 */
	if (!intel_enable_rc6(dev))
		return;

	mutex_lock(&dev->struct_mutex);
	ret = ironlake_setup_rc6(dev);
	if (ret) {
		mutex_unlock(&dev->struct_mutex);
		return;
	}

	/*
	 * GPU can automatically power down the render unit if given a page
	 * to save state.
	 */
	ret = BEGIN_LP_RING(6);
	if (ret) {
		ironlake_teardown_rc6(dev);
		mutex_unlock(&dev->struct_mutex);
		return;
	}

	OUT_RING(MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN);
	OUT_RING(MI_SET_CONTEXT);
	OUT_RING(dev_priv->renderctx->gtt_offset |
		 MI_MM_SPACE_GTT |
		 MI_SAVE_EXT_STATE_EN |
		 MI_RESTORE_EXT_STATE_EN |
		 MI_RESTORE_INHIBIT);
	OUT_RING(MI_SUSPEND_FLUSH);
	OUT_RING(MI_NOOP);
	OUT_RING(MI_FLUSH);
	ADVANCE_LP_RING();

	/*
	 * Wait for the command parser to advance past MI_SET_CONTEXT. The HW
	 * does an implicit flush, combined with MI_FLUSH above, it should be
	 * safe to assume that renderctx is valid
	 */
	ret = intel_wait_ring_idle(LP_RING(dev_priv));
	if (ret) {
		DRM_ERROR("failed to enable ironlake power power savings\n");
		ironlake_teardown_rc6(dev);
		mutex_unlock(&dev->struct_mutex);
		return;
	}

	I915_WRITE(PWRCTXA, dev_priv->pwrctx->gtt_offset | PWRCTX_EN);
	I915_WRITE(RSTDBYCTL, I915_READ(RSTDBYCTL) & ~RCX_SW_EXIT);
	mutex_unlock(&dev->struct_mutex);
}

void intel_init_clock_gating(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->display.init_clock_gating(dev);

	if (dev_priv->display.init_pch_clock_gating)
		dev_priv->display.init_pch_clock_gating(dev);
}

/* Set up chip specific display functions */
static void intel_init_display(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* We always want a DPMS function */
	if (IS_HASWELL(dev)) {
		dev_priv->display.crtc_mode_set = haswell_crtc_mode_set;
		dev_priv->display.crtc_enable = haswell_crtc_enable;
		dev_priv->display.crtc_disable = haswell_crtc_disable;
		dev_priv->display.off = haswell_crtc_off;
		dev_priv->display.update_plane = ironlake_update_plane;
	} else if (HAS_PCH_SPLIT(dev)) {
		dev_priv->display.crtc_mode_set = ironlake_crtc_mode_set;
		dev_priv->display.crtc_enable = ironlake_crtc_enable;
		dev_priv->display.crtc_disable = ironlake_crtc_disable;
		dev_priv->display.off = ironlake_crtc_off;
		dev_priv->display.update_plane = ironlake_update_plane;
	} else {
		dev_priv->display.crtc_mode_set = i9xx_crtc_mode_set;
		dev_priv->display.crtc_enable = i9xx_crtc_enable;
		dev_priv->display.crtc_disable = i9xx_crtc_disable;
		dev_priv->display.off = i9xx_crtc_off;
		dev_priv->display.update_plane = i9xx_update_plane;
	}

	/* Returns the core display clock speed */
	if (IS_VALLEYVIEW(dev))
		dev_priv->display.get_display_clock_speed =
			valleyview_get_display_clock_speed;
	else if (IS_I945G(dev) || (IS_G33(dev) && !IS_PINEVIEW_M(dev)))
		dev_priv->display.get_display_clock_speed =
			i945_get_display_clock_speed;
	else if (IS_I915G(dev))
		dev_priv->display.get_display_clock_speed =
			i915_get_display_clock_speed;
	else if (IS_I945GM(dev) || IS_845G(dev) || IS_PINEVIEW_M(dev))
		dev_priv->display.get_display_clock_speed =
			i9xx_misc_get_display_clock_speed;
	else if (IS_I915GM(dev))
		dev_priv->display.get_display_clock_speed =
			i915gm_get_display_clock_speed;
	else if (IS_I865G(dev))
		dev_priv->display.get_display_clock_speed =
			i865_get_display_clock_speed;
	else if (IS_I85X(dev))
		dev_priv->display.get_display_clock_speed =
			i855_get_display_clock_speed;
	else /* 852, 830 */
		dev_priv->display.get_display_clock_speed =
			i830_get_display_clock_speed;

	if (HAS_PCH_SPLIT(dev)) {
		if (IS_GEN5(dev)) {
			dev_priv->display.fdi_link_train = ironlake_fdi_link_train;
			dev_priv->display.write_eld = ironlake_write_eld;
		} else if (IS_GEN6(dev)) {
			dev_priv->display.fdi_link_train = gen6_fdi_link_train;
			dev_priv->display.write_eld = ironlake_write_eld;
		} else if (IS_IVYBRIDGE(dev)) {
			/* FIXME: detect B0+ stepping and use auto training */
			dev_priv->display.fdi_link_train = ivb_manual_fdi_link_train;
			dev_priv->display.write_eld = ironlake_write_eld;
			dev_priv->display.modeset_global_resources =
				ivb_modeset_global_resources;
		} else if (IS_HASWELL(dev)) {
			dev_priv->display.fdi_link_train = hsw_fdi_link_train;
			dev_priv->display.write_eld = haswell_write_eld;
		} else
			dev_priv->display.update_wm = NULL;
	} else if (IS_G4X(dev)) {
		dev_priv->display.write_eld = g4x_write_eld;
	}

	/* Default just returns -ENODEV to indicate unsupported */
	dev_priv->display.queue_flip = intel_default_queue_flip;

	switch (INTEL_INFO(dev)->gen) {
	case 2:
		dev_priv->display.queue_flip = intel_gen2_queue_flip;
		break;

	case 3:
		dev_priv->display.queue_flip = intel_gen3_queue_flip;
		break;

	case 4:
	case 5:
		dev_priv->display.queue_flip = intel_gen4_queue_flip;
		break;

	case 6:
		dev_priv->display.queue_flip = intel_gen6_queue_flip;
		break;
	case 7:
		dev_priv->display.queue_flip = intel_gen7_queue_flip;
		break;
	}
}

/*
 * Some BIOSes insist on assuming the GPU's pipe A is enabled at suspend,
 * resume, or other times.  This quirk makes sure that's the case for
 * affected systems.
 */
static void quirk_pipea_force(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->quirks |= QUIRK_PIPEA_FORCE;
	DRM_INFO("applying pipe a force quirk\n");
}

/*
 * Some machines (Lenovo U160) do not work with SSC on LVDS for some reason
 */
static void quirk_ssc_force_disable(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	dev_priv->quirks |= QUIRK_LVDS_SSC_DISABLE;
	DRM_INFO("applying lvds SSC disable quirk\n");
}

/*
 * A machine (e.g. Acer Aspire 5734Z) may need to invert the panel backlight
 * brightness value
 */
static void quirk_invert_brightness(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	dev_priv->quirks |= QUIRK_INVERT_BRIGHTNESS;
	DRM_INFO("applying inverted panel brightness quirk\n");
}

/*
 * Some machines (Dell XPS13) suffer broken backlight controls if
 * BLM_PCH_PWM_ENABLE is set.
 */
static void quirk_no_pcm_pwm_enable(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	dev_priv->quirks |= QUIRK_NO_PCH_PWM_ENABLE;
	DRM_INFO("applying no-PCH_PWM_ENABLE quirk\n");
}

/*
 * A machine (e.g. Acer Aspire 5734Z) may need to invert the panel backlight
 * brightness value
 */
static void quirk_invert_brightness(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	dev_priv->quirks |= QUIRK_INVERT_BRIGHTNESS;
}

struct intel_quirk {
	int device;
	int subsystem_vendor;
	int subsystem_device;
	void (*hook)(struct drm_device *dev);
};

/* For systems that don't have a meaningful PCI subdevice/subvendor ID */
struct intel_dmi_quirk {
	void (*hook)(struct drm_device *dev);
	const struct dmi_system_id (*dmi_id_list)[];
};

static int intel_dmi_reverse_brightness(const struct dmi_system_id *id)
{
	DRM_INFO("Backlight polarity reversed on %s\n", id->ident);
	return 1;
}

static const struct intel_dmi_quirk intel_dmi_quirks[] = {
	{
		.dmi_id_list = &(const struct dmi_system_id[]) {
			{
				.callback = intel_dmi_reverse_brightness,
				.ident = "NCR Corporation",
				.matches = {DMI_MATCH(DMI_SYS_VENDOR, "NCR Corporation"),
					    DMI_MATCH(DMI_PRODUCT_NAME, ""),
				},
			},
			{ }  /* terminating entry */
		},
		.hook = quirk_invert_brightness,
	},
};

/* For systems that don't have a meaningful PCI subdevice/subvendor ID */
struct intel_dmi_quirk {
	void (*hook)(struct drm_device *dev);
	const struct dmi_system_id (*dmi_id_list)[];
};

static int intel_dmi_reverse_brightness(const struct dmi_system_id *id)
{
	DRM_INFO("Backlight polarity reversed on %s\n", id->ident);
	return 1;
}

static const struct intel_dmi_quirk intel_dmi_quirks[] = {
	{
		.dmi_id_list = &(const struct dmi_system_id[]) {
			{
				.callback = intel_dmi_reverse_brightness,
				.ident = "NCR Corporation",
				.matches = {DMI_MATCH(DMI_SYS_VENDOR, "NCR Corporation"),
					    DMI_MATCH(DMI_PRODUCT_NAME, ""),
				},
			},
			{ }  /* terminating entry */
		},
		.hook = quirk_invert_brightness,
	},
};

static struct intel_quirk intel_quirks[] = {
	/* HP Mini needs pipe A force quirk (LP: #322104) */
	{ 0x27ae, 0x103c, 0x361a, quirk_pipea_force },

	/* Toshiba Protege R-205, S-209 needs pipe A force quirk */
	{ 0x2592, 0x1179, 0x0001, quirk_pipea_force },

	/* ThinkPad T60 needs pipe A force quirk (bug #16494) */
	{ 0x2782, 0x17aa, 0x201a, quirk_pipea_force },

	/* 830/845 need to leave pipe A & dpll A up */
	{ 0x2562, PCI_ANY_ID, PCI_ANY_ID, quirk_pipea_force },
	{ 0x3577, PCI_ANY_ID, PCI_ANY_ID, quirk_pipea_force },

	/* Lenovo U160 cannot use SSC on LVDS */
	{ 0x0046, 0x17aa, 0x3920, quirk_ssc_force_disable },

	/* Sony Vaio Y cannot use SSC on LVDS */
	{ 0x0046, 0x104d, 0x9076, quirk_ssc_force_disable },

	/* Acer Aspire 5734Z must invert backlight brightness */
	{ 0x2a42, 0x1025, 0x0459, quirk_invert_brightness },

	/* Acer/eMachines G725 */
	{ 0x2a42, 0x1025, 0x0210, quirk_invert_brightness },

	/* Acer/eMachines e725 */
	{ 0x2a42, 0x1025, 0x0212, quirk_invert_brightness },

	/* Acer/Packard Bell NCL20 */
	{ 0x2a42, 0x1025, 0x034b, quirk_invert_brightness },

	/* Dell XPS13 HD Sandy Bridge */
	{ 0x0116, 0x1028, 0x052e, quirk_no_pcm_pwm_enable },
	/* Dell XPS13 HD and XPS13 FHD Ivy Bridge */
	{ 0x0166, 0x1028, 0x058b, quirk_no_pcm_pwm_enable },
};

static void intel_init_quirks(struct drm_device *dev)
{
	struct pci_dev *d = dev->pdev;
	int i;

	for (i = 0; i < ARRAY_SIZE(intel_quirks); i++) {
		struct intel_quirk *q = &intel_quirks[i];

		if (d->device == q->device &&
		    (d->subsystem_vendor == q->subsystem_vendor ||
		     q->subsystem_vendor == PCI_ANY_ID) &&
		    (d->subsystem_device == q->subsystem_device ||
		     q->subsystem_device == PCI_ANY_ID))
			q->hook(dev);
	}
	for (i = 0; i < ARRAY_SIZE(intel_dmi_quirks); i++) {
		if (dmi_check_system(*intel_dmi_quirks[i].dmi_id_list) != 0)
			intel_dmi_quirks[i].hook(dev);
	}
	for (i = 0; i < ARRAY_SIZE(intel_dmi_quirks); i++) {
		if (dmi_check_system(*intel_dmi_quirks[i].dmi_id_list) != 0)
			intel_dmi_quirks[i].hook(dev);
	}
}

/* Disable the VGA plane that we never use */
static void i915_disable_vga(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u8 sr1;
	u32 vga_reg;

	if (HAS_PCH_SPLIT(dev))
		vga_reg = CPU_VGACNTRL;
	else
		vga_reg = VGACNTRL;

	vga_get_uninterruptible(dev->pdev, VGA_RSRC_LEGACY_IO);
	outb(SR01, VGA_SR_INDEX);
	sr1 = inb(VGA_SR_DATA);
	outb(sr1 | 1<<5, VGA_SR_DATA);
	vga_put(dev->pdev, VGA_RSRC_LEGACY_IO);
	udelay(300);

	I915_WRITE(vga_reg, VGA_DISP_DISABLE);
	POSTING_READ(vga_reg);
}

void i915_redisable_vga(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 vga_reg;

	if (HAS_PCH_SPLIT(dev))
		vga_reg = CPU_VGACNTRL;
	else
		vga_reg = VGACNTRL;

	if (I915_READ(vga_reg) != VGA_DISP_DISABLE) {
		DRM_DEBUG_KMS("Something enabled VGA plane, disabling it\n");
		I915_WRITE(vga_reg, VGA_DISP_DISABLE);
		POSTING_READ(vga_reg);
	}
}

void intel_modeset_init_hw(struct drm_device *dev)
{
	/* We attempt to init the necessary power wells early in the initialization
	 * time, so the subsystems that expect power to be enabled can work.
	 */
	intel_init_power_wells(dev);

	intel_prepare_ddi(dev);

	intel_init_clock_gating(dev);

	mutex_lock(&dev->struct_mutex);
	intel_enable_gt_powersave(dev);
	mutex_unlock(&dev->struct_mutex);
}

void intel_modeset_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i, j, ret;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.preferred_depth = 24;
	dev->mode_config.prefer_shadow = 1;

	dev->mode_config.funcs = &intel_mode_funcs;

	intel_init_quirks(dev);

	intel_init_pm(dev);

	if (INTEL_INFO(dev)->num_pipes == 0)
		return;

	intel_init_display(dev);

	if (IS_GEN2(dev)) {
		dev->mode_config.max_width = 2048;
		dev->mode_config.max_height = 2048;
	} else if (IS_GEN3(dev)) {
		dev->mode_config.max_width = 4096;
		dev->mode_config.max_height = 4096;
	} else {
		dev->mode_config.max_width = 8192;
		dev->mode_config.max_height = 8192;
	}
	dev->mode_config.fb_base = dev_priv->mm.gtt_base_addr;

	DRM_DEBUG_KMS("%d display pipe%s available.\n",
		      INTEL_INFO(dev)->num_pipes,
		      INTEL_INFO(dev)->num_pipes > 1 ? "s" : "");

	for (i = 0; i < INTEL_INFO(dev)->num_pipes; i++) {
		intel_crtc_init(dev, i);
		for (j = 0; j < dev_priv->num_plane; j++) {
			ret = intel_plane_init(dev, i, j);
			if (ret)
				DRM_DEBUG_KMS("pipe %d plane %d init failed: %d\n",
					      i, j, ret);
		}
	}

	intel_cpu_pll_init(dev);
	intel_pch_pll_init(dev);

	/* Just disable it once at startup */
	i915_disable_vga(dev);
	intel_setup_outputs(dev);
}

static void
intel_connector_break_all_links(struct intel_connector *connector)
{
	connector->base.dpms = DRM_MODE_DPMS_OFF;
	connector->base.encoder = NULL;
	connector->encoder->connectors_active = false;
	connector->encoder->base.crtc = NULL;
}

static void intel_enable_pipe_a(struct drm_device *dev)
{
	struct intel_connector *connector;
	struct drm_connector *crt = NULL;
	struct intel_load_detect_pipe load_detect_temp;

	/* We can't just switch on the pipe A, we need to set things up with a
	 * proper mode and output configuration. As a gross hack, enable pipe A
	 * by enabling the load detect pipe once. */
	list_for_each_entry(connector,
			    &dev->mode_config.connector_list,
			    base.head) {
		if (connector->encoder->type == INTEL_OUTPUT_ANALOG) {
			crt = &connector->base;
			break;
		}
	}

	if (!crt)
		return;

	if (intel_get_load_detect_pipe(crt, NULL, &load_detect_temp))
		intel_release_load_detect_pipe(crt, &load_detect_temp);


}

static bool
intel_check_plane_mapping(struct intel_crtc *crtc)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 reg, val;

	if (INTEL_INFO(dev)->num_pipes == 1)
		return true;

	reg = DSPCNTR(!crtc->plane);
	val = I915_READ(reg);

	if ((val & DISPLAY_PLANE_ENABLE) &&
	    (!!(val & DISPPLANE_SEL_PIPE_MASK) == crtc->pipe))
		return false;

	return true;
}

static void intel_sanitize_crtc(struct intel_crtc *crtc)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 reg;

	/* Clear any frame start delays used for debugging left by the BIOS */
	reg = PIPECONF(crtc->config.cpu_transcoder);
	I915_WRITE(reg, I915_READ(reg) & ~PIPECONF_FRAME_START_DELAY_MASK);

	/* We need to sanitize the plane -> pipe mapping first because this will
	 * disable the crtc (and hence change the state) if it is wrong. Note
	 * that gen4+ has a fixed plane -> pipe mapping.  */
	if (INTEL_INFO(dev)->gen < 4 && !intel_check_plane_mapping(crtc)) {
		struct intel_connector *connector;
		bool plane;

		DRM_DEBUG_KMS("[CRTC:%d] wrong plane connection detected!\n",
			      crtc->base.base.id);

		/* Pipe has the wrong plane attached and the plane is active.
		 * Temporarily change the plane mapping and disable everything
		 * ...  */
		plane = crtc->plane;
		crtc->plane = !plane;
		dev_priv->display.crtc_disable(&crtc->base);
		crtc->plane = plane;

		/* ... and break all links. */
		list_for_each_entry(connector, &dev->mode_config.connector_list,
				    base.head) {
			if (connector->encoder->base.crtc != &crtc->base)
				continue;

			intel_connector_break_all_links(connector);
		}

		WARN_ON(crtc->active);
		crtc->base.enabled = false;
	}

	if (dev_priv->quirks & QUIRK_PIPEA_FORCE &&
	    crtc->pipe == PIPE_A && !crtc->active) {
		/* BIOS forgot to enable pipe A, this mostly happens after
		 * resume. Force-enable the pipe to fix this, the update_dpms
		 * call below we restore the pipe to the right state, but leave
		 * the required bits on. */
		intel_enable_pipe_a(dev);
	}

	/* Adjust the state of the output pipe according to whether we
	 * have active connectors/encoders. */
	intel_crtc_update_dpms(&crtc->base);

	if (crtc->active != crtc->base.enabled) {
		struct intel_encoder *encoder;

		/* This can happen either due to bugs in the get_hw_state
		 * functions or because the pipe is force-enabled due to the
		 * pipe A quirk. */
		DRM_DEBUG_KMS("[CRTC:%d] hw state adjusted, was %s, now %s\n",
			      crtc->base.base.id,
			      crtc->base.enabled ? "enabled" : "disabled",
			      crtc->active ? "enabled" : "disabled");

		crtc->base.enabled = crtc->active;

		/* Because we only establish the connector -> encoder ->
		 * crtc links if something is active, this means the
		 * crtc is now deactivated. Break the links. connector
		 * -> encoder links are only establish when things are
		 *  actually up, hence no need to break them. */
		WARN_ON(crtc->active);

		for_each_encoder_on_crtc(dev, &crtc->base, encoder) {
			WARN_ON(encoder->connectors_active);
			encoder->base.crtc = NULL;
		}
	}
}

static void intel_sanitize_encoder(struct intel_encoder *encoder)
{
	struct intel_connector *connector;
	struct drm_device *dev = encoder->base.dev;

	/* We need to check both for a crtc link (meaning that the
	 * encoder is active and trying to read from a pipe) and the
	 * pipe itself being active. */
	bool has_active_crtc = encoder->base.crtc &&
		to_intel_crtc(encoder->base.crtc)->active;

	if (encoder->connectors_active && !has_active_crtc) {
		DRM_DEBUG_KMS("[ENCODER:%d:%s] has active connectors but no active pipe!\n",
			      encoder->base.base.id,
			      drm_get_encoder_name(&encoder->base));

		/* Connector is active, but has no active pipe. This is
		 * fallout from our resume register restoring. Disable
		 * the encoder manually again. */
		if (encoder->base.crtc) {
			DRM_DEBUG_KMS("[ENCODER:%d:%s] manually disabled\n",
				      encoder->base.base.id,
				      drm_get_encoder_name(&encoder->base));
			encoder->disable(encoder);
		}

		/* Inconsistent output/port/pipe state happens presumably due to
		 * a bug in one of the get_hw_state functions. Or someplace else
		 * in our code, like the register restore mess on resume. Clamp
		 * things to off as a safer default. */
		list_for_each_entry(connector,
				    &dev->mode_config.connector_list,
				    base.head) {
			if (connector->encoder != encoder)
				continue;

			intel_connector_break_all_links(connector);
		}
	}
	/* Enabled encoders without active connectors will be fixed in
	 * the crtc fixup. */
}

static void i915_redisable_vga(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 vga_reg;

	if (HAS_PCH_SPLIT(dev))
		vga_reg = CPU_VGACNTRL;
	else
		vga_reg = VGACNTRL;

	if (I915_READ(vga_reg) != VGA_DISP_DISABLE) {
		DRM_DEBUG_KMS("Something enabled VGA plane, disabling it\n");
		I915_WRITE(vga_reg, VGA_DISP_DISABLE);
		POSTING_READ(vga_reg);
	}
}

/* Scan out the current hw modeset state, sanitizes it and maps it into the drm
 * and i915 state tracking structures. */
void intel_modeset_setup_hw_state(struct drm_device *dev,
				  bool force_restore)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum pipe pipe;
	u32 tmp;
	struct drm_plane *plane;
	struct intel_crtc *crtc;
	struct intel_encoder *encoder;
	struct intel_connector *connector;

	if (IS_HASWELL(dev)) {
		tmp = I915_READ(TRANS_DDI_FUNC_CTL(TRANSCODER_EDP));

		if (tmp & TRANS_DDI_FUNC_ENABLE) {
			switch (tmp & TRANS_DDI_EDP_INPUT_MASK) {
			case TRANS_DDI_EDP_INPUT_A_ON:
			case TRANS_DDI_EDP_INPUT_A_ONOFF:
				pipe = PIPE_A;
				break;
			case TRANS_DDI_EDP_INPUT_B_ONOFF:
				pipe = PIPE_B;
				break;
			case TRANS_DDI_EDP_INPUT_C_ONOFF:
				pipe = PIPE_C;
				break;
			default:
				/* A bogus value has been programmed, disable
				 * the transcoder */
				WARN(1, "Bogus eDP source %08x\n", tmp);
				intel_ddi_disable_transcoder_func(dev_priv,
						TRANSCODER_EDP);
				goto setup_pipes;
			}

			crtc = to_intel_crtc(dev_priv->pipe_to_crtc_mapping[pipe]);
			crtc->config.cpu_transcoder = TRANSCODER_EDP;

			DRM_DEBUG_KMS("Pipe %c using transcoder EDP\n",
				      pipe_name(pipe));
		}
	}

setup_pipes:
	list_for_each_entry(crtc, &dev->mode_config.crtc_list,
			    base.head) {
		enum transcoder tmp = crtc->config.cpu_transcoder;
		memset(&crtc->config, 0, sizeof(crtc->config));
		crtc->config.cpu_transcoder = tmp;

		crtc->active = dev_priv->display.get_pipe_config(crtc,
								 &crtc->config);

		crtc->base.enabled = crtc->active;

		DRM_DEBUG_KMS("[CRTC:%d] hw state readout: %s\n",
			      crtc->base.base.id,
			      crtc->active ? "enabled" : "disabled");
	}

	if (IS_HASWELL(dev))
		intel_ddi_setup_hw_pll_state(dev);

	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {
		pipe = 0;

		if (encoder->get_hw_state(encoder, &pipe)) {
			encoder->base.crtc =
				dev_priv->pipe_to_crtc_mapping[pipe];
		} else {
			encoder->base.crtc = NULL;
		}

		encoder->connectors_active = false;
		DRM_DEBUG_KMS("[ENCODER:%d:%s] hw state readout: %s, pipe=%i\n",
			      encoder->base.base.id,
			      drm_get_encoder_name(&encoder->base),
			      encoder->base.crtc ? "enabled" : "disabled",
			      pipe);
	}

	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    base.head) {
		if (connector->get_hw_state(connector)) {
			connector->base.dpms = DRM_MODE_DPMS_ON;
			connector->encoder->connectors_active = true;
			connector->base.encoder = &connector->encoder->base;
		} else {
			connector->base.dpms = DRM_MODE_DPMS_OFF;
			connector->base.encoder = NULL;
		}
		DRM_DEBUG_KMS("[CONNECTOR:%d:%s] hw state readout: %s\n",
			      connector->base.base.id,
			      drm_get_connector_name(&connector->base),
			      connector->base.encoder ? "enabled" : "disabled");
	}

	/* HW state is read out, now we need to sanitize this mess. */
	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {
		intel_sanitize_encoder(encoder);
	}

	for_each_pipe(pipe) {
		crtc = to_intel_crtc(dev_priv->pipe_to_crtc_mapping[pipe]);
		intel_sanitize_crtc(crtc);
	}

	if (force_restore) {
		for_each_pipe(pipe) {
			crtc = to_intel_crtc(dev_priv->pipe_to_crtc_mapping[pipe]);
			intel_set_mode(&crtc->base, &crtc->base.mode,
				       crtc->base.x, crtc->base.y, crtc->base.fb);
		}

		i915_redisable_vga(dev);
	} else {
		intel_modeset_update_staged_output_state(dev);
	}

	intel_modeset_check_state(dev);

	drm_mode_config_reset(dev);
}

void intel_modeset_gem_init(struct drm_device *dev)
{
	intel_modeset_init_hw(dev);

	intel_setup_overlay(dev);

	intel_modeset_setup_hw_state(dev, false);
}

void intel_modeset_cleanup(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;
	struct intel_crtc *intel_crtc;

	drm_kms_helper_poll_fini(dev);
	mutex_lock(&dev->struct_mutex);

	intel_unregister_dsm_handler();


	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		/* Skip inactive CRTCs */
		if (!crtc->fb)
			continue;

		intel_crtc = to_intel_crtc(crtc);
		intel_increase_pllclock(crtc);
	}

	intel_disable_fbc(dev);

	intel_disable_gt_powersave(dev);

	ironlake_teardown_rc6(dev);

	if (IS_VALLEYVIEW(dev))
		vlv_init_dpio(dev);

	mutex_unlock(&dev->struct_mutex);

	/* Disable the irq before mode object teardown, for the irq might
	 * enqueue unpin/hotplug work. */
	drm_irq_uninstall(dev);
	cancel_work_sync(&dev_priv->hotplug_work);
	cancel_work_sync(&dev_priv->rps_work);

	/* flush any delayed tasks or pending work */
	flush_scheduled_work();

	/* Shut off idle work before the crtcs get freed. */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		intel_crtc = to_intel_crtc(crtc);
		del_timer_sync(&intel_crtc->idle_timer);
	}
	del_timer_sync(&dev_priv->idle_timer);
	cancel_work_sync(&dev_priv->idle_work);

	/* destroy backlight, if any, before the connectors */
	intel_panel_destroy_backlight(dev);

	/* destroy backlight, if any, before the connectors */
	intel_panel_destroy_backlight(dev);

	drm_mode_config_cleanup(dev);
}

/*
 * Return which encoder is currently attached for connector.
 */
struct drm_encoder *intel_best_encoder(struct drm_connector *connector)
{
	return &intel_attached_encoder(connector)->base;
}

void intel_connector_attach_encoder(struct intel_connector *connector,
				    struct intel_encoder *encoder)
{
	connector->encoder = encoder;
	drm_mode_connector_attach_encoder(&connector->base,
					  &encoder->base);
}

/*
 * set vga decode state - true == enable VGA decode
 */
int intel_modeset_vga_set_state(struct drm_device *dev, bool state)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u16 gmch_ctrl;

	pci_read_config_word(dev_priv->bridge_dev, INTEL_GMCH_CTRL, &gmch_ctrl);
	if (state)
		gmch_ctrl &= ~INTEL_GMCH_VGA_DISABLE;
	else
		gmch_ctrl |= INTEL_GMCH_VGA_DISABLE;
	pci_write_config_word(dev_priv->bridge_dev, INTEL_GMCH_CTRL, gmch_ctrl);
	return 0;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

struct intel_display_error_state {
	struct intel_cursor_error_state {
		u32 control;
		u32 position;
		u32 base;
		u32 size;
	} cursor[I915_MAX_PIPES];

	struct intel_pipe_error_state {
		u32 conf;
		u32 source;

		u32 htotal;
		u32 hblank;
		u32 hsync;
		u32 vtotal;
		u32 vblank;
		u32 vsync;
	} pipe[I915_MAX_PIPES];

	struct intel_plane_error_state {
		u32 control;
		u32 stride;
		u32 size;
		u32 pos;
		u32 addr;
		u32 surface;
		u32 tile_offset;
	} plane[I915_MAX_PIPES];
};

struct intel_display_error_state *
intel_display_capture_error_state(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_display_error_state *error;
	enum transcoder cpu_transcoder;
	int i;

	error = kmalloc(sizeof(*error), GFP_ATOMIC);
	if (error == NULL)
		return NULL;

	for_each_pipe(i) {
		cpu_transcoder = intel_pipe_to_cpu_transcoder(dev_priv, i);

		if (INTEL_INFO(dev)->gen <= 6 || IS_VALLEYVIEW(dev)) {
			error->cursor[i].control = I915_READ(CURCNTR(i));
			error->cursor[i].position = I915_READ(CURPOS(i));
			error->cursor[i].base = I915_READ(CURBASE(i));
		} else {
			error->cursor[i].control = I915_READ(CURCNTR_IVB(i));
			error->cursor[i].position = I915_READ(CURPOS_IVB(i));
			error->cursor[i].base = I915_READ(CURBASE_IVB(i));
		}

		error->plane[i].control = I915_READ(DSPCNTR(i));
		error->plane[i].stride = I915_READ(DSPSTRIDE(i));
		if (INTEL_INFO(dev)->gen <= 3) {
			error->plane[i].size = I915_READ(DSPSIZE(i));
			error->plane[i].pos = I915_READ(DSPPOS(i));
		}
		if (INTEL_INFO(dev)->gen <= 7 && !IS_HASWELL(dev))
			error->plane[i].addr = I915_READ(DSPADDR(i));
		if (INTEL_INFO(dev)->gen >= 4) {
			error->plane[i].surface = I915_READ(DSPSURF(i));
			error->plane[i].tile_offset = I915_READ(DSPTILEOFF(i));
		}

		error->pipe[i].conf = I915_READ(PIPECONF(cpu_transcoder));
		error->pipe[i].source = I915_READ(PIPESRC(i));
		error->pipe[i].htotal = I915_READ(HTOTAL(cpu_transcoder));
		error->pipe[i].hblank = I915_READ(HBLANK(cpu_transcoder));
		error->pipe[i].hsync = I915_READ(HSYNC(cpu_transcoder));
		error->pipe[i].vtotal = I915_READ(VTOTAL(cpu_transcoder));
		error->pipe[i].vblank = I915_READ(VBLANK(cpu_transcoder));
		error->pipe[i].vsync = I915_READ(VSYNC(cpu_transcoder));
	}

	return error;
}

void
intel_display_print_error_state(struct seq_file *m,
				struct drm_device *dev,
				struct intel_display_error_state *error)
{
	int i;

	seq_printf(m, "Num Pipes: %d\n", INTEL_INFO(dev)->num_pipes);
	for_each_pipe(i) {
		seq_printf(m, "Pipe [%d]:\n", i);
		seq_printf(m, "  CONF: %08x\n", error->pipe[i].conf);
		seq_printf(m, "  SRC: %08x\n", error->pipe[i].source);
		seq_printf(m, "  HTOTAL: %08x\n", error->pipe[i].htotal);
		seq_printf(m, "  HBLANK: %08x\n", error->pipe[i].hblank);
		seq_printf(m, "  HSYNC: %08x\n", error->pipe[i].hsync);
		seq_printf(m, "  VTOTAL: %08x\n", error->pipe[i].vtotal);
		seq_printf(m, "  VBLANK: %08x\n", error->pipe[i].vblank);
		seq_printf(m, "  VSYNC: %08x\n", error->pipe[i].vsync);

		seq_printf(m, "Plane [%d]:\n", i);
		seq_printf(m, "  CNTR: %08x\n", error->plane[i].control);
		seq_printf(m, "  STRIDE: %08x\n", error->plane[i].stride);
		if (INTEL_INFO(dev)->gen <= 3) {
			seq_printf(m, "  SIZE: %08x\n", error->plane[i].size);
			seq_printf(m, "  POS: %08x\n", error->plane[i].pos);
		}
		if (INTEL_INFO(dev)->gen <= 7 && !IS_HASWELL(dev))
			seq_printf(m, "  ADDR: %08x\n", error->plane[i].addr);
		if (INTEL_INFO(dev)->gen >= 4) {
			seq_printf(m, "  SURF: %08x\n", error->plane[i].surface);
			seq_printf(m, "  TILEOFF: %08x\n", error->plane[i].tile_offset);
		}

		seq_printf(m, "Cursor [%d]:\n", i);
		seq_printf(m, "  CNTR: %08x\n", error->cursor[i].control);
		seq_printf(m, "  POS: %08x\n", error->cursor[i].position);
		seq_printf(m, "  BASE: %08x\n", error->cursor[i].base);
	}
}
#endif
