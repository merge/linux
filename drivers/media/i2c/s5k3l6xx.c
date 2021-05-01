// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Samsung S5K3L6XX 1/3" 13M CMOS Image Sensor.
 *
 * Copyright (C) 2020-2021 Purism SPC
 *
 * Based on S5K5BAF driver
 * Copyright (C) 2013, Samsung Electronics Co., Ltd.
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * Based on S5K6AA driver authored by Sylwester Nawrocki
 * Copyright (C) 2013, Samsung Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/slab.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-fwnode.h>

static int debug;
module_param(debug, int, 0644);

#define S5K3L6XX_DRIVER_NAME		"s5k3l6xx"
#define S5K3L6XX_DEFAULT_MCLK_FREQ	24000000U
#define S5K3L6XX_CLK_NAME		"mclk"

#define S5K3L6XX_REG_MODEL_ID_L		0x0000
#define S5K3L6XX_REG_MODEL_ID_H		0x0001
#define S5K3L6XX_MODEL_ID_L		0x30
#define S5K3L6XX_MODEL_ID_H		0xc6

#define S5K3L6XX_REG_REVISION_NUMBER	0x0002
#define S5K3L6XX_REVISION_NUMBER	0xb0

#define S5K3L6XX_REG_FRAME_COUNT	0x0005
#define S5K3L6XX_REG_LANE_MODE		0x0114
#define S5K3L6XX_REG_FINE_INTEGRATION_TIME		0x0200 // 2 bytes
#define S5K3L6XX_REG_COARSE_INTEGRATION_TIME		0x0202 // 2 bytes
#define S5K3L6XX_REG_ANALOG_GAIN		0x0204 // 2 bytes
#define S5K3L6XX_REG_DIGITAL_GAIN		0x020e // 2 bytes

#define S5K3L6XX_REG_TEST_PATTERN_MODE	0x0601
#define S5K3L6XX_TEST_PATTERN_SOLID_COLOR	0x01
#define S5K3L6XX_TEST_PATTERN_COLOR_BAR	0x02

#define S5K3L6XX_REG_TEST_DATA_RED	0x0602
#define S5K3L6XX_REG_TEST_DATA_GREENR	0x0604
#define S5K3L6XX_REG_TEST_DATA_BLUE	0x0606
#define S5K3L6XX_REG_TEST_DATA_GREENB	0x0608

#define S5K3L6XX_REG_AF			0x3403
#define S5K3L6XX_REG_AF_BIT_FILTER	0b100

#define S5K3L6XX_REG_MODE_SELECT	0x100
#define S5K3L6XX_MODE_STREAMING		0x1
#define S5K3L6XX_MODE_STANDBY		0x0

#define S5K3L6XX_REG_DATA_FORMAT	0x0112
#define S5K3L6XX_DATA_FORMAT_RAW8	0x0808

#define S5K3L6XX_CIS_WIDTH		4208
#define S5K3L6XX_CIS_HEIGHT		3120

struct s5k3l6xx_reg {
	u16 address;
	u16 val;
	// Size of a single write.
	u8 size;
};

// Downscaled 1:4 in both directions.
// Spans the entire sensor. Fps unknown.
// Relies on defaults to be set correctly.
static const struct s5k3l6xx_reg frame_1052x780px_8bit_xfps_2lane[] = {
	// extclk freq 25MHz (doesn't seem to matter)
	{ 0x0136, 0x1900,       2 },

	// x_output_size
	{ 0x034c, 0x041c,       2 },
	// line length in pixel clocks. x_output_size * 1.16
	// if using binning multiply x_output_size by the binning factor first
	{ 0x0342, 0x1320,       2 },
	// y_output_size
	{ 0x034e, 0x030c,       2 },

	// op_pll_multiplier, default 0064
	{ 0x030e, 0x0036,       2 },

	// y_addr_start
	{ 0x0346, 0x0000,       2 },
	// end = y_output_size * binning_factor + y_addr_start
	{ 0x034a, 0x0c30,       2 },
	// x_addr_start
	{ 0x0344, 0x0008,       2 },
	// end = x_output_size * binning_factor + x_addr_start - 1
	{ 0x0348, 0x1077,       2 },

	// binning enable
	{ 0x0900, 0x01, 1 },
	// type: 1/?x, 1/?y, full binning when matching skips
	{ 0x0901, 0x44, 1 },
	// y_odd_inc
	{ 0x0387, 0x07, 1 },

	// Noise reduction
	// The last 3 bits (0x0007) control some global brightness/noise pattern.
	// They work slightly differently depending on the value of 307b:80
	// It's not strictly necessary here,
	// as the sensor seems to do the same correction without asking at 1:4 binning,
	// but added to formalize the default value.
	{ 0x3074, 0x0974, 2},
};

// Downscaled 1:2 in both directions.
// Spans the entire sensor. Fps unknown.
// Relies on defaults to be set correctly.
static const struct s5k3l6xx_reg frame_2104x1560px_8bit_xfps_2lane[] = {
	// extclk freq 25MHz (doesn't seem to matter)
	{ 0x0136, 0x1900,       2 },

	// x_output_size
	{ 0x034c, 0x0838,       2 },
	// y_output_size
	{ 0x034e, 0x0618,       2 },
	// op_pll_multiplier, default 0064
	// 0036 is good for 175MHz on mipi side, although it makes the source clock 216MHz (double-check)
	// 0042 ok for 200MHz
	// 0052 ok for 250MHz
	{ 0x030e, 0x0053,       2 },

	// y_addr_start
	{ 0x0346, 0x0000,       2 },
	// end
	{ 0x034a, 0x0c30,       2 },
	// x_addr_start
	{ 0x0344, 0x0000,       2 },
	// end to match sensor
	{ 0x0348, 0x1068,       2 },

	// binning in 1:2 mode seems to average out focus pixels.
	// binning enable
	{ 0x0900, 0x01, 1 },
	// type: 1/?x, 1/?y, full binning when matching skips
	{ 0x0901, 0x22, 1 },
	// x binning skips 8-pixel bocks, making it useless
	// y_odd_inc
	{ 0x0387, 0x03, 1 },

	// Noise reduction
	// The last 3 bits (0x0007) control some global brightness/noise pattern.
	// They work slightly differently depending on the value of 307b:80
	// 0x0972 makes focus pixels appear.
	{ 0x3074, 0x0974, 2}, // 74, 75, 76, 77 all good for binning 1:2.

	// filter out autofocus pixels
	// FIXME: this should be behind a custom control instead
	{ 0x3403, 0x42 | S5K3L6XX_REG_AF_BIT_FILTER, 1 },
};

// Not scaled.
// Spans the entire sensor. Fps unknown.
// Relies on defaults to be set correctly.
static const struct s5k3l6xx_reg frame_4208x3120px_8bit_xfps_2lane[] = {
	// extclk freq (doesn't actually matter)
	{ 0x0136, 0x1900,       2 },

	// x_output_size
	{ 0x034c, 0x1070,       2 },
	// y_output_size
	{ 0x034e, 0x0c30,       2 },
	// op_pll_multiplier, default 0064
	// 0036 is good (max) for 175MHz on mipi side, although it makes the source clock 216MHz (double-check)
	// 0042 ok for 200MHz
	// 0052 ok for 250MHz
	// 006c for 333Mhz
	{ 0x030e, 0x0033,       2 },

	// y_addr_start
	{ 0x0346, 0x0000,       2 },
	// end
	{ 0x034a, 0x0c30,       2 },
	// x_addr_start
	{ 0x0344, 0x0000,       2 },
	// end to match sensor
	{ 0x0348, 0x1068,       2 },
	// line length in pixel clocks. This is a slow mode.
	{ 0x0342, 0x3600,       2 },

	// Noise reduction
	// The last 3 bits (0x0007) control some global brightness/noise pattern.
	// They work slightly differently depending on the value of 307b:80
	{ 0x3074, 0x0977, 2}, // 74, 75, 76, 77 all good for binning 1:!, might introduce banding.

	// filter out autofocus pixels
	// FIXME: this should be behind a custom control instead
	{ 0x3403, 0x42 | S5K3L6XX_REG_AF_BIT_FILTER, 1 },
};

struct s5k3l6xx_gpio {
	int gpio;
	int level;
};

enum s5k3l6xx_gpio_id {
	RST,
	NUM_GPIOS,
};

#define PAD_CIS 0
#define PAD_OUT 1
#define NUM_CIS_PADS 1
#define NUM_ISP_PADS 2


struct s5k3l6xx_frame {
	char *name;
	u32 width;
	u32 height;
	u32 code;
	const struct s5k3l6xx_reg  *streamregs;
	u16 streamregcount;
};

struct s5k3l6xx_ctrls {
	struct v4l2_ctrl_handler handler;
	struct { /* Auto / manual white balance cluster */
		struct v4l2_ctrl *awb;
		struct v4l2_ctrl *gain_red;
		struct v4l2_ctrl *gain_blue;
	};
	struct { /* Mirror cluster */
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};
	struct { /* Auto exposure / manual exposure and gain cluster */
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *analog_gain;
		struct v4l2_ctrl *digital_gain;
	};
};

struct regstable_entry {
	u16 address;
	u8 value;
};

#define REGSTABLE_SIZE 4096

struct regstable {
	unsigned entry_count;
	struct regstable_entry entries[REGSTABLE_SIZE];
};

struct s5k3l6xx {
	struct s5k3l6xx_gpio gpios[NUM_GPIOS];
	enum v4l2_mbus_type bus_type;
	u8 nlanes;
	struct regulator *supply;

	struct clk *clock;
	u32 mclk_frequency;

	struct v4l2_subdev cis_sd;
	struct media_pad cis_pad;

	struct v4l2_subdev sd;
	struct media_pad pads[NUM_ISP_PADS];

	/* protects the struct members below */
	struct mutex lock;

	int error;

	/* Currently selected frame format */
	const struct s5k3l6xx_frame *frame_fmt;

	struct s5k3l6xx_ctrls ctrls;

	/* Solid color test pattern is in effect,
	 * write needs to happen after color choice writes.
	 * It doesn't seem that controls guarantee any order of application. */
	unsigned int apply_test_solid:1;

	unsigned int streaming:1;
	unsigned int apply_cfg:1;
	unsigned int apply_crop:1;
	unsigned int valid_auto_alg:1;
	unsigned int power;

	u8 debug_frame; // Enables any size, sets empty debug frame.
	/* For debug address temporary value */
	u16 debug_address;
	struct regstable debug_regs;
};

static const struct s5k3l6xx_reg no_regs[0] = {};

static const struct s5k3l6xx_frame s5k3l6xx_frame_debug = {
	.name = "debug_empty",
	.width = 640, .height = 480,
	.streamregs = no_regs,
	.streamregcount = 0,
	.code = MEDIA_BUS_FMT_SGRBG8_1X8,
};

// Frame sizes are only available in RAW, so this effectively replaces pixfmt.
// Supported frame configurations.
static const struct s5k3l6xx_frame s5k3l6xx_frames[] = {
	{
		.name = "1:4 8bpp ?fps",
		.width = 1052, .height = 780,
		.streamregs = frame_1052x780px_8bit_xfps_2lane,
		.streamregcount = ARRAY_SIZE(frame_1052x780px_8bit_xfps_2lane),
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
	},
	{
		.name = "1:2 8bpp +fps",
		.width = 2104, .height = 1560,
		.streamregs = frame_2104x1560px_8bit_xfps_2lane,
		.streamregcount = ARRAY_SIZE(frame_2104x1560px_8bit_xfps_2lane),
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
	},
	{
		.name = "1:1 8bpp ?fps",
		.width = 4208, .height = 3120,
		.streamregs = frame_4208x3120px_8bit_xfps_2lane,
		.streamregcount = ARRAY_SIZE(frame_4208x3120px_8bit_xfps_2lane),
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
	},
};

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct s5k3l6xx, ctrls.handler)->sd;
}

static inline bool s5k5baf_is_cis_subdev(struct v4l2_subdev *sd)
{
	return sd->entity.function == MEDIA_ENT_F_CAM_SENSOR;
}

static inline struct s5k3l6xx *to_s5k3l6xx(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k3l6xx, sd);
}

static u8 __s5k3l6xx_i2c_read(struct s5k3l6xx *state, u16 addr)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	__be16 w;
	u8 res;
	struct i2c_msg msg[] = {
		{ .addr = c->addr, .flags = 0,
		  .len = 2, .buf = (u8 *)&w },
		{ .addr = c->addr, .flags = I2C_M_RD,
		  .len = 1, .buf = (u8 *)&res },
	};
	int ret;

	if (state->error)
		return 0;

	w = cpu_to_be16(addr);
	ret = i2c_transfer(c->adapter, msg, 2);

	if (ret != 2) {
		v4l2_err(c, "i2c_read: error during transfer (%d)\n", ret);
		state->error = ret;
	}
	return res;
}

static u8 s5k3l6xx_i2c_read(struct s5k3l6xx *state, u16 addr)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	u8 res = __s5k3l6xx_i2c_read(state, addr);
	v4l2_dbg(3, debug, c, "i2c_read: 0x%04x : 0x%02x\n", addr, res);
	return res;
}

static void s5k3l6xx_i2c_write(struct s5k3l6xx *state, u16 addr, u8 val)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	u8 buf[3] = { addr >> 8, addr & 0xFF, val };

	struct i2c_msg msg[] = {
		{ .addr = c->addr, .flags = 0,
		  .len = 3, .buf = buf },
	};
	int ret;
	int actual;

	if (state->error)
		return;

	ret = i2c_transfer(c->adapter, msg, 1);

	v4l2_dbg(3, debug, c, "i2c_write to 0x%04x : 0x%02x\n", addr, val);

	if (ret != 1) {
		v4l2_err(c, "i2c_write: error during transfer (%d)\n", ret);
		state->error = ret;
	}
	// Not sure if actually needed. So really debugging code at the moment.
	actual = s5k3l6xx_i2c_read(state, addr);
	if (actual != val) {
		v4l2_err(c, "i2c_write: value didn't stick. 0x%04x = 0x%02x != 0x%02x", addr, actual, val);
	}
}

static void s5k3l6xx_i2c_write2(struct s5k3l6xx *state, u16 addr, u16 val)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	u8 buf[4] = { addr >> 8, addr & 0xFF, (val >> 8) & 0xff, val & 0xff };

	struct i2c_msg msg[] = {
		{ .addr = c->addr, .flags = 0,
		  .len = 4, .buf = buf },
	};
	int ret;

	if (state->error)
		return;

	ret = i2c_transfer(c->adapter, msg, 1);

	v4l2_dbg(3, debug, c, "i2c_write to 0x%04x : 0x%04x\n", addr, val);

	if (ret != 1) {
		v4l2_err(c, "i2c_write: error during transfer (%d)\n", ret);
		state->error = ret;
	}
}

static void s5k3l6xx_submit_regs(struct s5k3l6xx *state, const struct s5k3l6xx_reg *regs, u16 regcount) {
       unsigned i;
       for (i = 0; i < regcount; i++) {
	       if (regs[i].size == 2)
		       s5k3l6xx_i2c_write2(state, regs[i].address, regs[i].val);
	       else
		       s5k3l6xx_i2c_write(state, regs[i].address, (u8)regs[i].val);
       }
}

static u8 s5k3l6xx_read(struct s5k3l6xx *state, u16 addr)
{
	return s5k3l6xx_i2c_read(state, addr);
}

static void s5k3l6xx_write(struct s5k3l6xx *state, u16 addr, u8 val)
{
	s5k3l6xx_i2c_write(state, addr, val);
}

static void s5k3l6xx_submit_regstable(struct s5k3l6xx *state, const struct regstable *regs)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	unsigned i;
	for (i = 0; i < regs->entry_count; i++) {
		u16 addr = regs->entries[i].address;
		u8 val = regs->entries[i].value;
		if (debug >= 5) {
			u8 res = __s5k3l6xx_i2c_read(state, addr);
			if (res != val) {
				v4l2_dbg(5, debug, c, "overwriting: 0x%04x : 0x%02x\n", addr, res);
			}
		}
		s5k3l6xx_i2c_write(state, addr, val);
	}
}

static int s5k3l6xx_find_pixfmt(const struct v4l2_mbus_framefmt *mf)
{
	int i, c = -1;

	for (i = 0; i < ARRAY_SIZE(s5k3l6xx_frames); i++) {
		if ((mf->colorspace != V4L2_COLORSPACE_DEFAULT)
				&& (mf->colorspace != V4L2_COLORSPACE_RAW))
			continue;
		if ((mf->width != s5k3l6xx_frames[i].width) || (mf->height != s5k3l6xx_frames[i].height))
			continue;
		if (mf->code == s5k3l6xx_frames[i].code)
			return i;
	}
	return c;
}

static int s5k3l6xx_clear_error(struct s5k3l6xx *state)
{
	int ret = state->error;

	state->error = 0;
	return ret;
}

static const struct s5k3l6xx_reg setstream[] = {
	{ S5K3L6XX_REG_DATA_FORMAT, S5K3L6XX_DATA_FORMAT_RAW8, 2 },
	// Noise reduction
	// Bit 0x0080 will create noise when off (by default)
	// Raises data pedestal to 15-16.
	{ 0x307a, 0x0d00, 2 },
};

static void s5k3l6xx_hw_set_config(struct s5k3l6xx *state) {
	const struct s5k3l6xx_frame *frame_fmt = state->frame_fmt;
	v4l2_dbg(3, debug, &state->sd, "Setting frame format %s", frame_fmt->name);
	s5k3l6xx_submit_regs(state, frame_fmt->streamregs, frame_fmt->streamregcount);

	// This may mess up PLL settings...
	// If the above already enabled streaming (setfile A), we're also in trouble.
	s5k3l6xx_submit_regs(state, setstream, ARRAY_SIZE(setstream));
	s5k3l6xx_write(state, S5K3L6XX_REG_LANE_MODE, state->nlanes - 1);

	s5k3l6xx_submit_regstable(state, &state->debug_regs);
}

static void s5k3l6xx_hw_set_test_pattern(struct s5k3l6xx *state, int id)
{
	s5k3l6xx_write(state, S5K3L6XX_REG_TEST_PATTERN_MODE, (u8)id);
}


static void s5k3l6xx_gpio_assert(struct s5k3l6xx *state, int id)
{
	struct s5k3l6xx_gpio *gpio = &state->gpios[id];

	if (gpio == 0)
		return;

	gpio_set_value(gpio->gpio, gpio->level);
}

static void s5k3l6xx_gpio_deassert(struct s5k3l6xx *state, int id)
{
	struct s5k3l6xx_gpio *gpio = &state->gpios[id];

	if (gpio == 0)
		return;

	gpio_set_value(gpio->gpio, !gpio->level);
}

static int s5k3l6xx_power_on(struct s5k3l6xx *state)
{
	int ret;

	ret = regulator_enable(state->supply);
	if (ret < 0)
		goto err;

	ret = clk_set_rate(state->clock, state->mclk_frequency);
	if (ret < 0)
		goto err_reg_dis;

	ret = clk_prepare_enable(state->clock);
	if (ret < 0)
		goto err_reg_dis;

	v4l2_dbg(1, debug, &state->sd, "ON. clock frequency: %ld\n",
		 clk_get_rate(state->clock));

	usleep_range(50, 100);
	s5k3l6xx_gpio_deassert(state, RST);
	return 0;

err_reg_dis:
	if (regulator_is_enabled(state->supply))
		regulator_disable(state->supply);
err:
	v4l2_err(&state->sd, "%s() failed (%d)\n", __func__, ret);
	return ret;
}

static int s5k3l6xx_power_off(struct s5k3l6xx *state)
{
	int ret;

	state->streaming = 0;
	state->apply_cfg = 0;
	state->apply_crop = 0;

	s5k3l6xx_gpio_assert(state, RST);

	if (!IS_ERR(state->clock))
		clk_disable_unprepare(state->clock);

	if (!regulator_is_enabled(state->supply))
		return 0;

	ret = regulator_disable(state->supply);
	if (ret < 0)
		v4l2_err(&state->sd, "failed to disable regulators\n");
	else
		v4l2_dbg(1, debug, &state->sd, "OFF\n");
	return 0;
}

/*
 * V4L2 subdev core and video operations
 */

static int s5k3l6xx_set_power(struct v4l2_subdev *sd, int on)
{
	struct s5k3l6xx *state = to_s5k3l6xx(sd);
	int ret = 0;

	mutex_lock(&state->lock);

	if (state->power != !on)
		goto out;

	if (on) {
		ret = s5k3l6xx_power_on(state); // TODO: test this
		if (ret < 0)
			goto out;

		ret = s5k3l6xx_clear_error(state);
		if (!ret)
			state->power++;
	} else {
		s5k3l6xx_power_off(state);
		state->power--;
	}

out:
	mutex_unlock(&state->lock);
	return ret;
}

static void s5k3l6xx_hw_set_stream(struct s5k3l6xx *state, int enable)
{
	v4l2_dbg(3, debug, &state->sd, "set_stream %d", enable);
	s5k3l6xx_i2c_write(state, S5K3L6XX_REG_MODE_SELECT, enable ? S5K3L6XX_MODE_STREAMING : S5K3L6XX_MODE_STANDBY);
}

static int s5k3l6xx_s_stream(struct v4l2_subdev *sd, int on)
{
	struct s5k3l6xx *state = to_s5k3l6xx(sd);
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	int ret = 0;

	if (state->streaming == !!on) {
		return 0;
	}

	if (on) {
		ret = pm_runtime_get_sync(&c->dev);
		if (ret < 0) {
			dev_err(&c->dev, "%s: pm_runtime_get failed: %d\n",
				__func__, ret);
			pm_runtime_put_noidle(&c->dev);
			return ret;
		}

		ret = v4l2_ctrl_handler_setup(&state->ctrls.handler);
		if (ret < 0)
			goto out;

		mutex_lock(&state->lock);
		s5k3l6xx_hw_set_config(state);
		s5k3l6xx_hw_set_stream(state, 1);
	} else {
		mutex_lock(&state->lock);
		s5k3l6xx_hw_set_stream(state, 0);
		pm_runtime_put(&c->dev);
	}
	ret = s5k3l6xx_clear_error(state);
	if (!ret)
		state->streaming = !state->streaming;

out:
	mutex_unlock(&state->lock);

	return ret;
}

/*
 * V4L2 subdev pad level and video operations
 */
static int s5k3l6xx_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(s5k3l6xx_frames))
		return -EINVAL;
	code->code = s5k3l6xx_frames[code->index].code;
	return 0;
}

static int s5k3l6xx_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	int i;

	if (fse->index > 0)
		return -EINVAL;

	i = ARRAY_SIZE(s5k3l6xx_frames);
	while (--i)
		if (fse->code == s5k3l6xx_frames[i].code)
			break;
	fse->code = s5k3l6xx_frames[i].code;
	fse->min_width = s5k3l6xx_frames[i].width;
	fse->max_width = s5k3l6xx_frames[i].width;
	fse->max_height = s5k3l6xx_frames[i].height;
	fse->min_height = s5k3l6xx_frames[i].height;

	return 0;
}

static int s5k3l6xx_try_cis_format(struct v4l2_mbus_framefmt *mf)
{
	int pixfmt;
	const struct s5k3l6xx_frame *mode = v4l2_find_nearest_size(s5k3l6xx_frames,
				      ARRAY_SIZE(s5k3l6xx_frames),
				      width, height,
				      mf->width, mf->height);
	struct v4l2_mbus_framefmt candidate = *mf;
	candidate.width = mode->width;
	candidate.height = mode->height;

	pixfmt = s5k3l6xx_find_pixfmt(&candidate);
	if (pixfmt < 0)
		return pixfmt;

	mf->colorspace = V4L2_COLORSPACE_RAW;
	mf->code = s5k3l6xx_frames[pixfmt].code;
	mf->field = V4L2_FIELD_NONE;

	return pixfmt;
}

static int s5k3l6xx_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	mf = &fmt->format;
	if (fmt->pad == PAD_CIS) {
		s5k3l6xx_try_cis_format(mf);
		return 0;
	}
	return 0;
}

static int s5k3l6xx_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct s5k3l6xx *state = to_s5k3l6xx(sd);
	int pixfmt_idx = 0;
	mf->field = V4L2_FIELD_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = *mf;
		return 0;
	}

	mutex_lock(&state->lock);

	if (state->streaming) {
		mutex_unlock(&state->lock);
		return -EBUSY;
	}

	if (state->debug_frame) {
		state->frame_fmt = &s5k3l6xx_frame_debug;
		// Keep frame width/height as requested.
	} else {
		pixfmt_idx = s5k3l6xx_try_cis_format(mf);
		if (pixfmt_idx == -1) {
			v4l2_err(sd, "set_fmt choice unsupported");
			mutex_unlock(&state->lock);
			return -EINVAL; // could not find the format. Unsupported
		}
		state->frame_fmt = &s5k3l6xx_frames[pixfmt_idx];
		mf->width = state->frame_fmt->width;
		mf->height = state->frame_fmt->height;
	}

	mf->code = state->frame_fmt->code;
	mf->colorspace = V4L2_COLORSPACE_RAW;

	mutex_unlock(&state->lock);
	return 0;
}

enum selection_rect { R_CIS, R_CROP_SINK, R_COMPOSE, R_CROP_SOURCE, R_INVALID };

static const struct v4l2_subdev_pad_ops s5k3l6xx_cis_pad_ops = {
	.enum_mbus_code		= s5k3l6xx_enum_mbus_code,
	.enum_frame_size	= s5k3l6xx_enum_frame_size,
	.get_fmt		= s5k3l6xx_get_fmt,
	.set_fmt		= s5k3l6xx_set_fmt,
};

static const struct v4l2_subdev_pad_ops s5k3l6xx_pad_ops = {
	.enum_mbus_code		= s5k3l6xx_enum_mbus_code,
	.enum_frame_size	= s5k3l6xx_enum_frame_size,
//	.enum_frame_interval	= s5k5baf_enum_frame_interval,
	// doesn't seem to be used... ioctl(3, VIDIOC_S_FMT, ...)
	// instead seems to call enum_fmt, which does enum_mbus_code here.
	.get_fmt		= s5k3l6xx_get_fmt,
	.set_fmt		= s5k3l6xx_set_fmt,
};

static const struct v4l2_subdev_video_ops s5k3l6xx_video_ops = {
	//.g_frame_interval	= s5k5baf_g_frame_interval,
	//.s_frame_interval	= s5k5baf_s_frame_interval,
	.s_stream		= s5k3l6xx_s_stream,
};

/*
 * V4L2 subdev controls
 */

static int s5k3l6xx_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct s5k3l6xx *state = to_s5k3l6xx(sd);
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	int in_use;
	int ret = 0;

	v4l2_dbg(1, debug, sd, "ctrl: %s, value: %d\n", ctrl->name, ctrl->val);

	mutex_lock(&state->lock);

	// Don't do anything when powered off.
	// It will get called again when powering up.
	if (state->power == 0)
		goto unlock;
	/* v4l2_ctrl_handler_setup() function may not be used in the device’s runtime PM
	 * runtime_resume callback, as it has no way to figure out the power state of the device.
	 * https://www.kernel.org/doc/html/latest/driver-api/media/camera-sensor.html#control-framework
	 * Okay, so what's the right way to do it? So far relying on state->power.
	 */

	in_use = pm_runtime_get_if_in_use(&c->dev);

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		// Analog gain supported up to 0x200 (16). Gain = register / 32, so 0x20 gives gain 1.
		s5k3l6xx_i2c_write2(state, S5K3L6XX_REG_ANALOG_GAIN, (u16)ctrl->val & 0x3ff);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		s5k3l6xx_i2c_write2(state, S5K3L6XX_REG_DIGITAL_GAIN, (u16)ctrl->val & 0xfff);
		break;
	case V4L2_CID_EXPOSURE:
		s5k3l6xx_i2c_write2(state, S5K3L6XX_REG_COARSE_INTEGRATION_TIME, (u16)ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		state->apply_test_solid = (ctrl->val == S5K3L6XX_TEST_PATTERN_SOLID_COLOR);
		v4l2_dbg(3, debug, sd, "Setting pattern %d", ctrl->val);
		s5k3l6xx_hw_set_test_pattern(state, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		s5k3l6xx_i2c_write2(state, S5K3L6XX_REG_TEST_DATA_RED, (u16)ctrl->val & 0x3ff);
		if (state->apply_test_solid)
			s5k3l6xx_hw_set_test_pattern(state, S5K3L6XX_TEST_PATTERN_SOLID_COLOR);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		s5k3l6xx_i2c_write2(state, S5K3L6XX_REG_TEST_DATA_GREENR, (u16)ctrl->val & 0x3ff);
		if (state->apply_test_solid)
			s5k3l6xx_hw_set_test_pattern(state, S5K3L6XX_TEST_PATTERN_SOLID_COLOR);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		s5k3l6xx_i2c_write2(state, S5K3L6XX_REG_TEST_DATA_BLUE, (u16)ctrl->val & 0x3ff);
		if (state->apply_test_solid)
			s5k3l6xx_hw_set_test_pattern(state, S5K3L6XX_TEST_PATTERN_SOLID_COLOR);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		s5k3l6xx_i2c_write2(state, S5K3L6XX_REG_TEST_DATA_GREENB, (u16)ctrl->val & 0x3ff);
		if (state->apply_test_solid)
			s5k3l6xx_hw_set_test_pattern(state, S5K3L6XX_TEST_PATTERN_SOLID_COLOR);
		break;
	}
	ret = s5k3l6xx_clear_error(state);

	if (in_use) { // came from other context than resume, need to manage PM
		pm_runtime_put(&c->dev);
	}
unlock:
	mutex_unlock(&state->lock);
	return ret;
}

static const struct v4l2_ctrl_ops s5k3l6xx_ctrl_ops = {
	.s_ctrl	= s5k3l6xx_s_ctrl,
};

static const char * const s5k3l6_test_pattern_menu[] = {
	"Disabled",
	"Solid", // Color selectable
	"Bars", // 8 bars 100% saturation: black, blue, red, magents, green, cyan, yellow, white
	"Fade", // Bars fading towards 50% at the bottom. 512px high. Subdivided into left smooth and right quantized halves.
	"White",
	"PN9", // pseudo-random noise
	"LFSR32",
	"Address",
};

static int s5k3l6xx_initialize_ctrls(struct s5k3l6xx *state)
{
	const struct v4l2_ctrl_ops *ops = &s5k3l6xx_ctrl_ops;
	struct s5k3l6xx_ctrls *ctrls = &state->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 16);
	if (ret < 0) {
		v4l2_err(&state->sd, "cannot init ctrl handler (%d)\n", ret);
		return ret;
	}

	// Exposure time (min: 2; max: frame length lines - 2; default: reset value)
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    2, 3118, 1, 0x03de);

	// Total gain: 32 <=> 1x
	ctrls->analog_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					0x20, 0x200, 1, 0x20);

	// Digital gain range: 1.0x - 3.0x
	ctrls->digital_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN,
					0x100, 0x300, 1, 0x100);

	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k3l6_test_pattern_menu) - 1,
				     0, 0, s5k3l6_test_pattern_menu);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_RED, 0, 1023, 1, 512);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_GREENR, 0, 1023, 1, 512);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_BLUE, 0, 1023, 1, 512);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_GREENB, 0, 1023, 1, 512);

	if (hdl->error) {
		v4l2_err(&state->sd, "error creating controls (%d)\n",
			 hdl->error);
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	state->sd.ctrl_handler = hdl;
	return 0;
}

/*
 * V4L2 subdev internal operations
 */
static int s5k3l6xx_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *mf;

	mf = v4l2_subdev_get_try_format(sd, fh->pad, PAD_CIS);
	s5k3l6xx_try_cis_format(mf);
	return 0;
}

static const struct v4l2_subdev_ops s5k5baf_cis_subdev_ops = {
	.pad	= &s5k3l6xx_cis_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k5baf_cis_subdev_internal_ops = {
	.open = s5k3l6xx_open,
};

static const struct v4l2_subdev_internal_ops s5k3l6xx_subdev_internal_ops = {
	.open = s5k3l6xx_open,
};

static const struct v4l2_subdev_core_ops s5k3l6xx_core_ops = {
	.s_power = s5k3l6xx_set_power,
	.log_status = v4l2_ctrl_subdev_log_status,
};

static const struct v4l2_subdev_ops s5k3l6xx_subdev_ops = {
	.core = &s5k3l6xx_core_ops,
	.pad = &s5k3l6xx_pad_ops,
	.video = &s5k3l6xx_video_ops,
};

static int __maybe_unused s5k3l6xx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k3l6xx *state = to_s5k3l6xx(sd);

	dev_dbg(dev, "%s\n", __func__);

	if (state->streaming)
		s5k3l6xx_hw_set_stream(state, 0);

	return s5k3l6xx_set_power(sd, FALSE);
}

static int __maybe_unused s5k3l6xx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k3l6xx *state = to_s5k3l6xx(sd);
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	ret = s5k3l6xx_set_power(sd, TRUE);
	msleep(500);

	if (ret == 0 && state->streaming)
		s5k3l6xx_hw_set_stream(state, 1);

	return ret;
}

// FIXME: are we even using this?
static int s5k3l6xx_configure_gpios(struct s5k3l6xx *state)
{
	static const char * const name[] = { "S5K5BAF_RST" };
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	struct s5k3l6xx_gpio *g = state->gpios;
	int ret, i;

	for (i = 0; i < NUM_GPIOS; ++i) {
		int flags = GPIOF_DIR_OUT;
		if (g[i].level)
			flags |= GPIOF_INIT_HIGH;
		ret = devm_gpio_request_one(&c->dev, g[i].gpio, flags, name[i]);
		if (ret < 0) {
			v4l2_err(c, "failed to request gpio %s\n", name[i]);
			return ret;
		}
	}
	return 0;
}

static int s5k3l6xx_parse_gpios(struct s5k3l6xx_gpio *gpios, struct device *dev)
{
	static const char * const names[] = {
		"rstn-gpios",
	};
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;
	int ret, i;

	for (i = 0; i < NUM_GPIOS; ++i) {
		ret = of_get_named_gpio_flags(node, names[i], 0, &flags);
		if (ret < 0) {
			dev_err(dev, "no %s GPIO pin provided\n", names[i]);
			return ret;
		}
		gpios[i].gpio = ret;
		gpios[i].level = !(flags & OF_GPIO_ACTIVE_LOW);
	}

	return 0;
}

static int s5k3l6xx_parse_device_node(struct s5k3l6xx *state, struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *node_ep;
	struct v4l2_fwnode_endpoint ep = { .bus_type = 0 };
	int ret;

	if (!node) {
		dev_err(dev, "no device-tree node provided\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "clock-frequency",
				   &state->mclk_frequency);
	if (ret < 0) {
		state->mclk_frequency = S5K3L6XX_DEFAULT_MCLK_FREQ;
		dev_warn(dev, "using default %u Hz clock frequency\n",
			 state->mclk_frequency);
	}

	ret = s5k3l6xx_parse_gpios(state->gpios, dev);
	if (ret < 0) {
		dev_err(dev, "parse gpios failed\n");
		return ret;
	}

	node_ep = of_graph_get_next_endpoint(node, NULL);
	if (!node_ep) {
		dev_err(dev, "no endpoint defined at node %pOF\n", node);
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node_ep), &ep);
	of_node_put(node_ep);
	if (ret) {
		dev_err(dev, "fwnode endpoint parse failed\n");
		return ret;
	}

	state->bus_type = ep.bus_type;

	switch (state->bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		state->nlanes = (unsigned char)ep.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		dev_err(dev, "unsupported bus %d in endpoint defined at node %pOF\n",
			state->bus_type, (void*)node);
		return -EINVAL;
	}

	return 0;
}

static int s5k3l6xx_configure_subdevs(struct s5k3l6xx *state,
				     struct i2c_client *c)
{
	struct v4l2_subdev *sd;
	int ret;

	/*sd = &state->cis_sd;
	v4l2_subdev_init(sd, &s5k5baf_cis_subdev_ops);
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, state);
	snprintf(sd->name, sizeof(sd->name), "S5K5BAF-CIS %d-%04x",
		 i2c_adapter_id(c->adapter), c->addr);

	sd->internal_ops = &s5k5baf_cis_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	state->cis_pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, NUM_CIS_PADS, &state->cis_pad);
	if (ret < 0)
		goto err;*/
	v4l2_info(&state->sd, "probe sd %px", (void*)&state->sd);
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, c, &s5k3l6xx_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "S5K3L6-CIS %d-%04x",
		 i2c_adapter_id(c->adapter), c->addr);
	v4l2_info(sd, "probe i2c %px", (void*)c);

	sd->internal_ops = &s5k3l6xx_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	state->cis_pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, NUM_CIS_PADS, &state->cis_pad);

	if (!ret)
		return 0;

	//media_entity_cleanup(&state->cis_sd.entity);
//err:
	dev_err(&c->dev, "cannot init media entity %s\n", sd->name);
	return ret;
}

static int s5k3l6xx_configure_regulators(struct s5k3l6xx *state)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);

	state->supply = devm_regulator_get(&c->dev, "vddio");
	if (IS_ERR(state->supply))
		v4l2_err(c, "failed to get regulators\n");

	return 0;
}

static int debug_add(void *data, u64 value)
{
	struct s5k3l6xx *state = data;
	struct regstable_entry entry = {
		.address = state->debug_address,
		.value = (u8)value,
	};
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	v4l2_dbg(1, debug, c, "debug add override 0x%04x 0x%02x\n", entry.address, entry.value);
	/* Not sure which error flag to set here.
	 * EOF is not available. E2BIG seems to be used too. */
	if (state->debug_regs.entry_count >= REGSTABLE_SIZE)
		return -EFBIG;
	if (value != entry.value)
		return -EINVAL;
	state->debug_regs.entries[state->debug_regs.entry_count] = entry;
	state->debug_regs.entry_count++;
	return 0;
}

static int debug_clear(void *data, u64 value)
{
	struct s5k3l6xx *state = data;
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	(void)value;
	v4l2_dbg(1, debug, c, "debug clear\n");
	state->debug_regs.entry_count = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_add_ops, NULL, debug_add, "%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_clear_ops, NULL, debug_clear, "%llu\n");

static int s5k3l6xx_probe(struct i2c_client *c)
{
	struct s5k3l6xx *state;
	int ret;
	u8 test;
	struct dentry *d;

	state = devm_kzalloc(&c->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	mutex_init(&state->lock);

	ret = s5k3l6xx_parse_device_node(state, &c->dev);
	if (ret < 0) {
		pr_err("s5k3l6xx_parse_device_node: failed");
		return ret;
	}

	ret = s5k3l6xx_configure_subdevs(state, c);
	if (ret < 0) {
		pr_err("s5k3l6xx_configure_subdevs: failed");
		return ret;
	}

	ret = s5k3l6xx_configure_gpios(state);
	if (ret < 0) {
		pr_err("s5k3l6xx_configure_gpios: failed");
		goto err_me;
	}

	ret = s5k3l6xx_configure_regulators(state);
	if (ret < 0) {
		pr_err("s5k3l6xx_configure_regulators: failed");
		goto err_me;
	}

	state->clock = devm_clk_get(state->sd.dev, S5K3L6XX_CLK_NAME);
	if (IS_ERR(state->clock)) {
		pr_err("get clk failed: failed");
		ret = -EPROBE_DEFER;
		goto err_me;
	}

	ret = s5k3l6xx_power_on(state);
	if (ret < 0) {
		pr_err("s5k3l6xx_power_on: failed");
		ret = -EPROBE_DEFER;
		goto err_me;
	}
	state->power = 1;

	test = s5k3l6xx_read(state, S5K3L6XX_REG_MODEL_ID_L);
	if (test != S5K3L6XX_MODEL_ID_L) {
		dev_err(&c->dev, "model mismatch: 0x%X != 0x30\n", test);
	} else {
		dev_info(&c->dev, "model low: 0x%X\n", test);
	}

	test = s5k3l6xx_read(state, S5K3L6XX_REG_MODEL_ID_H);
	if (test != S5K3L6XX_MODEL_ID_H) {
		dev_err(&c->dev, "model mismatch: 0x%X != 0xC6\n", test);
	} else {
		dev_info(&c->dev, "model high: 0x%X\n", test);
	}

	test = s5k3l6xx_read(state, S5K3L6XX_REG_REVISION_NUMBER);
	if (test != S5K3L6XX_REVISION_NUMBER) {
		dev_err(&c->dev, "revision mismatch: 0x%X != 0xB0\n", test);
	} else {
		dev_info(&c->dev, "revision number: 0x%X\n", test);
	}

	ret = s5k3l6xx_initialize_ctrls(state);
	if (ret < 0)
		goto err_me;

	ret = v4l2_async_register_subdev(&state->sd);
	if (ret < 0)
		goto err_ctrl;

	pm_runtime_set_active(&c->dev);
	pm_runtime_enable(&c->dev);

	// Default frame.
	state->frame_fmt = &s5k3l6xx_frames[0];

	d = debugfs_create_dir("s5k3l6", NULL);
	// When set to 1, then any frame size is accepted in frame set.
	// In addition, no sensor registers will be set, except stream on and bits per pixel.
	state->debug_frame = 0;
	debugfs_create_u8("debug_frame", S_IRUSR | S_IWUSR, d, &state->debug_frame);

	/* Can't be bothered to expose the entire register set in one file, so here it is.
	 * 1. Write u16 as hex to `address`.
	 * 2. Write u8 as hex to `add_value` and the *address = value will be saved.
	 * 3. Repeat if needed.
	 * 4. Reset the device (a suspend cycle will do)
	 * 5. Take pictures.
	 * 6. Write `1` to `clear` to erase all the added values.
	 */
	debugfs_create_x16("address", S_IRUSR | S_IWUSR, d, &state->debug_address);
	debugfs_create_file("add_value", S_IWUSR, d, (void*)state, &debug_add_ops);
	debugfs_create_file("clear", S_IWUSR, d, (void*)state, &debug_clear_ops);

	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(state->sd.ctrl_handler);
err_me:
	media_entity_cleanup(&state->sd.entity);
	media_entity_cleanup(&state->cis_sd.entity);
	return ret;
}

static int s5k3l6xx_remove(struct i2c_client *c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(c);
	struct s5k3l6xx *state = to_s5k3l6xx(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_entity_cleanup(&sd->entity);

	sd = &state->cis_sd;
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	pm_runtime_disable(&c->dev);
	pm_runtime_set_suspended(&c->dev);
	pm_runtime_put_noidle(&c->dev);

	return 0;
}

UNIVERSAL_DEV_PM_OPS(s5k3l6xx_pm_ops, s5k3l6xx_suspend, s5k3l6xx_resume, NULL);

static const struct i2c_device_id s5k3l6xx_id[] = {
	{ S5K3L6XX_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, s5k3l6xx_id);

static const struct of_device_id s5k3l6xx_of_match[] = {
	{ .compatible = "samsung,s5k3l6xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k3l6xx_of_match);

static struct i2c_driver s5k3l6xx_i2c_driver = {
	.driver = {
		.pm = &s5k3l6xx_pm_ops,
		.of_match_table = s5k3l6xx_of_match,
		.name = S5K3L6XX_DRIVER_NAME
	},
	.probe_new	= s5k3l6xx_probe,
	.remove		= s5k3l6xx_remove,
	.id_table	= s5k3l6xx_id,
};

module_i2c_driver(s5k3l6xx_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K3L6XX 13M camera driver");
MODULE_AUTHOR("Martin Kepplinger <martin.kepplinger@puri.sm>");
MODULE_AUTHOR("Dorota Czaplejewicz <dorota.czaplejewicz@puri.sm>");
MODULE_LICENSE("GPL v2");
