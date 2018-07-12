/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define DEBUG

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

/* this file is modified from ov5640.c for EMT project */

#define DEBUG_EN

#ifdef DEBUG_EN
    #define dprintk_func()          printk("tw9900:%s\n", __FUNCTION__)
	#define dprintk_debug1(a)		printk(a);
	#define dprintk_debug2(a,b)		printk(a,b);
	#define dprintk_debug3(a,b,c)	printk(a,b,c);
#else
    #define dprintk_func()
	#define dprintk_debug1(a)		
	#define dprintk_debug2(a,b)		
	#define dprintk_debug3(a,b,c)
#endif 

#define TW9900_IDENT                  0x00	/* IDENT */
#define TW9900_STATUS_1               0x01
#define TW9900_BRIGHTNESS             0x10	/* Brightness */
#define TW9900_SD_SATURATION_CB       0x13	/* SD Saturation Cb */
#define TW9900_SD_SATURATION_CR       0x14	/* SD Saturation Cr */
#define TW9900_STD_SELECTION		  0x1C 	/* Standard Selection */

#define V4L2_IDENT_TW9900  9900
#define TW9900_MAX_MUX   4

//code copied from ov5640
#define OV5640_VOLTAGE_ANALOG               2800000
#define OV5640_VOLTAGE_DIGITAL_CORE         1500000
#define OV5640_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

//code copied from ov5640
#define OV5640_XCLK_MIN 6000000
#define OV5640_XCLK_MAX 24000000

#define GET_ID(val) ((val&0xF8)>>3)

struct tw9900_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct tw9900 {
	struct v4l2_subdev		subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	v4l2_std_id std_id;
	const struct tw9900_datafmt	*fmt;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	void (*io_init)(void);
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct tw9900 tw9900_data;
//static int pwn_gpio, rst_gpio;
//static int prev_sysclk;
static int night_mode;
//static int AE_Target = 52, night_mode;
//static int prev_HTS;
//static int AE_high, AE_low;

/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	tw9900_NTSC = 0,	/*!< Locked on (M) NTSC video signal. */
	tw9900_PAL,		/*!< (B, G, H, I, N)PAL video signal. */
	tw9900_NOT_LOCKED,	/*!< Not locked on a signal. */
} video_fmt_idx;

#define MAX_TW9900_FMT      (tw9900_NOT_LOCKED)
/*! Number of video standards supported (including 'not locked' signal). */
#define tw9900_STD_MAX		(tw9900_PAL + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{			/*! NTSC */
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,	/* SENS_FRM_WIDTH */
	 .raw_height = 525,	/* SENS_FRM_HEIGHT */
	 .active_width = 720,	/* ACT_FRM_WIDTH plus 1 */
	 .active_height = 480,	/* ACT_FRM_WIDTH plus 1 */
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 525,
	 .active_width = 720,
	 .active_height = 480,
	 },
};

static int tw9900_framerates[] = {
	[tw9900_NTSC] = 30,
	[tw9900_PAL] = 25,
};

static const struct tw9900_datafmt tw9900_colour_fmts[] = {
	{MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SMPTE170M},
};

static video_fmt_idx video_idx = tw9900_NTSC;

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;

static int tw9900_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int tw9900_remove(struct i2c_client *client);

static int tw9900_write_reg(u8 reg,u8 val);
static int tw9900_read(u8 reg);

#ifdef CONFIG_OF
static const struct of_device_id bmi160_of_match[] = {
	{ .compatible = "intersil,tw9900" },
	{ },
};
MODULE_DEVICE_TABLE(of, bmi160_of_match);
#endif

static const struct i2c_device_id tw9900_id[] = {
	{"tw9900", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tw9900_id);

static struct i2c_driver tw9900_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "tw9900",
		  },
	.probe  = tw9900_probe,
	.remove = tw9900_remove,
	.id_table = tw9900_id,
};

static struct tw9900 *to_tw9900(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct tw9900, subdev);
}

static const struct tw9900_datafmt
			*tw9900_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tw9900_colour_fmts); i++)
		if (tw9900_colour_fmts[i].code == code)
			return tw9900_colour_fmts + i;

	return NULL;
}

static inline void tw9900_power_down(int enable)
{
	//Todo: implement device power up/down control
    /*
	gpio_set_value_cansleep(pwn_gpio, enable);

	msleep(2);
    */
}

static void tw9900_hard_reset(void)
{
	/* EMT Init Value */

	//NTSC PRO
	tw9900_write_reg(0xFF,0x00);  //; Page 00
	tw9900_write_reg(0x00,0x00);
	tw9900_write_reg(0x01,0x68);  //; 00
	tw9900_write_reg(0x02,0x40);
	tw9900_write_reg(0x03,0xA2);
	tw9900_write_reg(0x04,0x00);
	tw9900_write_reg(0x05,0x01);
	tw9900_write_reg(0x06,0x80);
	tw9900_write_reg(0x07,0x02);
	tw9900_write_reg(0x08,0x13);
	tw9900_write_reg(0x09,0xF2);
	tw9900_write_reg(0x0A,0x28);
	tw9900_write_reg(0x0B,0xD0);
	tw9900_write_reg(0x0C,0xCC);
	tw9900_write_reg(0x0D,0x00);
	tw9900_write_reg(0x0E,0x11);
	tw9900_write_reg(0x0F,0x01);
	tw9900_write_reg(0x10,0x04);
	tw9900_write_reg(0x11,0x64);
	tw9900_write_reg(0x12,0x11);
	tw9900_write_reg(0x13,0xC0);
	tw9900_write_reg(0x14,0x80);
	tw9900_write_reg(0x15,0x10);
	tw9900_write_reg(0x16,0x00);
	tw9900_write_reg(0x17,0x30);
	tw9900_write_reg(0x18,0x44);
	tw9900_write_reg(0x19,0x57);
	tw9900_write_reg(0x1A,0x0F);
	tw9900_write_reg(0x1B,0x02);
	tw9900_write_reg(0x1C,0x0F);
	tw9900_write_reg(0x1D,0x7F);
	tw9900_write_reg(0x1E,0x08);
	tw9900_write_reg(0x1F,0x00);
	tw9900_write_reg(0x20,0x50);
	tw9900_write_reg(0x21,0xC6);
	tw9900_write_reg(0x22,0xF4);
	tw9900_write_reg(0x23,0xD8);
	tw9900_write_reg(0x24,0xBC);
	tw9900_write_reg(0x25,0xF8);
	tw9900_write_reg(0x26,0x44);
	tw9900_write_reg(0x27,0x2A);
	tw9900_write_reg(0x28,0x00);
	tw9900_write_reg(0x29,0x03);
	tw9900_write_reg(0x2A,0x78);
	tw9900_write_reg(0x2B,0x46);
	tw9900_write_reg(0x2C,0x30);
	tw9900_write_reg(0x2D,0x07);
	tw9900_write_reg(0x2E,0xA5);
	tw9900_write_reg(0x2F,0xE0);
	tw9900_write_reg(0x30,0x00);
	tw9900_write_reg(0x31,0x10);
	tw9900_write_reg(0x32,0xFF);
	tw9900_write_reg(0x33,0x05);
	tw9900_write_reg(0x34,0x1A);
	tw9900_write_reg(0x35,0x81);
	tw9900_write_reg(0x50,0xA0);
	tw9900_write_reg(0x51,0x22);
	tw9900_write_reg(0x52,0x31);
	tw9900_write_reg(0x55,0x00);
	tw9900_write_reg(0x6B,0x09);
	tw9900_write_reg(0x6C,0x19);
	tw9900_write_reg(0x6D,0x0A);
	tw9900_write_reg(0x6E,0x28);
	tw9900_write_reg(0x6F,0x93);

}

static inline void tw9900_reset(void)
{
	/* camera reset */
	//gpio_set_value_cansleep(rst_gpio, 1);

	/* camera power down */
    /*
	gpio_set_value_cansleep(pwn_gpio, 1);
	msleep(5);
	gpio_set_value_cansleep(pwn_gpio, 0);
	msleep(5);
	gpio_set_value_cansleep(rst_gpio, 0);
	msleep(1);
	gpio_set_value_cansleep(rst_gpio, 1);
	msleep(5);
	gpio_set_value_cansleep(pwn_gpio, 1);
    */
}

//code reuse from ov5640
static int ov5640_regulator_enable(struct device *dev)
{
	int ret = 0;

	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator,
				      OV5640_VOLTAGE_DIGITAL_IO,
				      OV5640_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(io_regulator);
		if (ret) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set io voltage ok\n");
		}
	} else {
		io_regulator = NULL;
		dev_warn(dev, "cannot get io voltage\n");
	}

	core_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(core_regulator)) {
		regulator_set_voltage(core_regulator,
				      OV5640_VOLTAGE_DIGITAL_CORE,
				      OV5640_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(core_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set core voltage ok\n");
		}
	} else {
		core_regulator = NULL;
		dev_warn(dev, "cannot get core voltage\n");
	}

	analog_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(analog_regulator)) {
		regulator_set_voltage(analog_regulator,
				      OV5640_VOLTAGE_ANALOG,
				      OV5640_VOLTAGE_ANALOG);
		ret = regulator_enable(analog_regulator);
		if (ret) {
			dev_err(dev, "set analog voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set analog voltage ok\n");
		}
	} else {
		analog_regulator = NULL;
		dev_warn(dev, "cannot get analog voltage\n");
	}

	return ret;
}

static int tw9900_write_reg(u8 reg,u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(tw9900_data.i2c_client, reg, val);
	if (ret < 0) {
		dev_dbg(&tw9900_data.i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}
	return 0;
}

static int tw9900_read(u8 reg)
{
	int val;
	val = i2c_smbus_read_byte_data(tw9900_data.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&tw9900_data.i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

static int tw9900_init_mode(void)
{
	//struct reg_value *pModeSetting = NULL;
	//int ArySize = 0
    int retval = 0;

    /*
	ov5640_soft_reset();

	pModeSetting = ov5640_global_init_setting;
	ArySize = ARRAY_SIZE(ov5640_global_init_setting);
	retval = ov5640_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	pModeSetting = ov5640_init_setting_30fps_VGA;
	ArySize = ARRAY_SIZE(ov5640_init_setting_30fps_VGA);
	retval = ov5640_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;
    */

	/* change driver capability to 2x according to validation board.
	 * if the image is not stable, please increase the driver strength.
	 */
    /*
	ov5640_driver_capability(2);
	ov5640_set_bandingfilter();
	ov5640_set_AE_target(AE_Target);
	ov5640_set_night_mode(night_mode);
    */

	/* skip 9 vysnc: start capture at 10th vsync */
	//msleep(300);

	/* turn off night mode */
	night_mode = 0;
	//tw9900_data.pix.width = 640;
	//tw9900_data.pix.height = 480;
//err:
	return retval;
}

static int tw9900_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tw9900 *sensor = to_tw9900(client);

    dprintk_func();
	if (on)
		clk_enable(tw9900_data.sensor_clk);
	else
		clk_disable(tw9900_data.sensor_clk);

	sensor->on = on;

	return 0;
}

static int tw9900_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tw9900 *sensor = to_tw9900(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

    dprintk_func();

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		//ret = -EINVAL;
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int tw9900_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	int ret = 0;

    dprintk_func();

    /* 
	switch (a->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}
    */

	return ret;
}

static int tw9900_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct tw9900_datafmt *fmt = tw9900_find_datafmt(mf->code);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tw9900 *sensor = to_tw9900(client);

    dprintk_func();

	if (format->pad)
		return -EINVAL;

	if (!fmt) {
		mf->code	= tw9900_colour_fmts[0].code;
		mf->colorspace	= tw9900_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_INTERLACED;
	//mf->field	= V4L2_FIELD_NONE;

	pr_debug("width:%d, height:%d\n", mf->width, mf->height);
	pr_debug("code:%d, ", mf->code);
	pr_debug("colorspace:%d\n", mf->colorspace);
	pr_debug("field:%d\n", mf->field);
	if (format->which == V4L2_SUBDEV_FORMAT_TRY){
		printk("V4L2_SUBDEV_FORMAT_TRY\n");
		return 0;
	}

	sensor->fmt = fmt;

	return 0;
}

static int tw9900_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tw9900 *sensor = to_tw9900(client);
	const struct tw9900_datafmt *fmt = sensor->fmt;

    dprintk_func();

	if (format->pad)
		return -EINVAL;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->field	= V4L2_FIELD_INTERLACED;
	//mf->field	= V4L2_FIELD_NONE;

	pr_debug("Code:%d, colorspace:%d, field:%d\n", mf->code, mf->colorspace, mf->field);
	return 0;
}

static int tw9900_enum_code(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_mbus_code_enum *code)
{
    dprintk_func();

	if (code->pad || code->index >= ARRAY_SIZE(tw9900_colour_fmts))
		return -EINVAL;

	code->code = tw9900_colour_fmts[code->index].code;

	pr_debug("index:%d, code:%d\n", code->index, code->code);

	return 0;
}

static int tw9900_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
    dprintk_func();

	if (fse->index >= MAX_TW9900_FMT)
		return -EINVAL;

    //use 'raw_width'
	//fse->max_width = video_fmts[video_idx].raw_width;
	//fse->max_height  = video_fmts[video_idx].raw_height;
    //use 'active_width'
	fse->max_width = video_fmts[video_idx].active_width;
	fse->max_height  = video_fmts[video_idx].active_height;

    //use index as given by user
	//fse->max_width = video_fmts[fse->index].active_width;
	//fse->max_height  = video_fmts[fse->index].active_height;
	fse->min_width = fse->max_width;
	fse->min_height = fse->max_height;

	dprintk_debug3("image:width[%d], height[%d]\n", fse->max_width, fse->max_height);
	return 0;
}

static int tw9900_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	//int i;
	int j, count;

    dprintk_func();

	if (fie->index < 0 || fie->index >= MAX_TW9900_FMT)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warning("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
		for (j = 0; j < (MAX_TW9900_FMT + 1); j++) {
			if (fie->width == video_fmts[j].active_width
			 && fie->height == video_fmts[j].active_height) {
			//if (fie->width == video_fmts[j].raw_width
			//&& fie->height == video_fmts[j].raw_height) {
                if (fie->index == j) {
                    fie->interval.denominator =
                            tw9900_framerates[j];
					pr_debug("frameinterval[%d]\n", fie->interval.denominator);
                    return 0;
                }

		        pr_debug("tw9900 index fail:index[%d], match[%d]\n", fie->index, j);
			}
		}

	return -EINVAL;
}

static int tw9900_set_clk_rate(void)
{
	u32 tgt_xclk;	/* target xclk */
	int ret;

	pr_debug("   Ori mclk to %d MHz\n", tw9900_data.mclk / 1000000);
	/* mclk */
	tgt_xclk = tw9900_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV5640_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV5640_XCLK_MIN);
	tw9900_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(tw9900_data.sensor_clk, tw9900_data.mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", tw9900_data.mclk);
	return ret;
}

/*!
 * dev_init - V4L2 sensor init
 * @s: pointer to standard V4L2 device structure
 *
 */
static int init_device(void)
{
//	u32 tgt_xclk;	/* target xclk */
	int ret;

	tw9900_data.on = true;

	/* mclk */
//	tgt_xclk = tw9900_data.mclk;

	ret = tw9900_init_mode();

	return ret;
}

static struct v4l2_subdev_video_ops tw9900_subdev_video_ops = {
	.g_parm = tw9900_g_parm,		//get streaming parameter
	.s_parm = tw9900_s_parm,		//set streaming parameter
};

static const struct v4l2_subdev_pad_ops tw9900_subdev_pad_ops = {
	.enum_frame_size       = tw9900_enum_framesizes,			//return supported framesize: width, height
	.enum_frame_interval   = tw9900_enum_frameintervals,		//return supported framerate: 50/60
	.enum_mbus_code        = tw9900_enum_code,					//return supported color format: code/colorspace/field
	.set_fmt               = tw9900_set_fmt,					//set color format: code/colorspace/field
	.get_fmt               = tw9900_get_fmt,					//get color format: code/colorspace/field
};

static struct v4l2_subdev_core_ops tw9900_subdev_core_ops = {
	.s_power	= tw9900_s_power,
};

static struct v4l2_subdev_ops tw9900_subdev_ops = {
	.core	= &tw9900_subdev_core_ops,
	.video	= &tw9900_subdev_video_ops,
	.pad	= &tw9900_subdev_pad_ops,
};

static void tw9900_get_std(v4l2_std_id *std)
{
	int status_1, standard, idx,std_select;
	bool locked;

	dev_dbg(&tw9900_data.i2c_client->dev, "In tw9900_get_std\n");
	status_1 = tw9900_read(TW9900_STATUS_1);

	std_select = tw9900_read(TW9900_STD_SELECTION);
	dprintk_debug2("read[0x1c],ret[%x]==================\n", std_select);
	locked = status_1 & 0x80?0:1;
	standard = (std_select & 0x70) >> 4;
	dprintk_debug3("standard:%d, locked:%d\n", standard, locked);

//	mutex_lock(&mutex);
	*std = V4L2_STD_ALL;
	idx = tw9900_NOT_LOCKED;
//	standard = 0;
//	locked = 1;
	if (locked) {
		if (standard == 0) {
			*std = V4L2_STD_NTSC;
			idx = tw9900_NTSC;
		} else if (standard == 1) {
			*std = V4L2_STD_PAL;
			idx = tw9900_PAL;
		} else if (standard == 2) {
//			*std = V4L2_STD_SECAM;
//			idx = tw9900_SECAM;
		} else if (standard == 3) {
			*std = V4L2_STD_NTSC_443;
			idx = tw9900_NTSC;
		}
	}
//	mutex_unlock(&mutex);

	/* This assumes autodetect which this device uses. */
	if (*std != tw9900_data.std_id) {
		video_idx = idx;
		tw9900_data.std_id = *std;
		//tw9900_data.pix.width = video_fmts[video_idx].raw_width;
		//tw9900_data.pix.width = video_fmts[video_idx].raw_width;
		tw9900_data.pix.height = video_fmts[video_idx].active_height;
		tw9900_data.pix.height = video_fmts[video_idx].active_height;
	}
}

/*!
 * tw9900 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int tw9900_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	//struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
    int rev_id;
	int retval;
	//v4l2_std_id std;
	//u8 chip_id_high, chip_id_low;

    dprintk_debug1("tw9900_probe\n");
	/* ov5640 pinctrl */
	/*
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}
	*/

	/* request power down pin */
    /*
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
					"ov5640_pwdn");
	if (retval < 0)
		return retval;
    */

	/* request reset pin */
    /*
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(rst_gpio)) {
		dev_err(dev, "no sensor reset pin available\n");
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
					"tw9900_reset");
	if (retval < 0)
		return retval;
    */

	/* Set initial values for the sensor struct. */
	memset(&tw9900_data, 0, sizeof(tw9900_data));
	tw9900_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(tw9900_data.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(tw9900_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&tw9900_data.mclk);
	if (retval) {
		dev_err(dev, "mclk frequency is invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(tw9900_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(tw9900_data.csi));
	if (retval) {
		dev_err(dev, "csi_id invalid\n");
		return retval;
	}

	/* Set mclk rate before clk on */
	tw9900_set_clk_rate();

	clk_prepare_enable(tw9900_data.sensor_clk);

	tw9900_data.io_init = tw9900_reset;
	tw9900_data.i2c_client = client;
	tw9900_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	video_idx = tw9900_NOT_LOCKED;
	tw9900_data.std_id = V4L2_STD_ALL;
	tw9900_data.pix.width = video_fmts[video_idx].active_width;
	tw9900_data.pix.height = video_fmts[video_idx].active_height;
//	tw9900_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |			//???
//					   V4L2_CAP_TIMEPERFRAME;
	tw9900_data.streamcap.capturemode = 0;
	tw9900_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	tw9900_data.streamcap.timeperframe.numerator = 1;

	ov5640_regulator_enable(&client->dev);

	tw9900_reset();

	tw9900_power_down(0);

	/*! Read the revision ID of the tvin chip */
	rev_id = tw9900_read(TW9900_IDENT);
      //  dev_dbg(dev,
       //"%s:Analog Device TW99%d detected!\n", __func__,
	//	GET_ID(rev_id));
	dprintk_debug3("%s:Analog Device TW99%d detected!\n", __func__,
		GET_ID(rev_id));

    tw9900_hard_reset();

    /*
	msleep(100);

	retval = tw9900_read(0x03);
	dpintk_debug2("read[0x03], ret[%x]===========================", retval);
	msleep(5);
	retval = tw9900_read(0x1c);
	if(retval & 0x80)
	{
		msleep(200);
		retval = tw9900_read(0x1c);
	}
	dpintk_debug2("read[0x1c],ret[%x]===========================",retval); 
	msleep(5);
	retval = tw9900_read(0x01);
	dpintk_debug2("read[0x01],ret[%x]===========================",retval);

	msleep(200);
	tw9900_get_std(&std);
	dpintk_debug2("tw9900:colorsystem:%d\n", video_idx);
	*/

	init_device();

	clk_disable(tw9900_data.sensor_clk);

	v4l2_i2c_subdev_init(&tw9900_data.subdev, client, &tw9900_subdev_ops);

	retval = v4l2_async_register_subdev(&tw9900_data.subdev);
	if (retval < 0)
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);

	pr_info("******00007 : camera tw9900, is found\n");
	return retval;
}

/*!
 * tw9900 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int tw9900_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

    dprintk_debug1("tw9900_remove\n");
	v4l2_async_unregister_subdev(sd);

	clk_unprepare(tw9900_data.sensor_clk);

	tw9900_power_down(1);

	if (analog_regulator)
		regulator_disable(analog_regulator);

	if (core_regulator)
		regulator_disable(core_regulator);

	if (io_regulator)
		regulator_disable(io_regulator);

	return 0;
}

module_i2c_driver(tw9900_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("TW9900 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
