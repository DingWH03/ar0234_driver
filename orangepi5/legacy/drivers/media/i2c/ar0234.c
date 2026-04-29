// SPDX-License-Identifier: GPL-2.0
/*
 * ar0234 driver
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x00)

struct cam_regulator {
	char name[32];
	int val;
};

/* Module parameters */
static int trigger_mode;
module_param(trigger_mode, int, 0644);
MODULE_PARM_DESC(trigger_mode,
		 "Set trigger mode: 0=off, 1=external-trigger, 2=sync-sink");

/* Sensor frequencies */
#define AR0234_XVCLK_FREQ			24000000
#define AR0234_FREQ_PIXCLK_45MHZ		45000000
#define AR0234_FREQ_PIXCLK_90MHZ		90000000
#define AR0234_FREQ_LINK_8BIT			360000000
#define AR0234_FREQ_LINK_10BIT			450000000

#define AR0234_FRAME_LENGTH_LINES_120FPS  0x04C0
#define AR0234_FRAME_LENGTH_LINES_60FPS   0x0980  /* 0x04C0 * 2 */
#define AR0234_FRAME_LENGTH_LINES_30FPS   0x1300  /* 0x04C0 * 4 */

/* Chip ID */
#define AR0234_CHIP_ID				0x0A56
#define AR0234_REG_CHIP_ID			0x3000

/* Registers */
#define AR0234_REG_MODE_SELECT			0x301C
#define AR0234_REG_RESET				0x301A
#define AR0234_RESET_DEFAULT			0x2058
#define AR0234_RESET_STREAM			BIT(2)
#define AR0234_RESET_GPI_EN			BIT(8)
#define AR0234_RESET_FORCED_PLL_ON		BIT(11)

#define AR0234_REG_EXPOSURE_COARSE		0x3012
#define AR0234_REG_ANALOG_GAIN		0x3060
#define AR0234_REG_DIGITAL_GAIN		0x305E
#define AR0234_REG_VTS				0x300A
#define AR0234_REG_HTS				0x300C

#define AR0234_REG_IMAGE_ORIENTATION	0x301D
#define AR0234_REG_TEST_PATTERN_MODE	0x3070
#define AR0234_REG_TEST_DATA_RED		0x3072
#define AR0234_REG_TEST_DATA_GREENR		0x3074
#define AR0234_REG_TEST_DATA_BLUE		0x3076
#define AR0234_REG_TEST_DATA_GREENB		0x3078

#define AR0234_REG_SERIAL_FORMAT		0x31AE
#define AR0234_REG_GRR_CONTROL1		0x30CE
#define AR0234_GRR_SLAVE_SH_SYNC		BIT(8)

#define AR0234_REG_MFR_30BA			0x30BA
#define AR0234_MFR_30BA_GAIN_BITS(_val)	(0x7620 | (_val))
#define AR0234_MFR_30BA_DEFAULT		AR0234_MFR_30BA_GAIN_BITS(2)

#define AR0234_RESET_MASK_BAD        BIT(9)
#define AR0234_RESET_STREAM_ON       (AR0234_RESET_DEFAULT | AR0234_RESET_STREAM)
#define AR0234_RESET_STREAM_OFF      (AR0234_RESET_DEFAULT & ~AR0234_RESET_STREAM)

/* ROI registers */
#define AR0234_REG_ROI_X_START_OFFSET	0x3140
#define AR0234_REG_ROI_Y_START_OFFSET	0x3142
#define AR0234_REG_ROI_X_SIZE	0x3144
#define AR0234_REG_ROI_Y_SIZE	0x3146

#define AR0234_REG_LED_FLASH_CONTROL	0x3270
#define AR0234_FLASH_ENABLE			BIT(8)

/* Frame timing */
#define AR0234_FLL_OVERHEAD			5
#define AR0234_FLL_MAX      0xFFFF
#define AR0234_VBLANK_MIN   16
#define AR0234_FRAME_LENGTH_LINES_DEF  0x04C0
#define AR0234_LINE_LENGTH_PCK_DEF		612
#define AR0234_LINE_LENGTH_PIXEL_DEF	(AR0234_LINE_LENGTH_PCK_DEF * 4)

#define AR0234_ARRAY_WIDTH			1920
#define AR0234_ARRAY_HEIGHT			1200

/* Exposure */
#define AR0234_EXPOSURE_MIN			2
#define AR0234_EXPOSURE_STEP			1

/* Analog gain */
#define AR0234_ANA_GAIN_MIN			0x0D
#define AR0234_ANA_GAIN_MAX			0x40
#define AR0234_ANA_GAIN_STEP			1
#define AR0234_ANA_GAIN_DEFAULT		0x0E

/* Digital gain */
#define AR0234_DGTL_GAIN_MIN			0x0080
#define AR0234_DGTL_GAIN_MAX			0x07FF
#define AR0234_DGTL_GAIN_DEFAULT		0x0080
#define AR0234_DGTL_GAIN_STEP			1

/* Test pattern colors */
#define AR0234_TEST_PATTERN_COLOR_MIN	0
#define AR0234_TEST_PATTERN_COLOR_MAX	0x03FF
#define AR0234_TEST_PATTERN_COLOR_STEP	1

/* Trigger modes */
#define AR0234_TRIGGER_MODE_OFF		0
#define AR0234_TRIGGER_MODE_SLAVE_SYNC	2

/* Pixel array */
#define AR0234_NATIVE_WIDTH			1940U
#define AR0234_NATIVE_HEIGHT			1220U
#define AR0234_PIXEL_ARRAY_LEFT		10U
#define AR0234_PIXEL_ARRAY_TOP		10U
#define AR0234_PIXEL_ARRAY_WIDTH		1920U
#define AR0234_PIXEL_ARRAY_HEIGHT		1200U

/* Register access */
#define REG_NULL					0xFFFF
#define REG_DELAY					0xFFFE

#define AR0234_REG_VALUE_08BIT		1
#define AR0234_REG_VALUE_16BIT		2
#define AR0234_REG_VALUE_24BIT		3

#define AR0234_NAME					"ar0234"


static const struct cam_regulator ar0234_regulator[] = {
	{"camera_pwdn_gpio", 1800000}, /* 电压可忽略，实际由 GPIO 提供 */
};

struct regval {
	u16 addr;
	u16 val;
};

struct ar0234_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct ar0234 {
	struct i2c_client	*client;
	
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ar0234_regulator)];

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct clk			*xvclk;
	struct v4l2_rect	 crop_rect;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*vflip;
	struct v4l2_ctrl	*hflip;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ar0234_mode *cur_mode;
	u32 		cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;

	/* Hardware configuration */
	int trigger_mode;
	bool flash_enable;
	s8 flash_delay;
	bool monochrome;
	u32 num_data_lanes;
	struct v4l2_ctrl *pixel_rate;
	u16 mfr_30ba; /* cached value */
};

static const s64 link_freq_menu_items[] = {
    AR0234_FREQ_LINK_10BIT, 
};

#define to_ar0234(sd) container_of(sd, struct ar0234, subdev)

/* ------------------------------------------------------------------
 * Register sequences
 * ------------------------------------------------------------------ */

/* Common initialization (same for all modes) */
static const struct regval ar0234_common_regs[] = {
	{0x306E, 0x9010}, {0x3082, 0x0003},
	{0x31D0, 0x0000}, {0x3088, 0x8050}, {0x3086, 0x9237},
	{0x3096, 0x0280}, {0x31E0, 0x0003},
	{0x3F4C, 0x121F}, {0x3F4E, 0x121F}, {0x3F50, 0x0B81},
	{0x3ED2, 0xFA96}, {0x3180, 0x824F},
	{0x3ECC, 0x0C42}, {0x3ECC, 0x0C42},
	{0x30F0, 0x2283}, {0x3102, 0x5000},
	{0x30B4, 0x0011}, {0x3064, 0x1982},
	{REG_NULL, 0x0000},
};

/*
 * PLL config for 24MHz extclk, 450MHz link, 10-bit, 4-lane
 * (corresponding to pixel clock ~90MHz)
 */
static const struct regval ar0234_pll_24_450_10bit_regs[] = {
	{0x302A, 0x0005}, {0x302C, 0x0001}, {0x302E, 0x0008},
	{0x3030, 0x0096}, {0x3036, 0x000A}, {0x3038, 0x0001},
	{0x31B0, 0x0082}, {0x31B2, 0x005C},
	{0x31B4, 0x4248}, {0x31B6, 0x4258},
	{0x31B8, 0x904B}, {0x31BA, 0x030B},
	{0x31BC, 0x0D89}, {0x3354, 0x002B},
	{0x31AC, 0x0A0A}, /* 10-bit in/out */
	{REG_NULL, 0x0000},
};

/* Recommended manufacturer settings for 90 MHz pixel clock (4-lane) */
static const struct regval ar0234_pixclk_90mhz_regs[] = {
	/* No special settings; just ensure MFR_30BA default is set */
	{0x30BA, AR0234_MFR_30BA_DEFAULT},
	{REG_NULL, 0x0000},
};

/* Mode-specific sequences (crop/window) */
static const struct regval ar0234_1920x1200_regs[] = {
	{0x3002, 0x0008}, {0x3004, 0x0008},
	{0x3006, 0x04B7}, {0x3008, 0x0787},
	{0x30A2, 0x0001}, {0x30A6, 0x0001},
	{REG_NULL, 0x0000},
};

static const struct regval ar0234_1920x1080_regs[] = {
	{0x3002, 0x0044}, {0x3004, 0x0008},
	{0x3006, 0x047B}, {0x3008, 0x0787},
	{0x30A2, 0x0001}, {0x30A6, 0x0001},
	{REG_NULL, 0x0000},
};

static const struct regval ar0234_1280x720_regs[] = {
	{0x3002, 0x00F8}, {0x3004, 0x0148},
	{0x3006, 0x03C7}, {0x3008, 0x0647},
	{0x30A2, 0x0001}, {0x30A6, 0x0001},
	{REG_NULL, 0x0000},
};

static const struct regval ar0234_960x600_regs[] = {
	{0x3002, 0x0008}, {0x3004, 0x0008},
	{0x3006, 0x04B7}, {0x3008, 0x0787},
	{0x30A2, 0x0003}, {0x30A6, 0x0003},
	{0x3040, 0x3000}, /* read mode: binning */
	{REG_NULL, 0x0000},
};

/* Aggregate mode table */
static const struct ar0234_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1200,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1200000, /* 120fps */
		},
		.hts_def = AR0234_LINE_LENGTH_PIXEL_DEF,
		.vts_def = AR0234_FRAME_LENGTH_LINES_120FPS,
		.exp_def = 0x0010,
		.reg_list = ar0234_1920x1200_regs,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1200000, /* 120fps */
		},
		.hts_def = AR0234_LINE_LENGTH_PIXEL_DEF,
		.vts_def = AR0234_FRAME_LENGTH_LINES_120FPS,
		.exp_def = 0x0010,
		.reg_list = ar0234_1920x1080_regs,
	},
	{
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000, /* 30fps */
		},
		.hts_def = AR0234_LINE_LENGTH_PIXEL_DEF,
		.vts_def = AR0234_FRAME_LENGTH_LINES_30FPS,
		.exp_def = 0x0010,
		.reg_list = ar0234_1280x720_regs,
	},
	{
		.width = 960,
		.height = 600,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000, /* 60fps */
		},
		.hts_def = AR0234_LINE_LENGTH_PIXEL_DEF,
		.vts_def = AR0234_FRAME_LENGTH_LINES_60FPS,
		.exp_def = 0x0010,
		.reg_list = ar0234_960x600_regs,
	},
};

static const char * const ar0234_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"Vertical Color Bars",
	"Fade to Grey Color Bars",
	"Walking 1s",
};

static const u32 ar0234_test_pattern_val[] = {
	0,	/* Disabled: Generates output data from pixel array  */
	1,	/* Solid Color */
	2,	/* Vertical Color Bars */
	3,	/* Fade to Grey */
	256,	/* Walking 1s */
};

/* ------------------------------------------------------------------
 * I2C read/write utilities (ported from ar0230)
 * ------------------------------------------------------------------ */
static int ar0234_write_reg(struct i2c_client *client, u16 reg,
			    int len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;
	usleep_range(10, 20);
	return 0;
}

static int ar0234_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val, regs[i].val * 2);
		else
			ret = ar0234_write_reg(client, regs[i].addr,
					       AR0234_REG_VALUE_16BIT,
					       regs[i].val);
	}
	return ret;
}

static int ar0234_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);
	return 0;
}

/* ------------------------------------------------------------------
 * V4L2 subdev operations
 * ------------------------------------------------------------------ */
static int ar0234_get_reso_dist(const struct ar0234_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ar0234_mode *
ar0234_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ar0234_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
	return &supported_modes[cur_best_fit];
}

static int ar0234_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	const struct ar0234_mode *mode;
	s64 h_blank, vblank_def, pixel_rate;
	u32 fps;
	u32 code;
	int ret = 0;

	mutex_lock(&ar0234->mutex);

	mode = ar0234_find_best_fit(fmt);
	code = ar0234->monochrome ? MEDIA_BUS_FMT_Y10_1X10 :
				    MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.code = code;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		return -ENOTTY;
#endif
		goto unlock;
	} 
	ar0234->cur_mode = mode;
	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(ar0234->hblank, h_blank,
					h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(ar0234->vblank, vblank_def,
					AR0234_FLL_MAX - mode->height,
					1, vblank_def);

	if (ar0234->pixel_rate) {
		fps = mode->max_fps.numerator ?
			  DIV_ROUND_CLOSEST(mode->max_fps.denominator,
					    mode->max_fps.numerator) : 30;
		pixel_rate = mode->vts_def * mode->hts_def * fps;
		__v4l2_ctrl_modify_range(ar0234->pixel_rate, pixel_rate,
					 pixel_rate, 1, pixel_rate);
	}

	ar0234->crop_rect.left = (AR0234_ARRAY_WIDTH - mode->width) / 2;
	ar0234->crop_rect.top  = (AR0234_ARRAY_HEIGHT - mode->height) / 2;
	ar0234->crop_rect.width = mode->width;
	ar0234->crop_rect.height = mode->height;
	/* 确保不会越界（防止除零或负值） */
	if (ar0234->crop_rect.left < 0)
		ar0234->crop_rect.left = 0;
	if (ar0234->crop_rect.top < 0)
		ar0234->crop_rect.top = 0;
unlock:
	mutex_unlock(&ar0234->mutex);
	return ret;
}

static int ar0234_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	const struct ar0234_mode *mode = ar0234->cur_mode;
	u32 code = ar0234->monochrome ? MEDIA_BUS_FMT_Y10_1X10 :
				       MEDIA_BUS_FMT_SGRBG10_1X10;

	mutex_lock(&ar0234->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ar0234->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = code;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ar0234->mutex);
	return 0;
}

static int ar0234_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	if (code->index > 0)
		return -EINVAL;
	code->code = ar0234->monochrome ? MEDIA_BUS_FMT_Y10_1X10 :
					 MEDIA_BUS_FMT_SGRBG10_1X10;
	return 0;
}

static int ar0234_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ar0234_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static int ar0234_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	fi->interval = ar0234->cur_mode->max_fps;
	return 0;
}

static int ar0234_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	config->type = V4L2_MBUS_CSI2_DPHY;
	return 0;
}

/* ------------------------------------------------------------------
 * Rockchip specific IOCTL
 * ------------------------------------------------------------------ */
static void ar0234_get_module_inf(struct ar0234 *ar0234,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, AR0234_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ar0234->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ar0234->len_name, sizeof(inf->base.lens));
}


static int ar0234_set_stream_reg(struct ar0234 *ar0234, bool on)
{
	u16 val = on ? AR0234_RESET_STREAM_ON : AR0234_RESET_STREAM_OFF;

	return ar0234_write_reg(ar0234->client, AR0234_REG_RESET,
				AR0234_REG_VALUE_16BIT, val);
}

static long ar0234_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ar0234_get_module_inf(ar0234, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		ret = ar0234_set_stream_reg(ar0234, !!stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long ar0234_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf)
			return -ENOMEM;
		ret = ar0234_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ar0234_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif

/* ------------------------------------------------------------------
 * Power management
 * ------------------------------------------------------------------ */
static inline u32 ar0234_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, AR0234_XVCLK_FREQ / 1000 / 1000);
}

static int ar0234_check_sensor_id(struct ar0234 *ar0234)
{
	u32 id = 0;
	int ret;

	ret = ar0234_read_reg(ar0234->client, AR0234_REG_CHIP_ID,
			      AR0234_REG_VALUE_16BIT, &id);
	if (ret)
		return ret;

	if (id == AR0234_CHIP_ID) {
		dev_info(&ar0234->client->dev, "AR0234 detected\n");
	} else {
		dev_err(&ar0234->client->dev, "Unexpected sensor id (0x%x)\n", id);
		return -ENODEV;
	}
	return 0;
}

static int __ar0234_power_on(struct ar0234 *ar0234)
{
	int ret;

	ret = clk_prepare_enable(ar0234->xvclk);
	if (ret) {
		dev_err(&ar0234->client->dev, "Failed to enable xvclk\n");
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(ar0234_regulator),
				    ar0234->supplies);
	if (ret) {
		clk_disable_unprepare(ar0234->xvclk);
		dev_err(&ar0234->client->dev, "Failed to enable supplies\n");
		return ret;
	}

	ar0234->power_on = true;
	return 0;
}

static void __ar0234_power_off(struct ar0234 *ar0234)
{
	regulator_bulk_disable(ARRAY_SIZE(ar0234_regulator),
			       ar0234->supplies);
	clk_disable_unprepare(ar0234->xvclk);
	ar0234->power_on = false;
}

static int ar0234_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	int ret = 0;

	mutex_lock(&ar0234->mutex);

	if (on) {
		ret = pm_runtime_get_sync(&ar0234->client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&ar0234->client->dev);
		}
	} else {
		pm_runtime_put(&ar0234->client->dev);
	}

	mutex_unlock(&ar0234->mutex);
	return ret;
}

/* ------------------------------------------------------------------
 * Streaming control
 * ------------------------------------------------------------------ */
static int ar0234_soft_reset(struct ar0234 *ar0234)
{
	int ret;

	usleep_range(20000, 21000);
	ret = ar0234_write_reg(ar0234->client, AR0234_REG_RESET,
			       AR0234_REG_VALUE_16BIT, 0x0001);
	usleep_range(200000, 201000);
	ret = ar0234_write_reg(ar0234->client, AR0234_REG_RESET,
			       AR0234_REG_VALUE_16BIT, AR0234_RESET_DEFAULT);
	return ret;
}


static int __ar0234_start_stream(struct ar0234 *ar0234)
{
	
	int ret;
	int tm = (ar0234->trigger_mode >= 0) ? ar0234->trigger_mode : trigger_mode;
	u16 digital_test = 0x0028;
	dev_info(&ar0234->client->dev, "AR0234 Starting stream\n");
	/* Reset */
	ret = ar0234_soft_reset(ar0234);
	if (ret)
		{
			dev_err(&ar0234->client->dev, "AR0234: Start stream error: Reset failed\n");
			return ret;}

	/* PLL / MIPI */
	ret = ar0234_write_array(ar0234->client, ar0234_pll_24_450_10bit_regs);
	if (ret)
			{
			dev_err(&ar0234->client->dev, "AR0234: Start stream error: PLL / MIPI failed\n");
			return ret;}

	/* Set number of lanes */
	ret = ar0234_write_reg(ar0234->client, AR0234_REG_SERIAL_FORMAT,
			       AR0234_REG_VALUE_16BIT,
			       0x0200 | ar0234->num_data_lanes);
	if (ret)
		{
			dev_err(&ar0234->client->dev, "AR0234: Start stream error: Set number of lanes failed\n");
			return ret;
		}

	/* Common init */
	ret = ar0234_write_array(ar0234->client, ar0234_common_regs);
	if (ret)
		{
			dev_err(&ar0234->client->dev, "AR0234: Start stream error: Common init failed\n");
			return ret;
		}

	

	if (ar0234->monochrome)
		digital_test |= BIT(7); /* MONO_CHROME_OPERATION */

	ret = ar0234_write_reg(ar0234->client, 0x30B0,
				AR0234_REG_VALUE_16BIT, digital_test);
	if (ret)
		return ret;

	/* Pixel clock related settings */
	ret = ar0234_write_array(ar0234->client, ar0234_pixclk_90mhz_regs);
	if (ret)
		{
			dev_err(&ar0234->client->dev, "AR0234: Start stream error: Pixel clock settings failed\n");
			return ret;
		}

	/* Mode specific windowing */
	ret = ar0234_write_array(ar0234->client, ar0234->cur_mode->reg_list);
	if (ret)
		{
			dev_err(&ar0234->client->dev, "AR0234: Start stream error: Mode specific windowing failed\n");
			return ret;
		}

	ret = ar0234_write_reg(ar0234->client, AR0234_REG_HTS,
		       AR0234_REG_VALUE_16BIT,
		       AR0234_LINE_LENGTH_PCK_DEF);
	ret |= ar0234_write_reg(ar0234->client, AR0234_REG_VTS,
				AR0234_REG_VALUE_16BIT,
				ar0234->cur_mode->vts_def);
	if (ret)
		return ret;

	/* Flash control */
	if (ar0234->flash_enable) {
		u16 flash_val = AR0234_FLASH_ENABLE |
				(u8)ar0234->flash_delay;
		ret = ar0234_write_reg(ar0234->client, AR0234_REG_LED_FLASH_CONTROL,
				       AR0234_REG_VALUE_16BIT, flash_val);
		if (ret)
			{
				dev_err(&ar0234->client->dev, "AR0234: Start stream error: Flash control failed\n");
				return ret;

			}
	}

	/* Apply V4L2 controls */
	mutex_unlock(&ar0234->mutex);
	ret = v4l2_ctrl_handler_setup(&ar0234->ctrl_handler);
	mutex_lock(&ar0234->mutex);
	if (ret)
		{
			dev_err(&ar0234->client->dev, "AR0234: Start stream error: Control setup failed\n");
			return ret;

		}

	/* Enter streaming mode (or trigger mode) */
	if (tm == AR0234_TRIGGER_MODE_OFF) {
		ret = ar0234_set_stream_reg(ar0234, true);
		if (ret)
			dev_err(&ar0234->client->dev, "AR0234: failed to set stream on\n");
	} else {
		u16 reset_val = AR0234_RESET_DEFAULT | AR0234_RESET_GPI_EN |
				AR0234_RESET_FORCED_PLL_ON;

		if (tm == AR0234_TRIGGER_MODE_SLAVE_SYNC) {
			reset_val |= AR0234_RESET_STREAM;
			ret = ar0234_write_reg(ar0234->client, AR0234_REG_GRR_CONTROL1,
					     AR0234_REG_VALUE_16BIT,
					     AR0234_GRR_SLAVE_SH_SYNC);
		}
		ret = ar0234_write_reg(ar0234->client, AR0234_REG_RESET,
				       AR0234_REG_VALUE_16BIT, reset_val);
	}
	return ret;
}

static int __ar0234_stop_stream(struct ar0234 *ar0234)
{
	return ar0234_set_stream_reg(ar0234, false);
}

static int ar0234_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct i2c_client *client = ar0234->client;
	int ret = 0;

	dev_info(&client->dev, "AR0234: %s stream\n", on ? "Start" : "Stop");
	mutex_lock(&ar0234->mutex);
	on = !!on;
	if (on == ar0234->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			dev_err(&client->dev, "AR0234: Failed to get power: %d\n", ret);
			goto unlock_and_return;
		}

		ret = __ar0234_start_stream(ar0234);
		if (ret) {
			v4l2_err(sd, "AR0234: start stream failed\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ar0234_stop_stream(ar0234);
		pm_runtime_put(&client->dev);
	}

	ar0234->streaming = on;
unlock_and_return:
	mutex_unlock(&ar0234->mutex);
	return ret;
}

static int ar0234_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = AR0234_PIXEL_ARRAY_LEFT;
		sel->r.top = AR0234_PIXEL_ARRAY_TOP;
		sel->r.width = AR0234_PIXEL_ARRAY_WIDTH;
		sel->r.height = AR0234_PIXEL_ARRAY_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = ar0234->crop_rect;
		return 0;
	default:
		return -EINVAL;
	}
}

/* ------------------------------------------------------------------
 * V4L2 Controls
 * ------------------------------------------------------------------ */
static int ar0234_set_analog_gain(struct ar0234 *ar0234, u16 gain)
{
	int ret;
	u16 mfr_val = ar0234->mfr_30ba;
	u16 new_mfr;

	/* Determine 0x30BA value based on pixel clock and gain level.
	 * For 90 MHz (4-lane) we use the following thresholds.
	 */
	if (gain < 0x20)
		new_mfr = AR0234_MFR_30BA_GAIN_BITS(2);
	else if (gain < 0x3A)
		new_mfr = AR0234_MFR_30BA_GAIN_BITS(1);
	else
		new_mfr = AR0234_MFR_30BA_GAIN_BITS(0);

	if (mfr_val != new_mfr) {
		/* Use grouped parameter hold */
		ret = ar0234_write_reg(ar0234->client, 0x3022,
				       AR0234_REG_VALUE_08BIT, 1);
		ret = ar0234_write_reg(ar0234->client, AR0234_REG_MFR_30BA,
				       AR0234_REG_VALUE_16BIT, new_mfr);
		ret = ar0234_write_reg(ar0234->client, AR0234_REG_ANALOG_GAIN,
				       AR0234_REG_VALUE_16BIT, gain);
		ret = ar0234_write_reg(ar0234->client, 0x3022,
				       AR0234_REG_VALUE_08BIT, 0);
		if (ret)
			return ret;
		ar0234->mfr_30ba = new_mfr;
	} else {
		ret = ar0234_write_reg(ar0234->client, AR0234_REG_ANALOG_GAIN,
				       AR0234_REG_VALUE_16BIT, gain);
	}

	return ret;
}

static int ar0234_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0234 *ar0234 = container_of(ctrl->handler,
					     struct ar0234, ctrl_handler);
	struct i2c_client *client = ar0234->client;
	int ret = 0;

	if (pm_runtime_enabled(&client->dev)) {
        ret = pm_runtime_get_if_in_use(&client->dev);
        if (ret <= 0 && ret != -EINVAL) // -EINVAL 意味着 PM 还没启用
            return 0;
    }

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ar0234_write_reg(client, AR0234_REG_EXPOSURE_COARSE,
				       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ar0234_set_analog_gain(ar0234, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = ar0234_write_reg(client, AR0234_REG_DIGITAL_GAIN,
				       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = ar0234_write_reg(client, AR0234_REG_VTS,
			       AR0234_REG_VALUE_16BIT,
			       ctrl->val + ar0234->cur_mode->height);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = ar0234_write_reg(client, AR0234_REG_IMAGE_ORIENTATION,
				       AR0234_REG_VALUE_08BIT,
				       (ar0234->vflip->val << 1) |
				       ar0234->hflip->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ar0234_write_reg(client, AR0234_REG_TEST_PATTERN_MODE,
				       AR0234_REG_VALUE_16BIT,
				       ar0234_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = ar0234_write_reg(client, AR0234_REG_TEST_DATA_RED,
				       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = ar0234_write_reg(client, AR0234_REG_TEST_DATA_GREENR,
				       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = ar0234_write_reg(client, AR0234_REG_TEST_DATA_BLUE,
				       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = ar0234_write_reg(client, AR0234_REG_TEST_DATA_GREENB,
				       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "AR0234: Unhandled ctrl id 0x%x\n", ctrl->id);
		break;
	}

	if (pm_runtime_enabled(&client->dev))
        pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops ar0234_ctrl_ops = {
	.s_ctrl = ar0234_set_ctrl,
};

static int ar0234_initialize_controls(struct ar0234 *ar0234)
{
	const struct ar0234_mode *mode = ar0234->cur_mode;
	struct v4l2_ctrl_handler *handler = &ar0234->ctrl_handler;
	s64 exposure_max, vblank_def, pixel_rate;
	u32 h_blank, fps;
	int i, ret;

	dev_info(&ar0234->client->dev, "AR0234: Initializing controls\n");
	ret = v4l2_ctrl_handler_init(handler, 16);
	if (ret)
		return ret;
	handler->lock = &ar0234->mutex;

	ar0234->hflip = v4l2_ctrl_new_std(handler, &ar0234_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	ar0234->vflip = v4l2_ctrl_new_std(handler, &ar0234_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

    exposure_max = mode->vts_def - 1;
	ar0234->exposure = v4l2_ctrl_new_std(handler, &ar0234_ctrl_ops,
				V4L2_CID_EXPOSURE, AR0234_EXPOSURE_MIN,
				exposure_max, AR0234_EXPOSURE_STEP,
				mode->exp_def);

	ar0234->anal_gain = v4l2_ctrl_new_std(handler, &ar0234_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN,
				AR0234_ANA_GAIN_MIN, AR0234_ANA_GAIN_MAX,
				AR0234_ANA_GAIN_STEP, AR0234_ANA_GAIN_DEFAULT);

	ar0234->digi_gain = v4l2_ctrl_new_std(handler, &ar0234_ctrl_ops,
				V4L2_CID_DIGITAL_GAIN,
				AR0234_DGTL_GAIN_MIN, AR0234_DGTL_GAIN_MAX,
				AR0234_DGTL_GAIN_STEP, AR0234_DGTL_GAIN_DEFAULT);

	h_blank = mode->hts_def - mode->width;
	ar0234->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ar0234->hblank)
		ar0234->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ar0234->vblank = v4l2_ctrl_new_std(handler, &ar0234_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				AR0234_FLL_MAX - mode->height,
				1, vblank_def);
	v4l2_ctrl_new_int_menu(&ar0234->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
			       0, 0, link_freq_menu_items);

	fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
					mode->max_fps.numerator);
	pixel_rate = (s64)mode->vts_def * mode->hts_def * fps;
	ar0234->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
					0, INT_MAX, 1, pixel_rate);

	ar0234->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ar0234_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ar0234_test_pattern_menu) - 1,
				0, 0, ar0234_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		v4l2_ctrl_new_std(handler, &ar0234_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  AR0234_TEST_PATTERN_COLOR_MIN,
				  AR0234_TEST_PATTERN_COLOR_MAX,
				  AR0234_TEST_PATTERN_COLOR_STEP,
				  AR0234_TEST_PATTERN_COLOR_MAX);
	}

	

	dev_info(&ar0234->client->dev, "AR0234: Controls initialized\n");

	if (handler->error) {
		ret = handler->error;
		dev_err(&ar0234->client->dev,
			"AR0234: Failed to init controls (%d)\n", ret);
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	ret = v4l2_ctrl_handler_setup(handler);
	if (ret) {
		dev_err(&ar0234->client->dev,
			"AR0234: Failed to setup controls (%d)\n", ret);
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	ar0234->subdev.ctrl_handler = handler;
	return 0;
}

/* ------------------------------------------------------------------ */

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ar0234_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ar0234_mode *def_mode = &supported_modes[0];

	mutex_lock(&ar0234->mutex);
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = ar0234->monochrome ? MEDIA_BUS_FMT_Y10_1X10 :
				     MEDIA_BUS_FMT_SGRBG10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&ar0234->mutex);
	return 0;
}

static const struct v4l2_subdev_internal_ops ar0234_internal_ops = {
	.open = ar0234_open,
};
#endif

static const struct v4l2_subdev_core_ops ar0234_core_ops = {
	.s_power = ar0234_s_power,
	.ioctl = ar0234_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ar0234_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ar0234_video_ops = {
	.s_stream = ar0234_s_stream,
	.g_frame_interval = ar0234_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0234_pad_ops = {
	.enum_mbus_code = ar0234_enum_mbus_code,
	.enum_frame_size = ar0234_enum_frame_sizes,
	.enum_frame_interval = ar0234_enum_frame_interval,
	.get_fmt = ar0234_get_fmt,
	.set_fmt = ar0234_set_fmt,
	.get_mbus_config = ar0234_g_mbus_config,
	.get_selection = ar0234_get_selection,
};

static const struct v4l2_subdev_ops ar0234_subdev_ops = {
	.core	= &ar0234_core_ops,
	.video	= &ar0234_video_ops,
	.pad	= &ar0234_pad_ops,
};

/* ------------------------------------------------------------------
 * Probe / Remove
 * ------------------------------------------------------------------ */


static int ar0234_configure_regulators(struct ar0234 *ar0234)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(ar0234_regulator); i++)
        ar0234->supplies[i].supply = ar0234_regulator[i].name;

    return devm_regulator_bulk_get(&ar0234->client->dev,
                                   ARRAY_SIZE(ar0234_regulator),
                                   ar0234->supplies);
}

static int ar0234_parse_dt(struct ar0234 *ar0234)
{
	struct device *dev = &ar0234->client->dev;
	struct device_node *node = dev->of_node;
	u32 sensor_type = 1;
	int ret;
	u32 tm;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ar0234->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ar0234->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ar0234->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ar0234->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	/* Trigger mode (optional) */
	ret = of_property_read_u32(node, "trigger-mode", &tm);
	ar0234->trigger_mode = (ret == 0) ? tm : -1;

	/* Flash settings (optional) */
	ar0234->flash_enable = of_property_read_bool(node, "flash");
	if (ar0234->flash_enable) {
		u32 lead = 0, lag = 0;
		of_property_read_u32(node, "flash-lead", &lead);
		of_property_read_u32(node, "flash-lag", &lag);
		if (lead)
			ar0234->flash_delay = -(s8)lead;
		else if (lag)
			ar0234->flash_delay = (s8)lag;
	}

	

	ret = of_property_read_u32(node, "sensor-type", &sensor_type);
	if (!ret)
		ar0234->monochrome = !!sensor_type;
	else
		ar0234->monochrome = true;

	dev_info(dev, "%s sensor detected\n",
		ar0234->monochrome ? "Monochrome" : "Color");

	return 0;
}

static int ar0234_parse_endpoint(struct ar0234 *ar0234)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct device *dev = &ar0234->client->dev;
	int ret;

	ret = v4l2_fwnode_endpoint_parse(ar0234->subdev.fwnode, &ep);
	if (ret) {
		dev_err(dev, "failed to parse endpoint: %d\n", ret);
		return ret;
	}

	ar0234->num_data_lanes = ep.bus.mipi_csi2.num_data_lanes;

	if (ar0234->num_data_lanes != 1 &&
	    ar0234->num_data_lanes != 2 &&
	    ar0234->num_data_lanes != 4) {
		dev_err(dev, "invalid data lanes: %u\n", ar0234->num_data_lanes);
		return -EINVAL;
	}

	dev_info(dev, "data lanes: %u\n", ar0234->num_data_lanes);
	return 0;
}
static int ar0234_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ar0234 *ar0234;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ar0234 = devm_kzalloc(dev, sizeof(*ar0234), GFP_KERNEL);
	if (!ar0234)
		return -ENOMEM;

	mutex_init(&ar0234->mutex);

	ar0234->client = client;

	ret = ar0234_parse_dt(ar0234);
	if (ret)
		return ret;

	ar0234->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ar0234->xvclk)) {
		dev_err(dev, "Error %ld getting clock\n", 
			PTR_ERR(ar0234->xvclk));
		return -EPROBE_DEFER;
	}

	ar0234->cur_mode = &supported_modes[0];
	ar0234->cfg_num = ARRAY_SIZE(supported_modes);

	ar0234->crop_rect.left = (AR0234_ARRAY_WIDTH - ar0234->cur_mode->width) / 2;
	ar0234->crop_rect.top  = (AR0234_ARRAY_HEIGHT - ar0234->cur_mode->height) / 2;
	ar0234->crop_rect.width = ar0234->cur_mode->width;
	ar0234->crop_rect.height = ar0234->cur_mode->height;

	sd = &ar0234->subdev;
	v4l2_i2c_subdev_init(sd, client, &ar0234_subdev_ops);
	sd->fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!sd->fwnode) {
		dev_err(dev, "Failed to get fwnode endpoint\n");
		return -EINVAL;
	}
	ret = ar0234_parse_endpoint(ar0234);
	if (ret)
		return ret;
	/* Power on to detect sensor */
	ret = ar0234_configure_regulators(ar0234);
	if (ret) {
		dev_err(dev, "Failed to get regulators\n");
		return ret;
	}
	ret = __ar0234_power_on(ar0234);
	if (ret)
		goto err_destroy_mutex;

	ret = ar0234_check_sensor_id(ar0234);
	if (ret)
		goto err_power_off;

	/* Initialize controls */
	ret = ar0234_initialize_controls(ar0234);
	if (ret)
		goto err_power_off;

	// ar0234->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	// if (IS_ERR(ar0234->reset_gpio))
	// 	dev_warn(dev, "Failed to get reset-gpios\n");

	// ar0234->pwdn_gpio = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_LOW);
	// if (IS_ERR(ar0234->pwdn_gpio))
	// 	dev_warn(dev, "Failed to get pwdn-gpios\n");

	/* Hardcode 4 lanes, 90 MHz pixel rate */
	// ar0234->num_data_lanes = 4;

	/* Initialize cached MFR value */
	ar0234->mfr_30ba = AR0234_MFR_30BA_DEFAULT;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ar0234_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	ar0234->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ar0234->pad);
	if (ret < 0)
		goto err_free_handler;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ar0234->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ar0234->module_index, facing,
		 AR0234_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_free_handler:
	v4l2_ctrl_handler_free(&ar0234->ctrl_handler);
err_power_off:
	__ar0234_power_off(ar0234);
err_destroy_mutex:
	mutex_destroy(&ar0234->mutex);
	return ret;
}

static int ar0234_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ar0234->ctrl_handler);
	mutex_destroy(&ar0234->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ar0234_power_off(ar0234);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static int ar0234_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	__ar0234_power_off(ar0234);
	return 0;
}

static int ar0234_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	return __ar0234_power_on(ar0234);
}

static const struct dev_pm_ops ar0234_pm_ops = {
	SET_RUNTIME_PM_OPS(ar0234_runtime_suspend,
			   ar0234_runtime_resume, NULL)
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ar0234_of_match[] = {
	{ .compatible = "onnn,ar0234cs" },
	{},
};
MODULE_DEVICE_TABLE(of, ar0234_of_match);
#endif

static const struct i2c_device_id ar0234_match_id[] = {
	{ "onnn,ar0234cs", 0 },
	{ },
};

static struct i2c_driver ar0234_i2c_driver = {
	.driver = {
		.name = AR0234_NAME,
		.pm = &ar0234_pm_ops,
		.of_match_table = of_match_ptr(ar0234_of_match),
	},
	.probe		= &ar0234_probe,
	.remove		= &ar0234_remove,
	.id_table	= ar0234_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ar0234_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ar0234_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OnSemi AR0234 sensor driver (Rockchip port)");
MODULE_LICENSE("GPL v2");