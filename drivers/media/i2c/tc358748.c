// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Toshiba TC358748 CSI-2 to PARALLEL bridge
 *
 */

//#define DEBUG 1

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/property.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/media-entity.h>

#define HZ_PER_MHZ              1000000UL

/* 16-bit registers */
#define CHIPID_REG			0x0000
#define		CHIPID			GENMASK(15, 8)
#define		REVID			GENMASK(7, 0)

#define SYSCTL_REG			0x0002
#define		SRESET			BIT(0)
#define 	SLEEP 			BIT(1)

#define CONFCTL_REG			0x0004
#define		PDATAF_MASK		GENMASK(9, 8)
#define		PDATAF_MODE0	0
#define		PDATAF_MODE1	1
#define		PDATAF_MODE2	2
#define		PDATAF(val)		FIELD_PREP(PDATAF_MASK, (val))
#define		PPEN			BIT(6)
#define		VVALIDP			BIT(5)
#define		HVALIDP			BIT(4)
#define		PCLKP			BIT(3)
#define		DATALANE_MASK	GENMASK(1, 0)

#define FIFOCTL_REG			0x0006
#define DATAFMT_REG			0x0008
#define		PDFMT(val)		FIELD_PREP(GENMASK(7, 4), (val))
#define		UDT_EN			BIT(0)

#define GPIOEN_REG			0x000E
#define GPIODIR_REG			0x0010

#define MCLKCTL_REG			0x000c
#define		MCLK_HIGH_MASK	GENMASK(15, 8)
#define		MCLK_LOW_MASK	GENMASK(7, 0)
#define		MCLK_HIGH(val)	FIELD_PREP(MCLK_HIGH_MASK, (val))
#define		MCLK_LOW(val)	FIELD_PREP(MCLK_LOW_MASK, (val))

#define PLLCTL0_REG			0x0016
#define		PLL_PRD_MASK	GENMASK(15, 12)
#define		PLL_PRD(val)	FIELD_PREP(PLL_PRD_MASK, (val))
#define		PLL_FBD_MASK	GENMASK(8, 0)
#define		PLL_FBD(val)	FIELD_PREP(PLL_FBD_MASK, (val))

#define PLLCTL1_REG			0x0018
#define		PLL_FRS_MASK	GENMASK(11, 10)
#define		PLL_FRS(val)	FIELD_PREP(PLL_FRS_MASK, (val))
#define		CKEN			BIT(4)
#define		RESETB			BIT(1)
#define		PLL_EN			BIT(0)

#define CLKCTL_REG			0x0020
#define		MCLKDIV_MASK	GENMASK(3, 2)
#define		MCLKDIV(val)	FIELD_PREP(MCLKDIV_MASK, (val))
#define		MCLKDIV_8		0
#define		MCLKDIV_4		1
#define		MCLKDIV_2		2

#define WORDCNT_REG			0x0022
#define PP_MISC_REG			0x0032
#define		FRMSTOP			BIT(15)
#define		RSTPTR			BIT(14)

#define DBG_ACT_LINE_CNT   0x00E0
#define DBG_LINE_WIDTH     0x00E2
#define DBG_VERT_BLANK_LINE_CNT     0x00E4
#define DBG_VIDEO_DATA     0x00E8


#define PHYCLKCTL_REG		0x0056
#define PHYDATA0CTL_REG		0x0058
#define PHYDATA1CTL_REG		0x005A
#define PHYDATA2CTL_REG		0x005C
#define PHYDATA3CTL_REG		0x005E
#define PHYTIMDLY_REG		0x0060
#define		TC_TERM_SEL		BIT(15)
#define		TD_TERM_SEL		BIT(7)
#define		DSETTLE_MASK	GENMASK(6, 0)
#define		DSETTLE(val)	FIELD_PREP(DSETTLE_MASK, (val))

#define PHYSTA_REG			0x0062
#define CSISTATUS_REG		0x0064
#define CSIERREN_REG		0x0066
#define MDLSYNERR_REG		0x0068
#define CSIDID_REG			0x006A
#define CSIDIDERR_REG		0x006C
#define CSIPKTLEN_REG		0x006E
#define CSIRX_DPCTL_REG		0x0070

#define FRMERRCNT_REG		0x0080
#define CRCERRCNT_REG		0x0082
#define CORERRCNT_REG		0x0084
#define HDRERRCNT_REG		0x0086
#define EIDERRCNT_REG		0x0088
#define CTLERRCNT_REG		0x008A
#define SOTERRCNT_REG		0x008C
#define SYNERRCNT_REG		0x008E
#define MDLERRCNT_REG		0x0090
#define FIFOSTATUS_REG		0x00F8

/* 32-bit registers */
#define CLW_DPHYCONTTX_REG		0x0100
#define CLW_CNTRL_REG			0x0140
#define D0W_CNTRL_REG			0x0144
#define		LANEDISABLE		BIT(0)
#define D1W_CNTRL_REG			0x0148
#define D2W_CNTRL_REG			0x014C

#define STARTCNTRL_REG			0x0204
#define		START			BIT(0)

#define PPISTATUS_REG			0x0208
#define LINEINITCNT_REG			0x0210
#define LPTXTIMECNT_REG			0x0214
#define TCLK_HEADERCNT_REG		0x0218
#define		TCLK_ZEROCNT(val)	FIELD_PREP(GENMASK(15, 8), (val))
#define		TCLK_PREPARECNT(val)	FIELD_PREP(GENMASK(6, 0), (val))

#define TCLK_TRAILCNT_REG		0x021C
#define THS_HEADERCNT_REG		0x0220
#define		THS_ZEROCNT(val)	FIELD_PREP(GENMASK(14, 8), (val))
#define		THS_PREPARECNT(val)	FIELD_PREP(GENMASK(6, 0), (val))

#define TWAKEUP_REG			0x0224
#define TCLK_POSTCNT_REG		0x0228
#define THS_TRAILCNT_REG		0x022C
#define HSTXVREGEN_REG			0x0234
#define TXOPTIONCNTRL_REG		0x0238
#define CSI_CONTROL_REG			0x040C
#define		CSI_MODE		BIT(15)
#define		TXHSMD			BIT(7)
#define		NOL(val)		FIELD_PREP(GENMASK(2, 1), (val))

#define CSI_CONFW_REG			0x0500
#define		MODE(val)		FIELD_PREP(GENMASK(31, 29), (val))
#define		MODE_SET		0x5
#define		ADDRESS(val)		FIELD_PREP(GENMASK(28, 24), (val))
#define		CSI_CONTROL_ADDRESS	0x3
#define		DATA(val)		FIELD_PREP(GENMASK(15, 0), (val))

#define CSI_START_REG			0x0518
#define		STRT			BIT(0)

//#undef MCLK_ENABLE
#define MCLK_ENABLE

static const struct v4l2_mbus_framefmt tc358748_def_sink_fmt = {
	.width		= 640,
	.height		= 480,
	.code		= MEDIA_BUS_FMT_SRGGB8_1X8,
	.field		= V4L2_FIELD_NONE,
	.colorspace	= V4L2_COLORSPACE_DEFAULT,
	.ycbcr_enc	= V4L2_YCBCR_ENC_DEFAULT,
	.quantization	= V4L2_QUANTIZATION_DEFAULT,
	.xfer_func	= V4L2_XFER_FUNC_DEFAULT,
};

static const struct v4l2_mbus_framefmt tc358748_def_source_fmt = {
	.width		= 640,
	.height		= 480,
	.code		= MEDIA_BUS_FMT_RGB888_1X24,
	.field		= V4L2_FIELD_NONE,
	.colorspace	= V4L2_COLORSPACE_DEFAULT,
	.ycbcr_enc	= V4L2_YCBCR_ENC_DEFAULT,
	.quantization	= V4L2_QUANTIZATION_DEFAULT,
	.xfer_func	= V4L2_XFER_FUNC_DEFAULT,
};

static const char * const tc358748_supply_name[] = {
	"vddc", "vddio", "vddmipi"
};

#define TC358748_NUM_SUPPLIES		ARRAY_SIZE(tc358748_supply_name)

enum {
	TC358748_SINK,
	TC358748_SOURCE,
	TC358748_NR_PADS
};

struct tc358748_dev {
	struct v4l2_subdev sd;
	struct media_pad pads[TC358748_NR_PADS];

	struct v4l2_ctrl_handler	ctrl_hdl;

	/* endpoints info */
	struct v4l2_fwnode_endpoint tx;
	struct v4l2_fwnode_endpoint rx;
	/* remote source */
	struct v4l2_async_subdev asd;
	struct v4l2_async_notifier notifier;

	struct clk *refclk;
	unsigned long refclk_freq;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[TC358748_NUM_SUPPLIES];

#ifdef MCLK_ENABLE
	struct clk_hw	mclk_hw;
	unsigned long	mclk_rate;
	u8				mclk_prediv;
	u16				mclk_postdiv;
#endif // MCLK_ENABLE

	unsigned long	pll_rate;
	u8				pll_post_div;
	u16				pll_pre_div;
	u16				pll_mul;

	struct i2c_client *i2c_client;

	/* lock to protect all members below */
	struct mutex lock;
	struct v4l2_mbus_framefmt sink_fmt;
	struct v4l2_mbus_framefmt source_fmt;

#define TC358748_VB_MAX_SIZE		(511 * 32)
#define TC358748_VB_DEFAULT_SIZE	 (32 * 32)
	unsigned int	vb_size; /* Video buffer size in bits */

	u8	hsync_active;
	u8	vsync_active;
};

#define notifier_to_tc358748(n) container_of(n, struct tc358748_dev, notifier)


struct tc358748_format {
	u32		code;
	bool 	csi_format;
	unsigned char	bpp;
	/* Register values */
	u8		pdformat; /* Peripheral Data Format */
	u8		pdataf;   /* Parallel Data Format Option */
};

enum {
	PDFORMAT_RAW8 = 0,
	PDFORMAT_RAW10,
	PDFORMAT_RAW12,
	PDFORMAT_RGB888,
	PDFORMAT_RGB666,
	PDFORMAT_RGB565,
	PDFORMAT_YUV422_8BIT,
	/* RESERVED = 7 */
	PDFORMAT_RAW14 = 8,
	PDFORMAT_YUV422_10BIT,
	PDFORMAT_YUV444,
};

/* Check tc358748_src_mbus_code() if you add new formats */
static const struct tc358748_format tc358748_formats[] = {	
	{
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.bpp = 8,
		.pdformat = PDFORMAT_RAW8,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.bpp = 8,
		.pdformat = PDFORMAT_RAW8,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.bpp = 8,
		.pdformat = PDFORMAT_RAW8,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.bpp = 8,
		.pdformat = PDFORMAT_RAW8,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.bpp = 10,
		.pdformat = PDFORMAT_RAW10,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.bpp = 10,
		.pdformat = PDFORMAT_RAW10,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.bpp = 10,
		.pdformat = PDFORMAT_RAW10,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.bpp = 10,
		.pdformat = PDFORMAT_RAW10,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.bpp = 12,
		.pdformat = PDFORMAT_RAW12,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.bpp = 12,
		.pdformat = PDFORMAT_RAW12,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.bpp = 12,
		.pdformat = PDFORMAT_RAW12,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.bpp = 12,
		.pdformat = PDFORMAT_RAW12,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SBGGR14_1X14,
		.bpp = 14,
		.pdformat = PDFORMAT_RAW14,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SGBRG14_1X14,
		.bpp = 14,
		.pdformat = PDFORMAT_RAW14,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SGRBG14_1X14,
		.bpp = 14,
		.pdformat = PDFORMAT_RAW14,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.bpp = 14,
		.pdformat = PDFORMAT_RAW14,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.bpp = 24,
		.pdformat = PDFORMAT_RGB888,
		.pdataf = PDATAF_MODE0, /* don't care */
	}, {
		.code = MEDIA_BUS_FMT_UYVY8_2X8, //
		.bpp = 16,
		.pdformat = PDFORMAT_YUV422_8BIT,
		.pdataf = PDATAF_MODE0,
	}, {
		.code = MEDIA_BUS_FMT_UYVY8_1X16, //
		.bpp = 16,
		.pdformat = PDFORMAT_YUV422_8BIT,
		.pdataf = PDATAF_MODE1,
	}, {
		.code = MEDIA_BUS_FMT_YUYV8_1X16, //
		.bpp = 16,
		.pdformat = PDFORMAT_YUV422_8BIT,
		.pdataf = PDATAF_MODE2,
	}, {
		.code = MEDIA_BUS_FMT_UYVY10_2X10, //
		.bpp = 20,
		.pdformat = PDFORMAT_YUV422_10BIT,
		.pdataf = PDATAF_MODE0, /* don't care */
	}
};

/* Get n-th format for pad */
static const struct tc358748_format *
tc358748_get_format_by_idx(unsigned int pad, unsigned int index)
{
	unsigned int idx = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tc358748_formats); i++) {
		const struct tc358748_format *fmt = &tc358748_formats[i];

		if ((pad == TC358748_SINK) ||
		    (pad == TC358748_SOURCE)) {
			if (idx == index)
				return fmt;
			idx++;
		}
	}

	return ERR_PTR(-EINVAL);
}

static const struct tc358748_format *
tc358748_get_format_by_code(unsigned int pad, u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tc358748_formats); i++) {
		const struct tc358748_format *fmt = &tc358748_formats[i];

//		if (pad == TC358748_SOURCE && fmt->code == code)
//			return fmt;

//		if (pad == TC358748_SINK && !fmt->csi_format)
//			continue;

		if (fmt->code == code)
			return fmt;

	}
	return ERR_PTR(-EINVAL);
}

static u32 tc358748_src_mbus_code(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		return MEDIA_BUS_FMT_UYVY8_1X16;
	case MEDIA_BUS_FMT_UYVY10_2X10:
		return MEDIA_BUS_FMT_UYVY10_1X20;
	default:
		return code;
	}
}

static inline struct tc358748_dev *to_tc358748_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358748_dev, sd);
}

#ifdef MCLK_ENABLE
static inline struct tc358748_dev *clk_hw_to_tc358748(struct clk_hw *hw)
{
	return container_of(hw, struct tc358748_dev, mclk_hw);
}
#endif // MCLK_ENABLE

static inline void fsleep(unsigned long usecs)
{       
        if (usecs <= 10)
                udelay(usecs);
        else if (usecs <= 20000)
                usleep_range(usecs, 2 * usecs);
        else 
                msleep(DIV_ROUND_UP(usecs, 1000));
}

static int tc358748_read(struct tc358748_dev *bridge, u16 reg, u32 *val)
{
	struct i2c_client *client = bridge->i2c_client;
	int ret;
	__be16 data16;
	__be32 data32;

	data16 = cpu_to_be16(reg);
	ret = i2c_master_send(client, (char *)&data16, 2);
	if (ret < 2) {
			dev_err(&client->dev, "%s: i2c read error, reg: 0x%x\n",
					__func__, reg);
			return ret < 0 ? ret : -EIO;
	}

	if (reg < CLW_DPHYCONTTX_REG) {
			ret = i2c_master_recv(client, (char *)&data16, 2);
			*val = be16_to_cpu(data16);
	} else {
			ret = i2c_master_recv(client, (char *)&data32, 4);
			*val = __swahw32(be32_to_cpu(data32));
	}

	if (ret < 1) {
			dev_err(&client->dev, "%s: i2c read error, reg: 0x%x\n",
					__func__, reg);
			return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int tc358748_write(struct tc358748_dev *bridge, u16 reg, u32 val)
{
	struct i2c_client *client = bridge->i2c_client;
	int ret;
	int size;
	__be16 data16;
	char data[6];

	data16 = cpu_to_be16(reg);
	memcpy(data, &data16, sizeof(data16));
	size = sizeof(data16);
	if (reg < CLW_DPHYCONTTX_REG) {
			data16 = cpu_to_be16(val);
			memcpy(data + size, &data16, sizeof(data16));
			size += sizeof(data16);
	} else {
			data16 = cpu_to_be16(val & 0xffff);
			memcpy(data + size, &data16, sizeof(data16));
			size += sizeof(data16);
			data16 = cpu_to_be16(val >> 16);
			memcpy(data + size, &data16, sizeof(data16));
			size += sizeof(data16);
	}
	ret = i2c_master_send(client, data, size);
	if (ret < size) {
			dev_err(&client->dev, "%s: i2c write error, reg: 0x%x\n",
					__func__, reg);
			return ret < 0 ? ret : -EIO;
	}
			
	return 0;
}               

static int tc358748_update_bits(struct tc358748_dev *bridge, u32 reg, u32 mask, u32 val)
{
	u32 tmp, orig;
	int err;

	err = tc358748_read(bridge, reg, &orig);
	if (err)
		return err;

	tmp = orig & ~mask;
	tmp |= val & mask;

	return tc358748_write(bridge, reg, tmp);
}

static int tc358748_set_bits(struct tc358748_dev *bridge, u32 reg, u32 bits)
{
	return tc358748_update_bits(bridge, reg, bits, bits);
}

static int tc358748_clear_bits(struct tc358748_dev *bridge, u32 reg, u32 bits)
{
	return tc358748_update_bits(bridge, reg, bits, 0);
}

static struct v4l2_subdev *tc358748_get_remote_sd(struct v4l2_subdev *sd)
{
	struct tc358748_dev *bridge = to_tc358748_dev(sd);
	struct device *dev = bridge->sd.dev;

	struct media_pad    *r_pad;
	struct v4l2_subdev  *r_sd;

	if(!bridge) {
		return NULL;
	}

	if(&bridge->pads[TC358748_SINK] == NULL) {
		return NULL;
	}

	r_pad = media_entity_remote_pad(&bridge->pads[TC358748_SINK]);


	if (r_pad == NULL || !is_media_entity_v4l2_subdev(r_pad->entity)) {
		dev_err(dev, "Nothing connected to input pad\n");
		return NULL;
	}

	r_sd = media_entity_to_v4l2_subdev(r_pad->entity);
	if (!r_sd) {
		dev_err(dev, "No connected subdev found\n");
		return NULL;
	}

	return r_sd;
}

static __u32 get_fmt_code(__u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tc358748_formats); i++) {
		const struct tc358748_format *fmt = &tc358748_formats[i];

		if (code == fmt->code)
			return code;
	}

	return tc358748_formats[0].code;
}

static __u32 serial_to_parallel_code(__u32 serial)
{
	switch (serial) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		return MEDIA_BUS_FMT_UYVY8_1X16;
	case MEDIA_BUS_FMT_UYVY10_2X10:
		return MEDIA_BUS_FMT_UYVY10_1X20;
	default:
		return serial;
	}
}

static int tc358748_get_regulators(struct tc358748_dev *bridge)
{
	unsigned int i;

	for (i = 0; i < TC358748_NUM_SUPPLIES; i++)
		bridge->supplies[i].supply = tc358748_supply_name[i];

	return devm_regulator_bulk_get(&bridge->i2c_client->dev,
				       TC358748_NUM_SUPPLIES,
				       bridge->supplies);
}

static int tc358748_apply_reset(struct tc358748_dev *bridge)
{
	gpiod_set_value_cansleep(bridge->reset_gpio, 0);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(bridge->reset_gpio, 1);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(bridge->reset_gpio, 0);
	usleep_range(5000, 10000);
	return 0;
}

static int tc358748_apply_sw_reset(struct tc358748_dev *bridge)
{
	int err;

	err = tc358748_set_bits(bridge, SYSCTL_REG, SRESET);
	if (err)
		return err;

	fsleep(100);

	return tc358748_clear_bits(bridge, SYSCTL_REG, SRESET);
}


static int tc358748_set_power_on(struct tc358748_dev *bridge)
{
	struct device *dev = bridge->sd.dev;
	int ret = 0;

//	ret = regulator_bulk_enable(TC358748_NUM_SUPPLIES, bridge->supplies);
//	if (ret) {
//		dev_err(dev, "%s: failed to enable regulators\n", __func__);
//		return ret;
//	}

	if (bridge->reset_gpio) {
		dev_dbg(dev, "apply reset");
		tc358748_apply_reset(bridge);
	} else {
		dev_dbg(dev, "apply sw reset");
		ret = tc358748_apply_sw_reset(bridge);
		if (ret) {
			dev_err(dev, "%s: failed to apply sw reset\n", __func__);
		}
	}
	return ret;
}

static void tc358748_set_power_off(struct tc358748_dev *bridge)
{
//	regulator_bulk_disable(TC358748_NUM_SUPPLIES, bridge->supplies);
}

static int tc358748_detect(struct tc358748_dev *bridge)
{
	struct device *dev = bridge->sd.dev;
	unsigned int chipid, revid;
	u32 val;
	int err;

	err = tc358748_read(bridge, CHIPID_REG, &val);
	if (err)
		return -ENODEV;

	chipid = FIELD_GET(CHIPID, val);
	revid = FIELD_GET(REVID, val);
	
	if (chipid != 0x44) {
		dev_err(bridge->sd.dev, "Invalid Chip ID 0x%02x\n", chipid);
		return -ENODEV;
	}

	dev_info(dev, "TC358748: Chip ID 0x%02x, Rev ID 0x%02x\n", chipid, revid);

	return 0;
}

static int tc358748_apply_pll_config(struct tc358748_dev *bridge)
{
	struct v4l2_subdev *sd = &bridge->sd;
	struct device *dev = sd->dev;

	u8 post = bridge->pll_post_div;
	u16 pre = bridge->pll_pre_div;
	u16 mul = bridge->pll_mul;
	u32 val, mask;
	int err;

	err = tc358748_read(bridge, PLLCTL1_REG, &val);
	if (err)
		return err;

	/* Don't touch the PLL if running */
	if (FIELD_GET(PLL_EN, val) == 1) {
		dev_dbg(dev, "PLL already running\n");
		return 0;
	}

	/* Pre-div and Multiplicator have a internal +1 logic */
	val = PLL_PRD(pre - 1) | PLL_FBD(mul - 1);
	mask = PLL_PRD_MASK | PLL_FBD_MASK;
	dev_dbg(dev, "PLLCTL0: 0x%x\n", val);
	err = tc358748_update_bits(bridge, PLLCTL0_REG, mask, val);
	if (err)
		return err;

	val = PLL_FRS(ilog2(post)) | RESETB | PLL_EN;
	mask = PLL_FRS_MASK | RESETB | PLL_EN;
	dev_dbg(dev, "PLLCTL1: 0x%x\n", val);
	tc358748_update_bits(bridge, PLLCTL1_REG, mask, val);
	if (err)
		return err;

	fsleep(1000);

	return tc358748_set_bits(bridge, PLLCTL1_REG, CKEN);
}


static unsigned long tc358748_find_pll_settings(struct tc358748_dev *bridge,
						unsigned long refclk, unsigned long fout)
{
	struct device *dev = bridge->sd.dev;
	unsigned long best_freq = 0;
	u32 min_delta = 0xffffffff;
	u16 prediv_max = 17;
	u16 prediv_min = 1;
	u16 m_best, mul;
	u16 p_best, p;
	u8 postdiv;

	if (fout > 1000 * HZ_PER_MHZ) {
		dev_err(dev, "HS-Clock above 1 Ghz are not supported\n");
		return 0;
	}

	if (fout >= 500 * HZ_PER_MHZ)
		postdiv = 1;
	else if (fout >= 250 * HZ_PER_MHZ)
		postdiv = 2;
	else if (fout >= 125 * HZ_PER_MHZ)
		postdiv = 4;
	else
		postdiv = 8;

	for (p = prediv_min; p <= prediv_max; p++) {
		unsigned long delta, fin;
		u64 tmp;

		fin = DIV_ROUND_CLOSEST(refclk, p);
		if (fin < 4 * HZ_PER_MHZ || fin > 40 * HZ_PER_MHZ)
			continue;

		tmp = fout * p * postdiv;
		do_div(tmp, fin);
		mul = tmp;
		if (mul > 511)
			continue;

		tmp = mul * fin;
		do_div(tmp, p * postdiv);

		delta = abs(fout - tmp);
		if (delta < min_delta) {
			p_best = p;
			m_best = mul;
			min_delta = delta;
			best_freq = tmp;
		};

		if (delta == 0)
			break;
	};

	if (!best_freq) {
		dev_err(dev, "Failed find PLL frequency\n");
		return 0;
	}

	bridge->pll_post_div = postdiv;
	bridge->pll_pre_div = p_best;
	bridge->pll_mul = m_best;

	if (best_freq != fout)
		dev_warn(dev, "Request PLL freq:%lu, found PLL freq:%lu\n",
			 fout, best_freq);

	dev_dbg(dev, "Found PLL settings: refclk:%lu freq:%lu prediv:%u multi:%u postdiv:%u\n",
		refclk, best_freq, p_best, m_best, postdiv);

	return best_freq;
}

#ifdef MCLK_ENABLE
static int tc358748_mclk_enable(struct tc358748_dev *bridge)
{
	unsigned int div;
	u32 val;
	int err;

	div = bridge->mclk_postdiv / 2;
	val = MCLK_HIGH(div - 1) | MCLK_LOW(div - 1);
	dev_dbg(bridge->sd.dev, "MCLKCTL: %u (0x%x)\n", val, val);
	err = tc358748_write(bridge, MCLKCTL_REG, val);
	if (err)
		return err;

	if (bridge->mclk_prediv == 8)
		val = MCLKDIV(MCLKDIV_8);
	else if (bridge->mclk_prediv == 4)
		val = MCLKDIV(MCLKDIV_4);
	else
		val = MCLKDIV(MCLKDIV_2);

	dev_dbg(bridge->sd.dev, "CLKCTL[MCLKDIV]: %u (0x%x)\n", val, val);

	return tc358748_update_bits(bridge, CLKCTL_REG, MCLKDIV_MASK, val);
}
#endif // MCLK_ENABLE

#ifdef MCLK_ENABLE
static long tc358748_find_mclk_settings(struct tc358748_dev *bridge, unsigned long mclk_rate)
{
	unsigned long pll_rate = bridge->pll_rate;
	const unsigned char prediv[] = { 2, 4, 8 };
	unsigned int mclk_prediv, mclk_postdiv;
	struct device *dev = bridge->sd.dev;
	unsigned int postdiv, mclkdiv;
	unsigned long best_mclk_rate;
	unsigned int i;

	/*
	 *                          MCLK-Div
	 *           -------------------´`---------------------
	 *          ´                                          `
	 *         +-------------+     +------------------------+
	 *         | MCLK-PreDiv |     |       MCLK-PostDiv     |
	 * PLL --> |   (2/4/8)   | --> | (mclk_low + mclk_high) | --> MCLK
	 *         +-------------+     +------------------------+
	 *
	 * The register value of mclk_low/high is mclk_low/high+1, i.e.:
	 *   mclk_low/high = 1   --> 2 MCLK-Ref Counts
	 *   mclk_low/high = 255 --> 256 MCLK-Ref Counts == max.
	 * If mclk_low and mclk_high are 0 then MCLK is disabled.
	 *
	 * Keep it simple and support 50/50 duty cycles only for now,
	 * so the calc will be:
	 *
	 *   MCLK = PLL / (MCLK-PreDiv * 2 * MCLK-PostDiv)
	 */

	if (mclk_rate == bridge->mclk_rate)
		return mclk_rate;

	/* Highest possible rate */
	mclkdiv = pll_rate / mclk_rate;
	if (mclkdiv <= 8) {
		mclk_prediv = 2;
		mclk_postdiv = 4;
		best_mclk_rate = pll_rate / (2 * 4);
		goto out;
	}

	/* First check the prediv */
	for (i = 0; i < ARRAY_SIZE(prediv); i++) {
		postdiv = mclkdiv / prediv[i];

		if (postdiv % 2)
			continue;

		if (postdiv >= 4 && postdiv <= 512) {
			mclk_prediv = prediv[i];
			mclk_postdiv = postdiv;
			best_mclk_rate = pll_rate / (prediv[i] * postdiv);
			goto out;
		}
	}

	/* No suitable prediv found, so try to adjust the postdiv */
	for (postdiv = 4; postdiv <= 512; postdiv += 2) {
		unsigned int pre;

		pre = mclkdiv / postdiv;
		if (pre == 2 || pre == 4 || pre == 8) {
			mclk_prediv = pre;
			mclk_postdiv = postdiv;
			best_mclk_rate = pll_rate / (pre * postdiv);
			goto out;
		}
	}

	/* The MCLK <-> PLL gap is to high -> use largest possible div */
	mclk_prediv = 8;
	mclk_postdiv = 512;
	best_mclk_rate = pll_rate / (8 * 512);

out:
	bridge->mclk_prediv = mclk_prediv;
	bridge->mclk_postdiv = mclk_postdiv;
	bridge->mclk_rate = best_mclk_rate;

	if (best_mclk_rate != mclk_rate)
		dev_warn(dev, "Request MCLK freq:%lu, found MCLK freq:%lu\n",
			 mclk_rate, best_mclk_rate);

	dev_dbg(dev, "Found MCLK settings: freq:%lu prediv:%u postdiv:%u\n",
		best_mclk_rate, mclk_prediv, mclk_postdiv);

	return best_mclk_rate;
}
#endif // MCLK_ENABLE

static int tc358748_enable_csi_to_parallel(struct v4l2_subdev *sd)
{
	struct tc358748_dev *bridge = to_tc358748_dev(sd);
	struct device *dev = bridge->sd.dev;
#if 0
	unsigned int lanes;
#endif
	const struct tc358748_format *sink_fmt;
	const struct tc358748_format *source_fmt;
	struct v4l2_subdev_format  sd_fmt;
	struct v4l2_subdev *remote;
	u32 val;
	int err = 0;

	remote = tc358748_get_remote_sd(sd);
	if (!remote) {
		dev_err(dev, "unable to find remote subdev.\n");
		return -ENODEV;
	}

	/* Get the format from the input */
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.pad   = 0;

	err = v4l2_subdev_call(remote, pad, get_fmt, NULL, &sd_fmt);
	if (err) {
		dev_err(dev, "unable to get fmt from subdev.\n");
		return err;
	}
	sink_fmt = tc358748_get_format_by_code(TC358748_SINK, sd_fmt.format.code);
	if (IS_ERR(sink_fmt)){
		dev_err(dev, "Unsupported format code 0x%X.\n", sd_fmt.format.code);
		return PTR_ERR(sink_fmt);
	}
	source_fmt = tc358748_get_format_by_code(TC358748_SOURCE, MEDIA_BUS_FMT_RGB888_1X24);
	if (IS_ERR(source_fmt)){
		dev_err(dev, "Unsupported format code 0x%X.\n", sd_fmt.format.code);
		return PTR_ERR(source_fmt);
	}


#if 1

	/* Set normal operation */
	err = tc358748_clear_bits(bridge, SYSCTL_REG, SLEEP);
	if (err) {
		dev_err(dev, "unable to set SYSCTL.\n");
		return err;
	}

	/* Clear previous error */
	tc358748_write(bridge, CSISTATUS_REG, 0xffff);

	/*=========== PLL,Clock Setting ===========*/
	err = tc358748_write(bridge, PLLCTL0_REG, 0x209F); //PLL Control Register 0 (PLL_PRD,PLL_FBD)
	if (err) {
		dev_err(dev, "unable to set PLLCTL0.\n");
		return err;
	}
	err = tc358748_write(bridge, PLLCTL1_REG, 0x1003); //PLL_FRS,PLL_LBWS, PLL oscillation enable
	if (err) {
		dev_err(dev, "unable to set PLLCTL1.\n");
		return err;
	}
	fsleep(1000);
	err = tc358748_write(bridge, PLLCTL1_REG, 0x1013); //PLL_FRS,PLL_LBWS, PLL clock out enable
	if (err) {
		dev_err(dev, "unable to set PLLCTL1.\n");
		return err;
	}
	err = tc358748_write(bridge, CLKCTL_REG, 0x0022); //CLK control register: Clock divider setting
	if (err) {
		dev_err(dev, "unable to set CLKCTL.\n");
		return err;
	}

#ifdef MCLK_ENABLE
	/*=========== MCLK Output ===========*/
	err = tc358748_write(bridge, MCLKCTL_REG, 0x0101); //MCLK duty setting
	if (err) {
		dev_err(dev, "unable to set MCLKCTL.\n");
		return err;
	}
#endif // MCLK_ENABLE

	/*=========== Format configuration, timing Setting ===========*/

	/* After activating the remote device, we will start the auto-calibration */
	val = 0x8004;
	dev_dbg(dev, "PHYTIMDLY: 0x%x\n", val);
	err = tc358748_write(bridge, PHYTIMDLY_REG, val); //PHY timing delay setting
	if (err) {
		dev_err(dev, "unable to set PHYTIMDLY.\n");
		return err;
	}

	//val = bridge->vb_size / 32;
	val = 0x80;
	dev_dbg(dev, "FIFOCTL: %u (0x%x)\n", val, val);
	err = tc358748_write(bridge, FIFOCTL_REG, val); //FIFO control
	if (err) {
		dev_err(dev, "unable to set FIFOCTL.\n");
		return err;
	}
	
	val = sd_fmt.format.width * sink_fmt->bpp / 8;
	dev_dbg(dev, "WORDCNT: %u (0x%x)\n", val, val);
	err = tc358748_write(bridge, WORDCNT_REG, val); // Total number of bytes for each line/width
	if (err) {
		dev_err(dev, "unable to set WORDCNT\n");
		return err;
	}

	err = tc358748_write(bridge, DATAFMT_REG, 0x0031); //Data format control
	if (err) {
		dev_err(dev, "unable to set DATAFMT.\n");
		return err;
	}
	err = tc358748_write(bridge, CONFCTL_REG, 0x0045); //Configuration control
	if (err) {
		dev_err(dev, "unable to set CONFCTL.\n");
		return err;
	}
	return tc358748_write(bridge, CTLERRCNT_REG, 0);

    /*
    tc358748_write(bridge, DBG_ACT_LINE_CNT, 0x8000);
	tc358748_write(bridge, DBG_LINE_WIDTH, 0x396); // 918
	tc358748_write(bridge, DBG_VERT_BLANK_LINE_CNT, 0x0000);
*/
    //int i = 0;
	/*
	for (i = 0; i < 400; i++)
    {
//		tc358748_write(bridge, DBG_VIDEO_DATA, 0x1612)
//		tc358748_write(bridge, DBG_VIDEO_DATA, 0x2c22);
		tc358748_write(bridge, DBG_VIDEO_DATA, 0x5832);
    }
	tc358748_write(bridge, DBG_ACT_LINE_CNT, 0xC1e0);
*/
/*
	for (i = 0; i < 80; i++)
		tc358748_write(bridge, DBG_VIDEO_DATA, 0xff7f);
	tc358748_write(bridge, DBG_VIDEO_DATA, 0xff00);

	for (i = 0; i < 40; i++)
		tc358748_write(bridge, DBG_VIDEO_DATA, 0xffff);
	tc358748_write(bridge, DBG_VIDEO_DATA, 0xc0ff);
	
    for (i = 0; i < 40; i++)
		tc358748_write(bridge, DBG_VIDEO_DATA, 0xc000);
	
    for (i = 0; i < 80; i++)
		tc358748_write(bridge, DBG_VIDEO_DATA, 0x7f00);
	
    for (i = 0; i < 80; i++)
		tc358748_write(bridge, DBG_VIDEO_DATA, 0x7fff);
	tc358748_write(bridge, DBG_VIDEO_DATA, 0x0000);
	
    for (i = 0; i < 40; i++)
		tc358748_write(bridge, DBG_VIDEO_DATA, 0x00ff);
	tc358748_write(bridge, DBG_VIDEO_DATA, 0x00ff);
	
    for (i = 0; i < 40; i++)
		tc358748_write(bridge, DBG_VIDEO_DATA, 0x0000);
	tc358748_write(bridge, DBG_VIDEO_DATA, 0x007f);

	tc358748_write(bridge, DBG_ACT_LINE_CNT, 0xC1DF);

*/

#else

	lanes = bridge->rx.bus.mipi_csi2.num_data_lanes;

	err = tc358748_apply_pll_config(bridge);
	if (err) {
		dev_err(dev, "unable to apply pll settings.\n");
		return err;
	}

#ifdef MCLK_ENABLE
	err = tc358748_mclk_enable(bridge);
	if (err) {
		dev_err(dev, "unable to apply mclk settings.\n");
		return err;
	}
#endif // MCLK_ENABLE

	val = bridge->vb_size / 32;
	dev_dbg(dev, "FIFOCTL: %u (0x%x)\n", val, val);
	err = tc358748_write(bridge, FIFOCTL_REG, val);
	if (err) {
		dev_err(dev, "unable to setup FIFO level.\n");
		return err;
	}

	/* Self defined CSI user data type id's are not supported yet */
	val = PDFMT(fmt->pdformat);
	dev_dbg(dev, "DATAFMT: 0x%x\n", val);
	err = tc358748_write(bridge, DATAFMT_REG, val);
	if (err) {
		dev_err(dev, "unable to set DATAFMT\n");
		return err;
	}

	//CSIRX: use Data Type ID detected from CSI Bus
	err = tc358748_clear_bits(bridge, DATAFMT_REG, UDT_EN);
	if (err) {
		dev_err(dev, "unable to clear UDT_EN in DATAFMT\n");
		return err;
	}

	/* After activating the remote device, we will start the auto-calibration */
	err = tc358748_write(bridge, PHYTIMDLY_REG, 0x8009);
	if (err) {
		dev_err(dev, "unable to set PHYTIMDLY\n");
		return err;
	}

	//CSIRX: set additional capacitance, bias resistance, clock skew
	//MIPI PHY Clock Lane
//	err = tc358748_write(bridge, PHYCLKCTL_REG, 0);
//	if (err)
//		return err;
	//MIPI PHY Data Lane 0
//	err = tc358748_write(bridge, PHYDATA0CTL_REG, 0);
//	if (err)
//		return err;
	//MIPI PHY Data Lane 1
//	err = tc358748_write(bridge, PHYDATA1CTL_REG, 0);
//	if (err)
//		return err;

//	err = tc358748_write(bridge, PP_MISC_REG, 0);
//	if (err) {
//		dev_err(dev, "unable to write PP_MISC\n");
//		return err;
//	}

	val = PDATAF(fmt->pdataf);
	dev_dbg(dev, "CONFCTL[PDATAF]: 0x%x\n", fmt->pdataf);
	err = tc358748_update_bits(bridge, CONFCTL_REG, PDATAF_MASK, val);
	if (err) {
		dev_err(dev, "unable to write CONFCTL[PDATAF]\n");
		return err;
	}

	err = tc358748_update_bits(bridge, CONFCTL_REG, DATALANE_MASK, lanes - 1);
	if (err) {
		dev_err(dev, "unable to write CONFCTL[DATALANE]\n");
		return err;
	}

	if(bridge->hsync_active)
		err = tc358748_clear_bits(bridge, CONFCTL_REG, HVALIDP); //active high
	else
		err = tc358748_set_bits(bridge, CONFCTL_REG, HVALIDP); //active low
	if (err) {
		dev_err(dev, "unable to write CONFCTL[HVALIDP]\n");
		return err;
	}

	if(bridge->vsync_active)
		err = tc358748_clear_bits(bridge, CONFCTL_REG, VVALIDP); //active high
	else
		err = tc358748_set_bits(bridge, CONFCTL_REG, VVALIDP); //active low
	if (err) {
		dev_err(dev, "unable to write CONFCTL[VVALIDP]\n");
		return err;
	}

	//Normal Parallel Clock (PCLK) Polarity
	err = tc358748_clear_bits(bridge, CONFCTL_REG, PCLKP);
	if (err) {
		dev_err(dev, "unable to write CONFCTL[PCLKP]\n");
		return err;
	}

	err = tc358748_update_bits(bridge, CONFCTL_REG, DATALANE_MASK, lanes - 1);
	if (err) {
		dev_err(dev, "unable to write CONFCTL[DATALANE]\n");
		return err;
	}

	return tc358748_set_bits(bridge, CONFCTL_REG, PPEN); //Parallel Port Enable
#endif
}


static int tc358748_status(struct tc358748_dev *bridge)
{
	struct device *dev = bridge->sd.dev;
	unsigned	 wait;
	u32 rxdata;

	/* Default to 30ms */
	wait = 100;
	msleep(wait); /* In order to receive some packets */

	dev_dbg(dev, "====================\n");
	tc358748_read(bridge, PHYSTA_REG, &rxdata);
	dev_dbg(dev, "PHYSTA=0x%X\n", rxdata);
	tc358748_read(bridge, CSISTATUS_REG, &rxdata);
	dev_dbg(dev, "CSISTATUS=0x%X\n", rxdata);
	tc358748_read(bridge, CSIERREN_REG, &rxdata);
	dev_dbg(dev, "CSIERREN=0x%X\n", rxdata);
	tc358748_read(bridge, MDLSYNERR_REG, &rxdata);
	dev_dbg(dev, "MDLSYNERR=0x%X\n", rxdata);
	tc358748_read(bridge, CSIDID_REG, &rxdata);
	dev_dbg(dev, "CSIDID=0x%X\n", rxdata);
	tc358748_read(bridge, CSIDIDERR_REG, &rxdata);
	dev_dbg(dev, "CSIDIDERR=0x%X\n", rxdata);
	tc358748_read(bridge, CSIPKTLEN_REG, &rxdata);
	dev_dbg(dev, "CSIPKTLEN=0x%X\n", rxdata);
	tc358748_read(bridge, CSIRX_DPCTL_REG, &rxdata);
	dev_dbg(dev, "CSIRX_DPCTL=0x%X\n", rxdata);

	tc358748_read(bridge, FRMERRCNT_REG, &rxdata);
	dev_dbg(dev, "FRMERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, CRCERRCNT_REG, &rxdata);
	dev_dbg(dev, "CRCERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, CORERRCNT_REG, &rxdata);
	dev_dbg(dev, "CORERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, HDRERRCNT_REG, &rxdata);
	dev_dbg(dev, "HDRERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, EIDERRCNT_REG, &rxdata);
	dev_dbg(dev, "EIDERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, CTLERRCNT_REG, &rxdata);
	dev_dbg(dev, "CTLERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, SOTERRCNT_REG, &rxdata);
	dev_dbg(dev, "SOTERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, SYNERRCNT_REG, &rxdata);
	dev_dbg(dev, "SYNERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, MDLERRCNT_REG, &rxdata);
	dev_dbg(dev, "MDLERRCNT=0x%X\n", rxdata);
	tc358748_read(bridge, FIFOSTATUS_REG, &rxdata);
	dev_dbg(dev, "FIFOSTATUS=0x%X\n", rxdata);
	tc358748_read(bridge, PP_MISC_REG, &rxdata);
	dev_dbg(dev, "PP_MISC_REG=0x%X\n", rxdata);

	tc358748_read(bridge, 0x00E0, &rxdata);
	dev_dbg(dev, "(DBG_LCNT=0x%X\n", rxdata);
	tc358748_read(bridge, 0x00E2, &rxdata);
	dev_dbg(dev, "DBG_Width=0x%X\n", rxdata);
	tc358748_read(bridge, 0x00E4, &rxdata);
	dev_dbg(dev, "DBG_VBlank=0x%X\n", rxdata);
	tc358748_read(bridge, 0x00E8, &rxdata);
	dev_dbg(dev, "DBG_Data=0x%X\n", rxdata);

	tc358748_read(bridge, CLW_CNTRL_REG, &rxdata);
	dev_dbg(dev, "CLW_CNTRL_REG=0x%X\n", rxdata);
	tc358748_read(bridge, D0W_CNTRL_REG, &rxdata);
	dev_dbg(dev, "D0W_CNTRL_REG=0x%X\n", rxdata);
	tc358748_read(bridge, D1W_CNTRL_REG, &rxdata);
	dev_dbg(dev, "D1W_CNTRL_REG=0x%X\n", rxdata);
	tc358748_read(bridge, D2W_CNTRL_REG, &rxdata);
	dev_dbg(dev, "D2W_CNTRL_REG=0x%X\n", rxdata);

	tc358748_read(bridge, STARTCNTRL_REG, &rxdata);
	dev_dbg(dev, "STARTCNTRL_REG=0x%X\n", rxdata);
	tc358748_read(bridge, PPISTATUS_REG, &rxdata);
	dev_dbg(dev, "PPISTATUS_REG=0x%X\n", rxdata);

	tc358748_read(bridge, MCLKCTL_REG, &rxdata);
	dev_dbg(dev, "MCLKCTL_REG=0x%X\n", rxdata);
	tc358748_read(bridge, 0xE, &rxdata); // GPIOEn
	dev_dbg(dev, "GPIOEn=0x%X\n", rxdata);
	tc358748_read(bridge, 0x10, &rxdata); // GPIO Direction Register
	dev_dbg(dev, "GPIODIR=0x%X\n", rxdata);
	tc358748_read(bridge, 0x12, &rxdata); // GPIO Direction Register
	dev_dbg(dev, "GPIO Pin Value=0x%X\n", rxdata);
	tc358748_read(bridge, 0x14, &rxdata); // GPIO Direction Register
	dev_dbg(dev, "GPIO Output Value=0x%X\n", rxdata);

	dev_dbg(dev, "====================\n");

	return 0;
}

/* Loop through all delay value in order to find the working window. And then
 * set the delay value at its middle
 */
static int tc358748_calibrate(struct tc358748_dev *bridge)
{
	struct device *dev = bridge->sd.dev;
	int			 first_dly = 0;
	int			 state	   = 0;
	unsigned	 error_wait;
	unsigned	 dly;
	u32 val, dsettle, csistatus, physta;

	/* Default to 30ms */
	error_wait = 30;

	for (dly = 0 ; dly <= 0x7f ; dly++) {
		int error;
		val = TC_TERM_SEL | TD_TERM_SEL | DSETTLE(dly);
		tc358748_write(bridge, PHYTIMDLY_REG, val);
		/* Clear previous error measure */
		tc358748_write(bridge, CSISTATUS_REG, 0xffff);
		msleep(error_wait); /* In order to receive some packets */
		/* Read Status */
		tc358748_read(bridge, CSISTATUS_REG, &csistatus);
		tc358748_read(bridge, PHYSTA_REG, &physta);

		//dev_dbg(dev, "dly=%d, CSISTATUS=0x%X, PHYSTA=0x%X\n", dly, csistatus, physta);

		error = ((physta & 0x0055) != 0) || ((csistatus & 0x01FF) != 0);
		/* We hit the first possible value of the PHYTIMDLY window */
		if (!error && !state) {
			/* In fast calibration mode, assume the window is ~10
			 * dly large and set the value in the middle */
#ifdef CONFIG_TC358748_FAST_CALIBRATION
			val = TC_TERM_SEL | TD_TERM_SEL | DSETTLE(dly+ 4);
			tc358748_write(bridge, PHYTIMDLY_REG, val);
			return 0;
#endif
			first_dly = dly;
			state = 1;
		}
		/* We hit the last possible value of the PHYTIMDLY window */
		if (error && state) {
			/* Position the PHYTIMDLY in the middle of the window */
			dsettle = (dly + first_dly) / 2;
			dev_dbg(dev, "Measured PHYTIMDLY: %d\n", dsettle);
			val = TC_TERM_SEL | TD_TERM_SEL | DSETTLE(dsettle);
			tc358748_write(bridge, PHYTIMDLY_REG, val);
			return 0;
		}
	}

	dev_err(dev, "Could not find a correct PHYTIMDLY value\n");
	return -1;
}

static int tc358748_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tc358748_dev *bridge = to_tc358748_dev(sd);
	struct device *dev = bridge->sd.dev;
	struct v4l2_subdev *remote;
	int ret = 0;

	dev_dbg(dev, "%s: %sable\n", __func__, enable ? "en" : "dis");

	remote = tc358748_get_remote_sd(sd);
	if (!remote)
		return -ENODEV;

	mutex_lock(&bridge->lock);
	if(enable == 0) {
		ret = v4l2_subdev_call(remote, video, s_stream, enable);
		if (ret < 0)
			v4l2_err(sd, "cannot call s_stream on remote subdev\n");
		tc358748_set_power_off(bridge);
		goto out;
	}

	ret = tc358748_set_power_on(bridge);
	if (ret)
		goto out;

	ret = tc358748_enable_csi_to_parallel(sd);
	if (ret)
		goto out;

	ret = v4l2_subdev_call(remote, video, s_stream, enable);
	if (ret)
		goto out;

	ret = tc358748_status(bridge);
//	ret = tc358748_calibrate(bridge);

out:
	mutex_unlock(&bridge->lock);
	return ret;
}

static int tc358748_init_cfg(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_get_try_format(sd, cfg, TC358748_SINK);
	*fmt = tc358748_def_sink_fmt;

	fmt = v4l2_subdev_get_try_format(sd, cfg, TC358748_SOURCE);
	*fmt = tc358748_def_source_fmt;
	fmt->code = tc358748_src_mbus_code(tc358748_def_source_fmt.code);

	return 0;
}


static int tc358748_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	const struct tc358748_format *fmt;

	fmt = tc358748_get_format_by_idx(code->pad, code->index);
	if (IS_ERR(fmt))
		return PTR_ERR(fmt);

	code->code = fmt->code;

	return 0;
}

static int tc358748_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct tc358748_dev *bridge = to_tc358748_dev(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_subdev	*remote = tc358748_get_remote_sd(sd);

	//dev_dbg(dev, "%s probe %d", __func__, format->pad);

	if (format->pad >= TC358748_NR_PADS)
		return -EINVAL;

	if (format->pad == TC358748_SINK)
		return -EINVAL;

	if (!remote)
		return -ENODEV;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(remote, cfg, format->pad);
	else
		fmt = &bridge->source_fmt;

	mutex_lock(&bridge->lock);

	*mbus_fmt = *fmt;
	/* code may need to be converted for source */
	if (format->pad == TC358748_SOURCE)
		mbus_fmt->code = serial_to_parallel_code(mbus_fmt->code);

	mutex_unlock(&bridge->lock);

	return 0;
}

static void tc358748_set_fmt_source(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *format)
{
	struct tc358748_dev *bridge = to_tc358748_dev(sd);

	/* source pad mirror active sink pad */
	format->format = bridge->source_fmt;
	/* but code may need to be converted */
	format->format.code = serial_to_parallel_code(format->format.code);

	/* only apply format for V4L2_SUBDEV_FORMAT_TRY case */
	if (format->which != V4L2_SUBDEV_FORMAT_TRY)
		return;

	*v4l2_subdev_get_try_format(sd, cfg, format->pad) = format->format;
}

static void tc358748_set_fmt_sink(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *format)
{
	struct tc358748_dev *bridge = to_tc358748_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	format->format.code = get_fmt_code(format->format.code);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sd, cfg, format->pad);
	else
		fmt = &bridge->sink_fmt;

	*fmt = format->format;
}

static int tc358748_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct tc358748_dev *bridge = to_tc358748_dev(sd);
	struct device *dev = bridge->sd.dev;
	int ret = 0;

	dev_dbg(dev, "%s for %d", __func__, format->pad);

	if (format->pad >= TC358748_NR_PADS)
		return -EINVAL;

	mutex_lock(&bridge->lock);

	if (format->pad == TC358748_SOURCE)
		tc358748_set_fmt_source(sd, cfg, format);
	else
		tc358748_set_fmt_sink(sd, cfg, format);

	mutex_unlock(&bridge->lock);

	return ret;
}

static int __maybe_unused
tc358748_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct tc358748_dev *bridge = to_tc358748_dev(sd);
	/* 32-bit registers starting from CLW_DPHYCONTTX */
	reg->size = reg->reg < CLW_DPHYCONTTX_REG ? 2 : 4;
	tc358748_read(bridge, reg->reg, (u32 *)&reg->val);
	return 0;
}

static int __maybe_unused
tc358748_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct tc358748_dev *bridge = to_tc358748_dev(sd);
	tc358748_write(bridge, (u32)reg->reg, (u32)reg->val);
	return 0;
}

static int tc358748_g_dv_timings(struct v4l2_subdev *sd,
				  struct v4l2_dv_timings *timings)
{
	struct v4l2_subdev *remote = tc358748_get_remote_sd(sd);
	if (!remote)
		return -ENODEV;
	/* Ask the remote directly */
	return v4l2_subdev_call(remote, video, g_dv_timings, timings);
}

//static int tc358748_s_ctrl(struct v4l2_ctrl *ctrl)
//{
//	return 0;
//}

//static int tc358748_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
//{
//	return 0;
//}

//static const struct v4l2_ctrl_ops tc358748_ctrl_ops = {
//	.s_ctrl = tc358748_s_ctrl,
//	.g_volatile_ctrl = tc358748_g_volatile_ctrl,
//};

static const struct v4l2_subdev_core_ops tc358748_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tc358748_g_register,
	.s_register = tc358748_s_register,
#endif
};

static const struct v4l2_subdev_video_ops tc358748_video_ops = {
	.s_stream = tc358748_s_stream,
	.g_dv_timings = tc358748_g_dv_timings,
};

static const struct v4l2_subdev_pad_ops tc358748_pad_ops = {
	.init_cfg = tc358748_init_cfg,
	.enum_mbus_code = tc358748_enum_mbus_code,
	.get_fmt = tc358748_get_fmt,
	.set_fmt = tc358748_set_fmt,
};

static const struct v4l2_subdev_ops tc358748_subdev_ops = {
	.core = &tc358748_core_ops,
	.video = &tc358748_video_ops,
	.pad = &tc358748_pad_ops,
};

static const struct media_entity_operations tc358748_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int tc358748_init_controls(struct tc358748_dev *bridge)
{
//	u64 *link_frequencies = bridge->tx.link_frequencies;
//	struct v4l2_ctrl *ctrl;
	int err;

	err = v4l2_ctrl_handler_init(&bridge->ctrl_hdl, 0);
	if (err)
		return err;

//	ctrl = v4l2_ctrl_new_int_menu(ctrl_hdl, &tc358748_ctrl_ops, V4L2_CID_LINK_FREQ,
//			       0, 0, link_frequencies);
//	if (ctrl)
//		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

//	err = bridge->ctrl_hdl.error;
//	if (err) {
//		v4l2_ctrl_handler_free(&bridge->ctrl_hdl);
//		return err;
//	}

	bridge->sd.ctrl_handler = &bridge->ctrl_hdl;

	return 0;
}

static int tc358748_async_bound(struct v4l2_async_notifier *notifier,
			       struct v4l2_subdev *sd,
			       struct v4l2_async_subdev *asd)
{
	struct tc358748_dev *bridge = to_tc358748_dev(notifier->sd);
	struct device *dev = bridge->sd.dev;
	int ret;
	struct v4l2_subdev *src_subdev;
	int src_pad;

	src_subdev = sd;
	src_pad = media_entity_get_fwnode_pad(&sd->entity, sd->fwnode, MEDIA_PAD_FL_SOURCE);
	if (src_pad < 0) {
		dev_err(dev, "Couldn't find output pad for subdev %s\n", sd->name);
		return src_pad;
	}

	/* Create the media link. */
	dev_dbg(dev, "creating %s:%u -> %s:%u link\n",
			src_subdev->entity.name, src_pad,
			bridge->sd.entity.name, TC358748_SINK);

	ret = media_create_pad_link(&src_subdev->entity, src_pad,
				    &bridge->sd.entity, TC358748_SINK,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);

	if (ret) {
		dev_err(dev, "Couldn't create media link %d", ret);
		return ret;
	}

	dev_dbg(dev, "Media link with subdev %s was created\n", src_subdev->name);
	return ret;
}

static const struct v4l2_async_notifier_operations tc358748_notifier_ops = {
	.bound		= tc358748_async_bound,
};

static int tc358748_parse_tx_ep(struct tc358748_dev *bridge)
{
	struct v4l2_fwnode_endpoint *vep;
	struct i2c_client *client = bridge->i2c_client;
	struct device_node *ep_node;
	unsigned long refclk;
	unsigned long csi_link_rate;
	unsigned char csi_lanes;
	int ret;

	/* parse rx (endpoint 0) */
	ep_node = of_graph_get_endpoint_by_regs(bridge->i2c_client->dev.of_node, 0, 0);
	if (!ep_node) {
		dev_err(&client->dev, "unable to find port0 ep");
		ret = -EINVAL;
		goto error;
	}

	/* Currently we only support 'parallel in' -> 'csi out' */
	vep = &bridge->tx;
	vep->bus_type = V4L2_MBUS_CSI2_DPHY;
	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep_node), vep);
	if (ret) {
		dev_err(&client->dev, "Could not parse sink v4l2 endpoint %d\n", ret);
		goto error_of_node_put;
	}

	/* do some sanity checks */
	csi_lanes = vep->bus.mipi_csi2.num_data_lanes;

	if (csi_lanes == 0 || csi_lanes > 2) {
		dev_err(&client->dev, "max supported data lanes is 2 / got %d\n",	csi_lanes);
		ret = -EINVAL;
		goto error_fwnode;
	}

	if ( vep->nr_of_link_frequencies == 0) {
		dev_err(&client->dev, "error: Invalid CSI-2 settings\n");
		ret = -EINVAL;
		goto error_fwnode;
	}

	refclk = bridge->refclk_freq;
	csi_link_rate = (unsigned long)vep->link_frequencies[0];

	bridge->pll_rate = tc358748_find_pll_settings(bridge, refclk, csi_link_rate * 2);
	if (!bridge->pll_rate) {
		dev_err(&client->dev, "unable to find pll settings. csi_link_rate is %lu\n", csi_link_rate);
		ret = -EINVAL;
		goto error_fwnode;
	}

	bridge->vb_size = TC358748_VB_DEFAULT_SIZE;

	/* register async notifier so we get noticed when sensor is connected */
	bridge->asd.match.fwnode =
		fwnode_graph_get_remote_port_parent(of_fwnode_handle(ep_node));
	bridge->asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
	of_node_put(ep_node);

	v4l2_async_notifier_init(&bridge->notifier);
	ret = v4l2_async_notifier_add_subdev(&bridge->notifier, &bridge->asd);
	if (ret) {
		dev_err(&client->dev, "fail to register asd to notifier %d", ret);
		fwnode_handle_put(bridge->asd.match.fwnode);
		return ret;
	}
	bridge->notifier.ops = &tc358748_notifier_ops;

	ret = v4l2_async_subdev_notifier_register(&bridge->sd, &bridge->notifier);
	if (ret)
		v4l2_async_notifier_cleanup(&bridge->notifier);

	return ret;

error_fwnode:
	v4l2_fwnode_endpoint_free(vep);
error_of_node_put:
	of_node_put(ep_node);
error:

	return ret;
}

static int tc358748_parse_rx_ep(struct tc358748_dev *bridge)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_PARALLEL };
	struct i2c_client *client = bridge->i2c_client;
	struct device_node *ep_node;
	int ret;

	/* parse tx (endpoint 1) */
	ep_node = of_graph_get_endpoint_by_regs(bridge->i2c_client->dev.of_node,
						1, 0);
	if (!ep_node) {
		dev_err(&client->dev, "unable to find port1 ep");
		ret = -EINVAL;
		goto error;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep_node), &ep);
	if (ret) {
		dev_err(&client->dev, "Could not parse v4l2 endpoint\n");
		goto error_of_node_put;
	}

	of_node_put(ep_node);
	bridge->rx = ep;

	return 0;

error_of_node_put:
	of_node_put(ep_node);
error:

	return -EINVAL;
}

static int tc358748_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct tc358748_dev *bridge;
#ifdef MCLK_ENABLE
	u32 mclk_freq;
#endif // MCLK_ENABLE
	int ret;

	bridge = devm_kzalloc(dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge)
		return -ENOMEM;

	bridge->sink_fmt = tc358748_def_sink_fmt;
	bridge->source_fmt = tc358748_def_source_fmt;

	bridge->i2c_client = client;
	v4l2_i2c_subdev_init(&bridge->sd, client, &tc358748_subdev_ops);

	/* got and check clock */
	bridge->refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR(bridge->refclk)) {
		dev_err(dev, "failed to get refclk\n");
		ret = PTR_ERR(bridge->refclk);
		bridge->refclk = NULL;
		return ret;
	}

	ret = clk_prepare_enable(bridge->refclk);
	if (ret) {
		dev_err(dev, "Failed to enable refclk\n");
		return ret;
	}

	bridge->refclk_freq = clk_get_rate(bridge->refclk);

	if (bridge->refclk_freq < 6 * HZ_PER_MHZ ||
	    bridge->refclk_freq > 40 * HZ_PER_MHZ) {
		dev_err(dev,
			"refclk freq must be in 6-40 Mhz range. got %ld Hz\n",
			bridge->refclk_freq);
		return -EINVAL;
	}

	bridge->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(bridge->reset_gpio)) {
		dev_err(dev, "failed to get reset GPIO\n");
		return PTR_ERR(bridge->reset_gpio);
	}

	ret = tc358748_get_regulators(bridge);
	if (ret) {
		dev_err(dev, "failed to get regulators %d\n", ret);
		return ret;
	}

	mutex_init(&bridge->lock);
	bridge->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	bridge->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	bridge->sd.entity.ops = &tc358748_subdev_entity_ops;
	bridge->pads[TC358748_SINK].flags = MEDIA_PAD_FL_SINK;
	bridge->pads[TC358748_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&bridge->sd.entity, TC358748_NR_PADS, bridge->pads);
	if (ret) {
		dev_err(dev, "pads init failed %d", ret);
		goto mutex_cleanup;
	}
	else
		dev_dbg(dev, "pads initialized\n");


	/* enable clock, power and reset device if available */
	ret = tc358748_set_power_on(bridge);
	if (ret)
		goto entity_cleanup;

	ret = tc358748_detect(bridge);
	if (ret) {
		dev_err(dev, "failed to detect tc358748 %d\n", ret);
		goto power_off;
	}

	ret = tc358748_parse_tx_ep(bridge);
	if (ret) {
		dev_err(dev, "failed to parse tx %d\n", ret);
		goto power_off;
	}

	ret = tc358748_parse_rx_ep(bridge);
	if (ret) {
		dev_err(dev, "failed to parse rx %d\n", ret);
		goto err_fwnode;
	}

	ret = tc358748_init_controls(bridge);
	if (ret)
		goto unregister_notifier;

#ifdef MCLK_ENABLE
	if (of_property_read_u32(client->dev.of_node, "mclk-rate", &mclk_freq)) {
		dev_err(&client->dev, "Not found mclk-rate property in device-tree\n");
		return -EINVAL;
	}
	tc358748_find_mclk_settings(bridge, mclk_freq);
#endif // MCLK_ENABLE

	if (of_property_read_u8(client->dev.of_node, "hsync-active", &bridge->hsync_active)) {
		bridge->hsync_active = 1;
	}
	if (of_property_read_u8(client->dev.of_node, "hsync-active", &bridge->vsync_active)) {
		bridge->vsync_active = 1;
	}

	ret = v4l2_async_register_subdev(&bridge->sd);
	if (ret < 0) {
		dev_err(dev, "v4l2_async_register_subdev failed %d\n", ret);
		goto unregister_notifier;
	}
	else
		dev_info(dev, "register sub-device '%s'\n", bridge->sd.name);

	dev_info(dev, "%s found @ 0x%x (%s)\n", client->name,
						client->addr, client->adapter->name);

	{
		u32 rxdata;
		tc358748_read(bridge, THS_HEADERCNT_REG, &rxdata); // THS_ZEROCNT[6:0]: THS_PREPARECNT[6:0]
		dev_dbg(dev, "THS_HEADERCNT_REG(%X)=0x%X\n", THS_HEADERCNT_REG, rxdata);
		tc358748_read(bridge, TCLK_TRAILCNT_REG, &rxdata); // TCLKTRAILCNT[7:0]
		dev_dbg(dev, "TCLK_TRAILCNT_REG(%X)=0x%X\n", TCLK_TRAILCNT_REG, rxdata);
	}
	return 0;

unregister_notifier:
	v4l2_async_notifier_unregister(&bridge->notifier);
	v4l2_async_notifier_cleanup(&bridge->notifier);
err_fwnode:
	v4l2_fwnode_endpoint_free(&bridge->tx);
power_off:
	tc358748_set_power_off(bridge);
entity_cleanup:
	media_entity_cleanup(&bridge->sd.entity);
mutex_cleanup:
	mutex_destroy(&bridge->lock);

	return ret;
}

static int tc358748_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tc358748_dev *bridge = to_tc358748_dev(sd);

	v4l2_async_notifier_unregister(&bridge->notifier);
	v4l2_async_notifier_cleanup(&bridge->notifier);
	v4l2_async_unregister_subdev(&bridge->sd);
	v4l2_fwnode_endpoint_free(&bridge->tx);

	tc358748_set_power_off(bridge);
	media_entity_cleanup(&bridge->sd.entity);
	mutex_destroy(&bridge->lock);

	return 0;
}

static const struct of_device_id tc358748_dt_ids[] = {
	{ .compatible = "toshiba,tc358748" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tc358748_dt_ids);

static struct i2c_driver tc358748_i2c_driver = {
	.driver = {
		.name  = "tc358748",
		.of_match_table = tc358748_dt_ids,
	},
	.probe_new = tc358748_probe,
	.remove = tc358748_remove,
};

module_i2c_driver(tc358748_i2c_driver);

MODULE_DESCRIPTION("Toshiba TC358748 CSI-2 bridge driver");
MODULE_LICENSE("GPL v2");
