#ifndef _MIPI_TX_PARAM_ST_7701_HD228001C31_ALT0_H_
#define _MIPI_TX_PARAM_ST_7701_HD228001C31_ALT0_H_

#ifndef __UBOOT__
#include <linux/vo_mipi_tx.h>
#include <linux/cvi_comm_mipi_tx.h>
#else
#include <cvi_mipi.h>
#endif

// vendor
#define ST7701_HD228001C31_ALT0_VACT	552
#define ST7701_HD228001C31_ALT0_VSA		2
#define ST7701_HD228001C31_ALT0_VBP		20
#define ST7701_HD228001C31_ALT0_VFP		20

#define ST7701_HD228001C31_ALT0_HACT		368
#define ST7701_HD228001C31_ALT0_HSA		8
#define ST7701_HD228001C31_ALT0_HBP		160
#define ST7701_HD228001C31_ALT0_HFP		160


// calc
/*
#define ST7701_HD228001C31_ALT0_VACT	552
#define ST7701_HD228001C31_ALT0_VSA	10
#define ST7701_HD228001C31_ALT0_VBP	6
#define ST7701_HD228001C31_ALT0_VFP	3

#define ST7701_HD228001C31_ALT0_HACT	368
#define ST7701_HD228001C31_ALT0_HSA	32
#define ST7701_HD228001C31_ALT0_HBP	80
#define ST7701_HD228001C31_ALT0_HFP	48
*/

#define HD22_ALT0_PIXEL_CLK(x) ((x##_VACT + x##_VSA + x##_VBP + x##_VFP) \
	* (x##_HACT + x##_HSA + x##_HBP + x##_HFP) * 75 / 1000)

struct combo_dev_cfg_s dev_cfg_st7701_368x552_alt0 = {
	.devno = 0,
	.lane_id = {MIPI_TX_LANE_0, MIPI_TX_LANE_CLK, MIPI_TX_LANE_1, -1, -1},
	.lane_pn_swap = {false, false, false, false, false},
	.output_mode = OUTPUT_MODE_DSI_VIDEO,
	.video_mode = BURST_MODE,
	.output_format = OUT_FORMAT_RGB_24_BIT,
	.sync_info = {
		.vid_hsa_pixels = ST7701_HD228001C31_ALT0_HSA,
		.vid_hbp_pixels = ST7701_HD228001C31_ALT0_HBP,
		.vid_hfp_pixels = ST7701_HD228001C31_ALT0_HFP,
		.vid_hline_pixels = ST7701_HD228001C31_ALT0_HACT,
		.vid_vsa_lines = ST7701_HD228001C31_ALT0_VSA,
		.vid_vbp_lines = ST7701_HD228001C31_ALT0_VBP,
		.vid_vfp_lines = ST7701_HD228001C31_ALT0_VFP,
		.vid_active_lines = ST7701_HD228001C31_ALT0_VACT,
		.vid_vsa_pos_polarity = true,
		.vid_hsa_pos_polarity = false,
	},
	.pixel_clk = HD22_ALT0_PIXEL_CLK(ST7701_HD228001C31_ALT0),
};

const struct hs_settle_s hs_timing_cfg_st7701_368x552_alt0 = { .prepare = 6, .zero = 32, .trail = 1 };

#ifndef CVI_U8
#define CVI_U8 unsigned char
#endif

static CVI_U8 data_st7701_hd228001c31_alt0_0[] = { 0x11 }; // 1
static CVI_U8 data_st7701_hd228001c31_alt0_1[] = { 0xff, 0x77, 0x01, 0x00, 0x00, 0x13 }; // 6
static CVI_U8 data_st7701_hd228001c31_alt0_2[] = { 0xef, 0x08 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_3[] = { 0xff, 0x77, 0x01, 0x00, 0x00, 0x10 }; // 6
static CVI_U8 data_st7701_hd228001c31_alt0_4[] = { 0xc0, 0x44, 0x00 }; // 3
static CVI_U8 data_st7701_hd228001c31_alt0_5[] = { 0xc1, 0x14, 0x06 }; // 3
static CVI_U8 data_st7701_hd228001c31_alt0_6[] = { 0xc2, 0x07, 0x1f }; // 3
static CVI_U8 data_st7701_hd228001c31_alt0_7[] = { 0xcc, 0x30 }; // 2


// gamma setting
static CVI_U8 data_st7701_hd228001c31_alt0_8[] = { 0xb0, 0x0f, 0x16, 0x1a, 0x07, 0x0d, 0x05, 0x02, 0x09,
	0x08, 0x1f, 0x05, 0x13, 0x10, 0x2b, 0x33, 0x1f }; // 17

static CVI_U8 data_st7701_hd228001c31_alt0_9[] = { 0xb1, 0x0f, 0x12, 0x17, 0x0e ,0x0e ,0x04, 0x00, 0x06,
	0x06, 0x1d, 0x05, 0x13, 0x11, 0x26, 0x2d, 0x1f }; // 17


// power control
static CVI_U8 data_st7701_hd228001c31_alt0_10[] = { 0xff, 0x77, 0x01, 0x00, 0x00, 0x11 }; // 6

// vcom setting
static CVI_U8 data_st7701_hd228001c31_alt0_11[] = { 0xb1, 0x7c }; // 2
// vcom setting end

static CVI_U8 data_st7701_hd228001c31_alt0_12[] = { 0xb2, 0x87 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_13[] = { 0xb3, 0x80 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_14[] = { 0xb5, 0x49 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_15[] = { 0xb7, 0x85 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_16[] = { 0xb8, 0x20 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_17[] = { 0xc0, 0x07 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_18[] = { 0xc1, 0x08 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_19[] = { 0xc2, 0x08 }; // 2
static CVI_U8 data_st7701_hd228001c31_alt0_20[] = { 0xd0, 0x88 }; // 2
// power control end

static CVI_U8 data_st7701_hd228001c31_alt0_21[] = { 0xB1, 0x0F, 0x12, 0x17, 0x0E, 0x0E, 0x04,
	0x00, 0x06, 0x06, 0x1D, 0x05, 0x13, 0x11, 0x26, 0x2D, 0x1F}; // 17


static CVI_U8 data_st7701_hd228001c31_alt0_22[] = { 0xB5, 0x49 };
static CVI_U8 data_st7701_hd228001c31_alt0_23[] = { 0xB7, 0x85 };
static CVI_U8 data_st7701_hd228001c31_alt0_24[] = { 0xB8, 0x20 };
static CVI_U8 data_st7701_hd228001c31_alt0_25[] = { 0xC0, 0x07 };
static CVI_U8 data_st7701_hd228001c31_alt0_26[] = { 0xC1, 0x08 };
static CVI_U8 data_st7701_hd228001c31_alt0_27[] = { 0xC2, 0x08 };
static CVI_U8 data_st7701_hd228001c31_alt0_28[] = { 0xD0, 0x88 };


static CVI_U8 data_st7701_hd228001c31_alt0_29[] = { 0xe0, 0x00, 0x02 };
static CVI_U8 data_st7701_hd228001c31_alt0_30[] = { 0xE1, 0x0A, 0xA0, 0x0C, 0xA0, 0x09, 0xA0, 0x0B,
	0xA0, 0x00, 0x44, 0x44 }; // 12
static CVI_U8 data_st7701_hd228001c31_alt0_31[] = { 0xE2, 0x00, 0x00, 0x44, 0x44, 0x05, 0xA0, 0x00,
       	0x00, 0x05, 0xA0, 0x00, 0x00 }; // 13
static CVI_U8 data_st7701_hd228001c31_alt0_32[] = { 0xE3, 0x00, 0x00, 0x22, 0x22 }; // 5
static CVI_U8 data_st7701_hd228001c31_alt0_33[] = { 0xe4, 0x44, 0x44 }; // 3
static CVI_U8 data_st7701_hd228001c31_alt0_34[] = { 0xE5, 0x11, 0x3D, 0x0A, 0xC0, 0x13, 0x3F, 0x0A,
	0xC0, 0x0D, 0x39, 0x0A, 0xC0, 0x0F, 0x3B, 0x0A, 0xC0 }; // 17
static CVI_U8 data_st7701_hd228001c31_alt0_35[] = { 0xE6, 0x00, 0x00, 0x22, 0x22 }; // 5
static CVI_U8 data_st7701_hd228001c31_alt0_36[] = { 0xE7, 0x44, 0x44 }; // 3
static CVI_U8 data_st7701_hd228001c31_alt0_37[] = { 0xE8, 0x10, 0x3C, 0x0A, 0xC0, 0x12, 0x3E, 0x0A,
       	0xC0, 0x0C, 0x38, 0x0A, 0xC0, 0x0E, 0x3A, 0x0A, 0xC0 }; // 17
static CVI_U8 data_st7701_hd228001c31_alt0_38[] = { 0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40 }; // 8

static CVI_U8 data_st7701_hd228001c31_alt0_39[] = { 0xED, 0x55, 0x44, 0x77, 0x66, 0xF0, 0xFC, 0x1A,
       	0x2B, 0xB2, 0xA1, 0xCF, 0x0F, 0x66, 0x77, 0x44, 0x55 }; //17

static CVI_U8 data_st7701_hd228001c31_alt0_40[] = { 0xEF, 0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F }; // 7
static CVI_U8 data_st7701_hd228001c31_alt0_41[] = { 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13 }; // 6

static CVI_U8 data_st7701_hd228001c31_alt0_42[] = { 0xE8, 0x00, 0x0E }; // 3

static CVI_U8 data_st7701_hd228001c31_alt0_43[] = { 0x11 }; // 1
//delay 120 ms
static CVI_U8 data_st7701_hd228001c31_alt0_44[] = { 0xE8, 0x00, 0x0c }; // 3
//delay 10 ms

static CVI_U8 data_st7701_hd228001c31_alt0_45[] = { 0xE8, 0x00, 0x00 }; // 3

static CVI_U8 data_st7701_hd228001c31_alt0_46[] = { 0xff, 0x77, 0x01, 0x00, 0x00, 0x00}; // 6

static CVI_U8 data_st7701_hd228001c31_alt0_47[] = { 0x29 }; // 1
//delay 50 ms


// len == 1 , type 0x05
// len == 2 , type 0x15 or type 23
// len >= 3 , type 0x29 or type 0x39
#define TYPE1_DCS_SHORT_WRITE 0x05
#define TYPE2_DCS_SHORT_WRITE 0x15
#define TYPE3_DCS_LONG_WRITE 0x39
#define TYPE3_GENERIC_LONG_WRITE 0x29
const struct dsc_instr dsi_init_cmds_st7701_368x552_alt0[] = {
{.delay = 0, .data_type = TYPE1_DCS_SHORT_WRITE,    .size = 1, .data = data_st7701_hd228001c31_alt0_0 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 6, .data = data_st7701_hd228001c31_alt0_1 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE,    .size = 2, .data = data_st7701_hd228001c31_alt0_2 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 6, .data = data_st7701_hd228001c31_alt0_3 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 3, .data = data_st7701_hd228001c31_alt0_4 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 3, .data = data_st7701_hd228001c31_alt0_5 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 3, .data = data_st7701_hd228001c31_alt0_6 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE,    .size = 2, .data = data_st7701_hd228001c31_alt0_7 },


// gamma cluster setting
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 17, .data = data_st7701_hd228001c31_alt0_8 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 17, .data = data_st7701_hd228001c31_alt0_9 },

// power control
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 6, .data = data_st7701_hd228001c31_alt0_10 },
// vcom setting
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_11 },
// vcom setting end

{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_11 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_12 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_13 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_14 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_15 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_16 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_17 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_18 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_19 },
{.delay = 100, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_20 },
// power control end

{.delay = 100, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 17, .data = data_st7701_hd228001c31_alt0_21 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_22 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_23 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_24 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_25 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_26 },
{.delay = 0, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_27 },
{.delay = 100, .data_type = TYPE2_DCS_SHORT_WRITE, .size = 2, .data = data_st7701_hd228001c31_alt0_28 },

{.delay = 100, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 4, .data = data_st7701_hd228001c31_alt0_29 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 12, .data = data_st7701_hd228001c31_alt0_30 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 13, .data = data_st7701_hd228001c31_alt0_31 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 5, .data = data_st7701_hd228001c31_alt0_32 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 3, .data = data_st7701_hd228001c31_alt0_33 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 17, .data = data_st7701_hd228001c31_alt0_34 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 5, .data = data_st7701_hd228001c31_alt0_35 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 6, .data = data_st7701_hd228001c31_alt0_36 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 17, .data = data_st7701_hd228001c31_alt0_37 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 8, .data = data_st7701_hd228001c31_alt0_38 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 17, .data = data_st7701_hd228001c31_alt0_39 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 7, .data = data_st7701_hd228001c31_alt0_40 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 6, .data = data_st7701_hd228001c31_alt0_41 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 3, .data = data_st7701_hd228001c31_alt0_42 },
{.delay = 120, .data_type = TYPE1_DCS_SHORT_WRITE, .size = 1, .data = data_st7701_hd228001c31_alt0_43 },
{.delay = 120, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 3, .data = data_st7701_hd228001c31_alt0_44 },
{.delay = 10, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 3, .data = data_st7701_hd228001c31_alt0_45 },
{.delay = 0, .data_type = TYPE3_GENERIC_LONG_WRITE, .size = 6, .data = data_st7701_hd228001c31_alt0_46 },
{.delay = 50, .data_type = TYPE1_DCS_SHORT_WRITE, .size = 1, .data = data_st7701_hd228001c31_alt0_47 },
};

#else
#error "MIPI_TX_PARAM multi-delcaration!!"
#endif // _MIPI_TX_PARAM_ST_7701_HD22801C31_H_
