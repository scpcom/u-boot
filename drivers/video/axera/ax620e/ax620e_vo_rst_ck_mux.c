/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#include <common.h>
#include <asm/io.h>

#include "ax_vo.h"

#define MM_CLK_MUX_0				0x0
#define MM_CLK_MUX_0_MM_GLB_SEL			21
#define MM_CLK_MUX_0_DPU_SRC_SEL		12
#define MM_CLK_MUX_0_DPU_SRC_SEL_MASK		0x7
#define MM_CLK_MUX_0_DPU_OUT_SEL		10
#define MM_CLK_MUX_0_DPU_OUT_SEL_MASK		0x3
#define MM_CLK_MUX_0_DPU_LITE_SRC_SEL		7
#define MM_CLK_MUX_0_DPU_LITE_SEL_MASK		0x7
#define MM_CLK_MUX_0_DPU_LITE_OUT_SEL		5
#define MM_CLK_MUX_0_DPU_LITE_OUT_SEL_MASK	0x3

#define MM_CLK_SEL_533M				0x5

#define MM_CLK_EB_0				0x4
#define MM_CLK_EB_0_DPU_OUT_EB			2
#define MM_CLK_EB_0_DPU_LITE_OUT_EB		1

#define MM_CLK_EB_1				0x8
#define MM_CLK_EB_1_PCLK_DPU_LITE_EB		21
#define MM_CLK_EB_1_PCLK_DPU_EB			20
#define MM_CLK_EB_1_PCLK_CMD_EB			19
#define MM_CLK_EB_1_CLK_DPU_LITE_EB		5
#define MM_CLK_EB_1_CLK_DPU_EB			4
#define MM_CLK_EB_1_CLK_CMD_EB			3

#define MM_CLK_DIV_0				0xC
#define MM_CLK_DIV_0_DPU_OUT_DIVN_UPDATE	9
#define MM_CLK_DIV_0_DPU_OUT_DIVN		5
#define MM_CLK_DIV_0_DPU_LITE_OUT_DIVN_UPDATE	4
#define MM_CLK_DIV_0_DPU_LITE_OUT_DIVN		0

#define MM_SW_RST_0				0x10
#define MM_SW_RST_0_DPU_SW_RST			12
#define MM_SW_RST_0_DPU_SW_PRST			11
#define MM_SW_RST_0_DPU_OUT_SW_PRST		10
#define MM_SW_RST_0_DPU_LITE_SW_RST		9
#define MM_SW_RST_0_DPU_LITE_SW_PRST		8
#define MM_SW_RST_0_DPU_LITE_OUT_SW_PRST	7
#define MM_SW_RST_0_CMD_SW_RST			5
#define MM_SW_RST_0_CMD_SW_PRST			4


#define MM_SET_OFFS(OFFS)			(((OFFS) >> 2) * 8 + 0xA4)
#define MM_CLR_OFFS(OFFS)			(((OFFS) >> 2) * 8 + 0xA8)

#define FLASH_CLK_MUX_0				0x0
#define FLASH_CLK_MUX_0_NX_VO1_SEL		14
#define FLASH_CLK_MUX_0_NX_VO1_SEL_MASK		0x3
#define FLASH_CLK_MUX_0_NX_VO0_SEL		12
#define FLASH_CLK_MUX_0_NX_VO0_SEL_MASK		0x3
#define FLASH_CLK_MUX_0_1X_VO1_SEL		2
#define FLASH_CLK_MUX_0_1X_VO1_SEL_MASK		0x3
#define FLASH_CLK_MUX_0_1X_VO0_SEL		0
#define FLASH_CLK_MUX_0_1X_VO0_SEL_MASK		0x3

#define FLASH_CLK_EB_0				0x4
#define FLASH_CLK_EB_0_NX_VO1_EB		8
#define FLASH_CLK_EB_0_NX_VO0_EB		7
#define FLASH_CLK_EB_0_1X_VO1_EB		1
#define FLASH_CLK_EB_0_1X_VO0_EB		0

#define FLASH_CLK_DIV_0				0xC
#define FLASH_CLK_DIV_0_NX_VO1_DIVN_UPDATE	19
#define FLASH_CLK_DIV_0_NX_VO1_DIVN		15
#define FLASH_CLK_DIV_0_NX_VO0_DIVN_UPDATE	14
#define FLASH_CLK_DIV_0_NX_VO0_DIVN		10
#define FLASH_CLK_DIV_0_1X_VO1_DIVN_UPDATE	9
#define FLASH_CLK_DIV_0_1X_VO1_DIVN		5
#define FLASH_CLK_DIV_0_1X_VO0_DIVN_UPDATE	4
#define FLASH_CLK_DIV_0_1X_VO0_DIVN		0

#define FLASH_SW_RST_0				0x14
#define FLASH_SW_RST_0_NX_VO1_SW_RST		29
#define FLASH_SW_RST_0_1X_VO1_SW_RST		28
#define FLASH_SW_RST_0_NX_VO0_SW_RST		27
#define FLASH_SW_RST_0_1X_VO0_SW_RST		26

#define FLASH_IMAGE_TX				0x1B8
#define FLASH_IMAGE_TX_CLKING_MODE		5
#define FLASH_IMAGE_TX_DLY_SEL			1
#define FLASH_IMAGE_TX_EN			0

#define FLASH_SET_OFFS(OFFS)			((OFFS) + 0x4000)
#define FLASH_CLR_OFFS(OFFS)			((OFFS) + 0x8000)


#define COMMON_CLK_MUX_1			0xC
#define COMMON_CLK_MUX_1_NX_VO1_SEL		22
#define COMMON_CLK_MUX_1_NX_VO1_SEL_MASK	0x3
#define COMMON_CLK_MUX_1_NX_VO0_SEL		20
#define COMMON_CLK_MUX_1_NX_VO0_SEL_MASK	0x3
#define COMMON_CLK_MUX_1_1X_VO1_SEL		14
#define COMMON_CLK_MUX_1_1X_VO1_SEL_MASK	0x3
#define COMMON_CLK_MUX_1_1X_VO0_SEL		12
#define COMMON_CLK_MUX_1_1X_VO0_SEL_MASK	0x3

#define COMMON_CLK_EB_0				0x24
#define COMMON_CLK_EB_0_NX_VO1_EB		13
#define COMMON_CLK_EB_0_NX_VO0_EB		12
#define COMMON_CLK_EB_0_1X_VO1_EB		7
#define COMMON_CLK_EB_0_1X_VO0_EB		6

#define COMMON_CLK_DIV_1			0x48
#define COMMON_CLK_DIV_1_NX_VO1_DIVN_UPDATE	19
#define COMMON_CLK_DIV_1_NX_VO1_DIVN		15
#define COMMON_CLK_DIV_1_NX_VO0_DIVN_UPDATE	14
#define COMMON_CLK_DIV_1_NX_VO0_DIVN		10
#define COMMON_CLK_DIV_1_1X_VO1_DIVN_UPDATE	9
#define COMMON_CLK_DIV_1_1X_VO1_DIVN		5
#define COMMON_CLK_DIV_1_1X_VO0_DIVN_UPDATE	4
#define COMMON_CLK_DIV_1_1X_VO0_DIVN		0

#define COMMON_SW_RST_0				0x54
#define COMMON_SW_RST_0_NX_VO1_SW_RST		29
#define COMMON_SW_RST_0_1X_VO1_SW_RST		28
#define COMMON_SW_RST_0_NX_VO0_SW_RST		27
#define COMMON_SW_RST_0_1X_VO0_SW_RST		26

#define COMMON_VO_CFG				0x424
#define COMMON_DPU_LITE_TX_DLY_SEL		5
#define COMMON_DPU_LITE_TX_CLKING_MODE		3
#define COMMON_DPU_LITE_DPHY_TX_EN		2
#define COMMON_DPU_LITE_DMUX_SEL		1
#define COMMON_LCD_VO_MUX_SEL			0

#define COMMON_SET_OFFS(OFFS)			((OFFS) + 0x4)
#define COMMON_CLR_OFFS(OFFS)			((OFFS) + 0x8)

#define DISPLAY_CLK_MUX_0			0x0
#define DISPLAY_CLK_EB_0			0x4
#define DISPLAY_CLK_EB_1			0x8
#define DISPLAY_SW_RST_0			0xC
#define DISPLAY_DSI				0x14
#define DISPLAY_DSI_SDI_CLK_SEL			0x48
#define DISPLAY_LVDS_CLK_SEL			0x4C

#define DISPLAY_SET_OFFS(OFFS)			0xA0
#define DISPLAY_CLR_OFFS(OFFS)			0xA4

#define VPLL_297 297000000
#define VPLL_198 198000000
#define VPLL_118P8 118800000
#define VPLL_108 108000000

#define SRC_CLK_NUM 4
#define CLK_DIV_NUM 16

static void __iomem *mm_sys_glb_regs = (void __iomem *)MM_SYS_GLB_BASE_ADDR;
static void __iomem *common_sys_glb_regs = (void __iomem *)COMMON_SYS_GLB_BASE_ADDR;
static void __iomem *flash_sys_glb_regs = (void __iomem *)FLASH_SYS_GLB_BASE_ADDR;
static void __iomem *display_sys_glb_regs = (void __iomem *)DISPLAY_SYS_GLB_BASE_ADDR;

static int supported_dpi_clk[SRC_CLK_NUM][CLK_DIV_NUM] = {0};
static int pixel_clk_sel(int clk, int *sel, int *div)
{
	int i, j;
	int diff = -1, min = VPLL_108 / CLK_DIV_NUM;

	*sel = -1;
	*div = -1;

	for (i = 0; i < SRC_CLK_NUM; i++) {
		for (j = 0; j < CLK_DIV_NUM; j++) {
			if (clk >= supported_dpi_clk[i][j]) {
				diff = clk - supported_dpi_clk[i][j];
				if (!diff) {
					*sel = i;
					*div = j;
					goto exit;
				}

				if (diff < min) {
					*sel = i;
					*div = j;
					min = diff;
				}
			}
		}
	}

exit:
	if (*sel < 0) {
		VO_ERROR("unsupported frequency point, clk = %d\n", clk);
		return -EINVAL;
	}

	if (*sel > 2)
		*sel += 1;

	VO_DEBUG("expected clk: %d, actual clk: %d, sel: %d, div: %d\n",
	         clk, supported_dpi_clk[i][j], i, j);

	return 0;
}

int pixel_clk_set_rate(u32 id, int clk)
{
	int ret;
	int sel = 0,div = 0;
	u32 val;
	ret = pixel_clk_sel(clk,&sel,&div);
	if (ret) {
		VO_ERROR("set clk fail\n");
		return ret;
	}

	val = (id == 0) ? (sel << MM_CLK_MUX_0_DPU_OUT_SEL) : (sel << MM_CLK_MUX_0_DPU_LITE_SRC_SEL);
	dpu_writel(mm_sys_glb_regs, MM_SET_OFFS(MM_CLK_MUX_0), val);

	val = ((id == 0) ? (div << MM_CLK_DIV_0_DPU_OUT_DIVN) : (div << MM_CLK_DIV_0_DPU_LITE_OUT_DIVN)) |
	      ((id == 0) ? (1 << MM_CLK_DIV_0_DPU_OUT_DIVN_UPDATE) : (1 << MM_CLK_DIV_0_DPU_LITE_OUT_DIVN_UPDATE));
	dpu_writel(mm_sys_glb_regs, MM_SET_OFFS(MM_CLK_DIV_0), val);
	dpu_writel(mm_sys_glb_regs, MM_CLR_OFFS(MM_CLK_DIV_0), (id == 0) ? (1 << MM_CLK_DIV_0_DPU_OUT_DIVN_UPDATE) : (1 << MM_CLK_DIV_0_DPU_LITE_OUT_DIVN_UPDATE));

	val = ((id == 0) ? (sel << COMMON_CLK_MUX_1_1X_VO0_SEL) : (sel << COMMON_CLK_MUX_1_1X_VO1_SEL)) |
	      ((id == 0) ? (sel << COMMON_CLK_MUX_1_NX_VO0_SEL) : (div << COMMON_CLK_MUX_1_NX_VO1_SEL));
	dpu_writel(common_sys_glb_regs, COMMON_SET_OFFS(COMMON_CLK_MUX_1), (sel << MM_CLK_MUX_0_DPU_OUT_SEL));

	val = ((id == 0) ? (div << COMMON_CLK_DIV_1_NX_VO0_DIVN) : (div << COMMON_CLK_DIV_1_NX_VO1_DIVN)) |
	      ((id == 0) ? (div << COMMON_CLK_DIV_1_1X_VO0_DIVN) : (div << COMMON_CLK_DIV_1_1X_VO1_DIVN)) |
	      ((id == 0) ? (1 << COMMON_CLK_DIV_1_1X_VO0_DIVN_UPDATE) : (1 << COMMON_CLK_DIV_1_1X_VO1_DIVN_UPDATE)) |
	      ((id == 0) ? (1 << COMMON_CLK_DIV_1_NX_VO0_DIVN_UPDATE) : (1 << COMMON_CLK_DIV_1_NX_VO1_DIVN_UPDATE));
	dpu_writel(common_sys_glb_regs, COMMON_SET_OFFS(COMMON_CLK_DIV_1), val);

	val = ((id == 0) ? (1 << COMMON_CLK_DIV_1_1X_VO0_DIVN_UPDATE) : (1 << COMMON_CLK_DIV_1_1X_VO1_DIVN_UPDATE)) |
	      ((id == 0) ? (1 << COMMON_CLK_DIV_1_NX_VO0_DIVN_UPDATE) : (1 << COMMON_CLK_DIV_1_NX_VO1_DIVN_UPDATE));
	dpu_writel(common_sys_glb_regs, COMMON_CLR_OFFS(COMMON_CLK_DIV_1), val);

	val = (sel << FLASH_CLK_MUX_0_1X_VO0_SEL) | (sel << FLASH_CLK_MUX_0_NX_VO0_SEL);
	dpu_writel(flash_sys_glb_regs, FLASH_SET_OFFS(FLASH_CLK_MUX_0), val);
	val = (div << FLASH_CLK_DIV_0_1X_VO0_DIVN) | (div << FLASH_CLK_DIV_0_NX_VO0_DIVN) |
	      (1 << FLASH_CLK_DIV_0_1X_VO0_DIVN_UPDATE) | (1 << FLASH_CLK_DIV_0_NX_VO0_DIVN_UPDATE) ;
	dpu_writel(flash_sys_glb_regs, FLASH_SET_OFFS(FLASH_CLK_DIV_0), val);
	dpu_writel(flash_sys_glb_regs, FLASH_CLR_OFFS(FLASH_CLK_DIV_0),  (1 << FLASH_CLK_DIV_0_1X_VO0_DIVN_UPDATE) | (1 << FLASH_CLK_DIV_0_NX_VO0_DIVN_UPDATE));

	return 0;
}

int pixel_clk_get_rate(u32 id)
{
	return 0;
}

static void pixel_clk_init(void)
{
	int i, j;
	int src[SRC_CLK_NUM] = {VPLL_108, VPLL_118P8, VPLL_198, VPLL_297};

	for (i = 0; i < SRC_CLK_NUM; i++) {
		for (j = 0; j < CLK_DIV_NUM; j++) {
			if (j == 0)
				supported_dpi_clk[i][j] = src[i];
			else
				supported_dpi_clk[i][j] = src[i] / (j * 2);

			VO_DEBUG("supported_dpi_clk[%d][%d] = %d\n", i, j, supported_dpi_clk[i][j]);
		}
	}
}

int dpu_glb_path_config(u32 id, u32 out_mode)
{
	VO_DEBUG("dpu%d out mode(%d)\n", id, out_mode);

	switch (out_mode) {
	case OUT_MODE_BT601:
	case OUT_MODE_BT656:
	case OUT_MODE_BT1120:
	case OUT_MODE_DPI:
		if (id == 0) {
			dpu_writel(common_sys_glb_regs, COMMON_CLR_OFFS(COMMON_VO_CFG), (1 << COMMON_LCD_VO_MUX_SEL));
		} else {
			dpu_writel(common_sys_glb_regs, COMMON_SET_OFFS(COMMON_VO_CFG), (1 << COMMON_LCD_VO_MUX_SEL));
			dpu_writel(common_sys_glb_regs, COMMON_SET_OFFS(COMMON_VO_CFG), (1 << COMMON_DPU_LITE_DMUX_SEL));
		}

		dpu_writel(display_sys_glb_regs, DISPLAY_LVDS_CLK_SEL, 0x1);

		dpu_writel(flash_sys_glb_regs, FLASH_CLR_OFFS(FLASH_IMAGE_TX), (0x3 << (FLASH_IMAGE_TX_CLKING_MODE)));
		dpu_writel(flash_sys_glb_regs, FLASH_SET_OFFS(FLASH_IMAGE_TX), (0x1 << FLASH_IMAGE_TX_EN));

		break;

	case OUT_MODE_DSI_DPI_VIDEO:
	case OUT_MODE_DSI_SDI_VIDEO:
	case OUT_MODE_DSI_SDI_CMD:
		break;
	case OUT_MODE_LVDS:
		break;
	default:
		VO_ERROR("unsupported dpu%d out mode(%d)\n", id, out_mode);
		return -EINVAL;
	}

	return 0;
}

void dpu_glb_init(u32 id)
{
	u32 val;

	pixel_clk_init();

	/* mm sys glb */
	val = ((id == 0) ? (1 << MM_CLK_EB_0_DPU_OUT_EB) : (1 << MM_CLK_EB_0_DPU_LITE_OUT_EB));
	dpu_writel(mm_sys_glb_regs, MM_SET_OFFS(MM_CLK_EB_0), val);

	val = (1 << MM_CLK_EB_1_CLK_CMD_EB) | (1 << MM_CLK_EB_1_PCLK_CMD_EB) |
	      ((id == 0) ? (1 << MM_CLK_EB_1_CLK_DPU_EB) : (1 << MM_CLK_EB_1_CLK_DPU_LITE_EB)) |
	      ((id == 0) ? (1 << MM_CLK_EB_1_PCLK_DPU_EB) : (1 << MM_CLK_EB_1_PCLK_DPU_LITE_EB));
	dpu_writel(mm_sys_glb_regs, MM_SET_OFFS(MM_CLK_EB_1), val);

	val = (1 << MM_SW_RST_0_CMD_SW_PRST) | (1 << MM_SW_RST_0_CMD_SW_RST) |
	      ((id == 0) ? (1 << MM_SW_RST_0_DPU_OUT_SW_PRST) : (1 << MM_SW_RST_0_DPU_LITE_OUT_SW_PRST)) |
	      ((id == 0) ? (1 << MM_SW_RST_0_DPU_SW_PRST) : (1 << MM_SW_RST_0_DPU_LITE_SW_PRST)) |
	      ((id == 0) ? (1 << MM_SW_RST_0_DPU_SW_RST) : (1 << MM_SW_RST_0_DPU_LITE_SW_RST));
	dpu_writel(mm_sys_glb_regs, MM_SET_OFFS(MM_SW_RST_0), val);
	udelay(50);
	dpu_writel(mm_sys_glb_regs, MM_CLR_OFFS(MM_SW_RST_0), val);
	dpu_writel(mm_sys_glb_regs, MM_SET_OFFS(MM_CLK_MUX_0), ((MM_CLK_SEL_533M << MM_CLK_MUX_0_MM_GLB_SEL) | (MM_CLK_SEL_533M <<MM_CLK_MUX_0_DPU_SRC_SEL)));

	/* common sys glb */
	val = ((id == 0) ? (1 << COMMON_CLK_EB_0_1X_VO0_EB) : (1 << COMMON_CLK_EB_0_1X_VO1_EB)) |
	      ((id == 0) ? (1 << COMMON_CLK_EB_0_NX_VO0_EB) : (1 << COMMON_CLK_EB_0_NX_VO1_EB));
	dpu_writel(common_sys_glb_regs, COMMON_SET_OFFS(COMMON_CLK_EB_0), val);

	val = ((id == 0) ? (1 << COMMON_SW_RST_0_1X_VO0_SW_RST) : (1 << COMMON_SW_RST_0_1X_VO1_SW_RST)) |
	      ((id == 0) ? (1 << COMMON_SW_RST_0_NX_VO0_SW_RST) : (1 << COMMON_SW_RST_0_NX_VO1_SW_RST));
	dpu_writel(common_sys_glb_regs, COMMON_SET_OFFS(COMMON_SW_RST_0), val);
	udelay(50);
	dpu_writel(common_sys_glb_regs, COMMON_CLR_OFFS(COMMON_SW_RST_0), val);

	/* flash sys glb */
	val = ((id == 0) ? (1 << FLASH_CLK_EB_0_1X_VO0_EB) : (1 << FLASH_CLK_EB_0_1X_VO1_EB)) |
	      ((id == 0) ? (1 << FLASH_CLK_EB_0_NX_VO0_EB) : (1 << FLASH_CLK_EB_0_NX_VO1_EB));
	dpu_writel(flash_sys_glb_regs, FLASH_SET_OFFS(FLASH_CLK_EB_0), val);

	val = ((id == 0) ? (1 << FLASH_SW_RST_0_1X_VO0_SW_RST) : (1 << FLASH_SW_RST_0_1X_VO1_SW_RST)) |
	      ((id == 0) ? (1 << FLASH_SW_RST_0_NX_VO0_SW_RST) : (1 << FLASH_SW_RST_0_NX_VO1_SW_RST));
	dpu_writel(flash_sys_glb_regs, FLASH_SET_OFFS(FLASH_SW_RST_0), val);
	udelay(50);
	dpu_writel(flash_sys_glb_regs, FLASH_CLR_OFFS(FLASH_SW_RST_0), val);
}

