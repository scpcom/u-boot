/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#include <linux/dma-mapping.h>
#include <common.h>

#include "ax620e_vo_reg.h"
#include "ax620e_vo_rst_ck_mux.h"
#include "ax_vo.h"

static void dpu_intr_mask_all(struct dpu_hw_device *hdev)
{
	dpu_writel(hdev->regs, DPU_INT_MASK0, 0x7);
	dpu_writel(hdev->regs, DPU_INT_MASK1, 0xF);
}

static void dpu_intr_sts_clr_all(struct dpu_hw_device *hdev)
{
	dpu_writel(hdev->regs, DPU_INT_CLR, 0x7F);
}

static void dpu_set_mode(struct dpu_hw_device *hdev)
{
	dpu_writel(hdev->regs, DPU_TOP_CTRL, hdev->is_online ? 0x0 : 0x1);
}

void dispc_update_lock(struct dpu_hw_device *hdev)
{
	dpu_writel(hdev->regs, DPU_DISP_UP, 0);
}

void dispc_update_unlock(struct dpu_hw_device *hdev)
{
	dpu_writel(hdev->regs, DPU_DISP_UP, 1);
}

static void dispc_enable(struct dpu_hw_device *hdev)
{
	dpu_writel(hdev->regs, DPU_DISP_EN, 0x1);
}

static void dispc_disable(struct dpu_hw_device *hdev)
{
	dpu_writel(hdev->regs, DPU_DISP_EN, 0x0);
}

static void dispc_set_out_mode(struct dpu_hw_device *hdev, u32 mode)
{
	dpu_writel(hdev->regs, DPU_OUT_MODE, mode);
}

static void dispc_set_format_in(struct dpu_hw_device *hdev, u32 fmt)
{
	dpu_writel(hdev->regs, DPU_RD_FORMAT, fmt);
}

static void dispc_set_format_out(struct dpu_hw_device *hdev, u32 fmt)
{
	dpu_writel(hdev->regs, DPU_DISP_FORMAT, fmt);
}

static void dispc_set_bt_mode(struct dpu_hw_device *hdev, u32 mode)
{
	dpu_writel(hdev->regs, DPU_BT_MODE, mode);
}

static void dispc_set_disp_reso(struct dpu_hw_device *hdev, u32 reso)
{
	dpu_writel(hdev->regs, DPU_DISP_RESO, reso);
}

static void dispc_set_timings(struct dpu_hw_device *hdev, struct ax_disp_mode *mode)
{
	u32 hp_pol, vp_pol, de_pol;
	u32 hbp, hfp, vbp, vfp, hpw, vpw;
	u32 hhalf = 0;

	hp_pol = mode->hp_pol;
	vp_pol = mode->vp_pol;
	de_pol = mode->de_pol;

	VO_INFO("hp_pol = %d, vp_pol = %d, de_pol = %d, out_mode:%d\n", hp_pol, vp_pol, de_pol, hdev->out_mode.mode);

	hpw = mode->hsync_end - mode->hsync_start;
	vpw = mode->vsync_end - mode->vsync_start;

	hbp = mode->htotal - mode->hsync_end;
	hfp = mode->hsync_start - mode->hdisplay;

	vbp = mode->vtotal - mode->vsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	if (mode->flags & MODE_FLAG_INTERLACE) {
		vbp /= 2;
		vfp /= 2;
	}

	hhalf = mode->htotal / 2;

	if (hdev->out_mode.mode == OUT_MODE_BT1120) {
		hpw = mode->htotal - mode->hdisplay - 4;
		vpw = mode->vtotal - mode->vsync_start;
	} else if (hdev->out_mode.mode == OUT_MODE_BT656) {
		hpw = mode->htotal - mode->hdisplay - 2;
		vpw = mode->vtotal - mode->vsync_start;
	}
	if (mode->flags & MODE_FLAG_INTERLACE)
		vpw /= 2;

	VO_INFO("hbp = %d, hfp = %d, hpw = %d\n", hbp, hfp, hpw);
	VO_INFO("vbp = %d, vfp = %d, vpw = %d\n", vbp, vfp, vpw);

	dpu_writel(hdev->regs, DPU_DISP_SYNC, (hpw << 16) | vpw);
	dpu_writel(hdev->regs, DPU_DISP_HSYNC, (hbp << 16) | hfp);
	dpu_writel(hdev->regs, DPU_DISP_VSYNC, (vbp << 16) | vfp);
	dpu_writel(hdev->regs, DPU_DISP_HHALF, hhalf);

	if (mode->flags & MODE_FLAG_INTERLACE)
		dpu_writel(hdev->regs, DPU_DISP_VTOTAL, !(mode->vtotal & 1));

	if (mode->type == AX_DISP_OUT_MODE_DSI_DPI_VIDEO || mode->type == AX_DISP_OUT_MODE_DSI_SDI_VIDEO) {
		hp_pol = !hp_pol;
		vp_pol = !vp_pol;
	}

	dpu_writel(hdev->regs, DPU_DISP_POLAR, de_pol << 2 | hp_pol << 1 | vp_pol);
}


static void dispc_set_yuv2rgb_matrix(struct dpu_hw_device *hdev, const struct yuv2rgb_regs *m)
{
	int i, j;
	u32 reg_offs = DPU_DISP_2RGB_MATRIX_00;
	enum dispc_format_out fmt_out = hdev->out_mode.fmt_out;

	/* yuv2rgb config */
	if (fmt_out != FMT_OUT_YUV422) {
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				dpu_writel(hdev->regs, reg_offs, m->matrix[i][j]);
				reg_offs += 4;
			}
		}

		for (i = 0; i < 2; i++) {
			for (j = 0; j < 3; j++) {
				dpu_writel(hdev->regs, reg_offs, m->offset[i][j]);
				reg_offs += 4;
			}
		}

		dpu_writel(hdev->regs, DPU_DISP_2RGB_CTRL, 0x3);

	} else {
		dpu_writel(hdev->regs, DPU_DISP_2RGB_CTRL, 0x2);
	}
}

static void dispc_set_rgb2yuv_matrix(struct dpu_hw_device *hdev, const struct rgb2yuv_regs *m) {
	int i, j;
	u32 reg_offs = DPU_RD_2YUV_MATRIX_00;
	enum dispc_format_in fmt_in = hdev->out_mode.fmt_in;

	/* rgb to yuv422 */
	if (fmt_in <= FMT_IN_RGB888) {
		dpu_writel(hdev->regs, DPU_RD_2YUV_EN, 0x1);
		dpu_writel(hdev->regs, DPU_RD_2YUV_CTRL, 0x11);

		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				dpu_writel(hdev->regs, reg_offs, m->matrix[i][j]);
				reg_offs += 4;
			}
		}

		dpu_writel(hdev->regs, DPU_RD_2YUV_OFFSET_00, m->offset[0][0]);
		dpu_writel(hdev->regs, DPU_RD_2YUV_OFFSET_01, m->offset[0][1]);
		dpu_writel(hdev->regs, DPU_RD_2YUV_OFFSET_02, m->offset[0][2]);
		dpu_writel(hdev->regs, DPU_RD_2YUV_OFFSET_10, m->offset[1][0]);
		dpu_writel(hdev->regs, DPU_RD_2YUV_OFFSET_11, m->offset[1][1]);
		dpu_writel(hdev->regs, DPU_RD_2YUV_OFFSET_12, m->offset[1][2]);
		dpu_writel(hdev->regs, DPU_RD_DECIMATION_H0, m->decimation_h[0]);
		dpu_writel(hdev->regs, DPU_RD_DECIMATION_H1, m->decimation_h[1]);
		dpu_writel(hdev->regs, DPU_RD_DECIMATION_H2, m->decimation_h[2]);
		dpu_writel(hdev->regs, DPU_RD_DECIMATION_H3, m->decimation_h[3]);
		dpu_writel(hdev->regs, DPU_RD_DECIMATION_H4, m->decimation_h[4]);
		dpu_writel(hdev->regs, DPU_RD_DECIMATION_H5, m->decimation_h[5]);
		dpu_writel(hdev->regs, DPU_RD_DECIMATION_H6, m->decimation_h[6]);

	} else {
		dpu_writel(hdev->regs, DPU_RD_2YUV_EN, 0x0);
		dpu_writel(hdev->regs, DPU_RD_2YUV_CTRL, 0x10);
	}
}

static void draw_disable(struct dpu_hw_device *hdev)
{
	dpu_writel(hdev->regs, DPU_DRAW_EN, 0x0);
}

static bool src_format_valid(u32 vo_fmt)
{
	switch (vo_fmt) {
	case AX_VO_FORMAT_NV12:
	case AX_VO_FORMAT_NV21:
	case AX_VO_FORMAT_ARGB1555:
	case AX_VO_FORMAT_ARGB4444:
	case AX_VO_FORMAT_RGBA5658:
	case AX_VO_FORMAT_ARGB8888:
	case AX_VO_FORMAT_RGB565:
	case AX_VO_FORMAT_RGB888:
	case AX_VO_FORMAT_RGBA4444:
	case AX_VO_FORMAT_RGBA5551:
	case AX_VO_FORMAT_RGBA8888:
	case AX_VO_FORMAT_ARGB8565:
	case AX_VO_FORMAT_P010:
	case AX_VO_FORMAT_P016:
	case AX_VO_FORMAT_NV16:
	case AX_VO_FORMAT_P210:
	case AX_VO_FORMAT_P216:
	case AX_VO_FORMAT_BITMAP:
		return true;
	}

	return false;
}

static void fmt2bytes_per_pixel(u32 vo_fmt, u32 *bytes_per_pixel, u32 *nplanes)
{
	switch (vo_fmt) {
	case AX_VO_FORMAT_NV12:
	case AX_VO_FORMAT_NV21:
	case AX_VO_FORMAT_NV16:
		*bytes_per_pixel = 1;
		*nplanes = 2;
		break;
	case AX_VO_FORMAT_ARGB1555:
	case AX_VO_FORMAT_ARGB4444:
	case AX_VO_FORMAT_RGBA4444:
	case AX_VO_FORMAT_RGBA5551:
	case AX_VO_FORMAT_RGB565:
		*bytes_per_pixel = 2;
		*nplanes = 1;
		break;
	case AX_VO_FORMAT_RGBA5658:
	case AX_VO_FORMAT_ARGB8565:
	case AX_VO_FORMAT_RGB888:
		*bytes_per_pixel = 3;
		*nplanes = 1;
		break;
	case AX_VO_FORMAT_ARGB8888:
	case AX_VO_FORMAT_RGBA8888:
		*bytes_per_pixel = 4;
		*nplanes = 1;
		break;
	case AX_VO_FORMAT_P010:
	case AX_VO_FORMAT_P016:
	case AX_VO_FORMAT_P210:
	case AX_VO_FORMAT_P216:
		*bytes_per_pixel = 2;
		*nplanes = 2;
		break;
	case AX_VO_FORMAT_BITMAP:
		*bytes_per_pixel = 1;
		*nplanes = 1;
		break;
	}
}

static int geometry_valid(struct draw_task *task)
{
	u16 x, y, w, h, stride_y, stride_c;
	u32 supported_width_max, supported_height_max;
	u32 bytes_per_pixel = 1, nplanes = 1;
	u32 format;

	supported_width_max = task->dst_w;
	supported_height_max = task->dst_h;

	x = task->dst_x;
	y = task->dst_y;
	w = task->src_w;
	h = task->src_h;
	stride_y = task->src_stride_y;
	stride_c = task->src_stride_c;

	format = task->src_fmt;

	if (!src_format_valid(format)) {
		VO_ERROR("draw task src format(%d) invalid\n", format);
		return -EINVAL;
	}

	fmt2bytes_per_pixel(format, &bytes_per_pixel, &nplanes);

	if (w < 2) {
		VO_ERROR("draw task src width(%d) invalid\n", w);
		return -EINVAL;
	}

	if (h < 2) {
		VO_ERROR("draw task src height(%d) invalid\n", h);
		return -EINVAL;
	}

	if (x + w > supported_width_max) {
		w = supported_width_max - x;
		task->src_w = w;
	}

	if (y + h > supported_height_max) {
		h = supported_height_max - y;
		task->src_h = h;
	}

	if ((stride_y < w * bytes_per_pixel) || (stride_y >= DRAW_STRIDE_MAX) ||
	    (stride_y & (DRAW_ALIGNED_BYTES - 1))) {
		VO_ERROR("draw task src stride_y(%d) invalid\n", stride_y);
		return -EINVAL;
	}

	if ((nplanes > 1) && ((stride_c < w * bytes_per_pixel) || (stride_c >= DRAW_STRIDE_MAX) ||
			       (stride_c & (DRAW_ALIGNED_BYTES - 1)))) {
		VO_ERROR("draw task src stride_c(%d) invalid\n", stride_c);
		return -EINVAL;
	}

	if (x >= supported_width_max) {
		VO_ERROR("draw task dst_x(%d) invalid\n", x);
		return -EINVAL;
	}

	if (y >= supported_height_max) {
		VO_ERROR("draw task dst_y(%d) invalid\n", y);
		return -EINVAL;
	}

	return 0;
}

static int ax620e_draw_task_valid(struct draw_task *task)
{
	int ret;
	u32 stride, bytes_per_pixel = 1, nplanes = 1;

	if (!task->data) {
		VO_ERROR("draw task data-field invalid\n");
		return -EINVAL;
	}

	/* The layer output format check */
	if (task->dst_fmt > AX_VO_FORMAT_P010) {
		VO_ERROR("draw task dst_fmt(%d) invalid\n", task->dst_fmt);
		return -EINVAL;
	}

	fmt2bytes_per_pixel(task->dst_fmt, &bytes_per_pixel, &nplanes);

	if ((task->dst_w < DRAW_WIDTH_MIN) || (task->dst_w > DRAW_WIDTH_MAX) || (task->dst_w & 0x1)) {
		VO_ERROR("draw task dst_w(%d) invalid\n", task->dst_w);
		return -EINVAL;
	}

	if ((task->dst_h < DRAW_HEIGHT_MIN) || (task->dst_h > DRAW_WIDTH_MAX) || (task->dst_h & 0x1)) {
		VO_ERROR("draw task dst_h(%d) invalid\n", task->dst_h);
		return -EINVAL;
	}

	stride = task->dst_stride_y;
	if ((stride % bytes_per_pixel) || ((stride / bytes_per_pixel) < task->dst_w) ||
	    (stride >= DRAW_STRIDE_MAX) || (stride & (DRAW_ALIGNED_BYTES - 1))) {
		VO_ERROR("draw task dst stride_y(%d) invalid\n", stride);
		return -EINVAL;
	}

	stride = task->dst_stride_c;
	if (stride && ((stride % bytes_per_pixel) || ((stride / bytes_per_pixel) < task->dst_w) ||
	    (stride >= DRAW_STRIDE_MAX) || (stride & (DRAW_ALIGNED_BYTES - 1)))) {
		VO_ERROR("draw task dst stride_c(%d) invalid\n", stride);
		return -EINVAL;
	}

	ret = geometry_valid(task);
	if (ret)
		return ret;

	return 0;
}

static const struct color_space_cfg cs_cfgs = {
	.yuv2rgb_cfg = { \
		.matrix = { \
				{0x100, 0x0, 0x167}, \
				{0x100, 0x7a8, 0x749}, \
				{0x100, 0x1c6, 0x0} \
		}, \
		.offset = { \
				{0x0, 0x0, 0x0}, \
				{0x0, 0x0, 0x0}  \
		},\
	},
	.rgb2yuv_cfg = { \
		.matrix = { \
				{0x4d, 0x96, 0x1d}, \
				{0x7d5, 0x7ab, 0x80}, \
				{0x80, 0x795, 0x7eb} \
		}, \
		.offset = { \
				{0x0, 0x0, 0x0}, \
				{0x40, 0x0, 0x0} \
		}, \
		.decimation_h = {0x0, 0x0, 0x0, 0x10, 0x10, 0x0, 0x0}, \
		.uv_offbin_en = 0, \
		.uv_seq_sel = 0, \
	},
};

static int draw_color_space_cfg(struct draw_task *task)
{
	int i, j, k;
	u32 reg_offs;
	const struct color_space_cfg *cs_cfg;
	const struct yuv2rgb_regs *yuv2rgb_cfg;
	const struct rgb2yuv_regs *rgb2yuv_cfg;
	struct dpu_hw_device *hdev = (struct dpu_hw_device *)task->data;

	cs_cfg = &cs_cfgs;
	if (!cs_cfg)
		return -EINVAL;

	rgb2yuv_cfg = &cs_cfg->rgb2yuv_cfg;

	/* rgb2yuv config */
	for (k = 0; k < 2; k++) {
		if (k == 0)
			reg_offs = DPU_V0_2YUV_MATRIX_00;
		else
			reg_offs = DPU_G0_2YUV_MATRIX_00;

		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				dpu_writel(hdev->regs, reg_offs,rgb2yuv_cfg->matrix[i][j]);
				reg_offs += 4;
			}
		}

		for (i = 0; i < 2; i++) {
			for (j = 0; j < 3; j++) {
				dpu_writel(hdev->regs, reg_offs, rgb2yuv_cfg->offset[i][j]);
				reg_offs += 4;
			}
		}

		for (i = 0; i < 7; i++) {
			dpu_writel(hdev->regs, reg_offs, rgb2yuv_cfg->decimation_h[i]);
			reg_offs += 4;
		}

	}

	/* yuv2rgb config */
	reg_offs = DPU_WR_2RGB_MATRIX_00;
	yuv2rgb_cfg = &cs_cfg->yuv2rgb_cfg;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			dpu_writel(hdev->regs, reg_offs, yuv2rgb_cfg->matrix[i][j]);
			reg_offs += 4;
		}
	}

	for (i = 0; i < 2; i++) {
		for (j = 0; j < 3; j++) {
			dpu_writel(hdev->regs, reg_offs, rgb2yuv_cfg->offset[i][j]);
			reg_offs += 4;
		}
	}

	return 0;
}

static void draw_in_cfg(struct draw_task *task)
{
	u32 fmt = vo_fmt2hw_fmt(task->src_fmt);
	struct dpu_hw_device *hdev = (struct dpu_hw_device *)task->data;

	dpu_writel(hdev->regs, DPU_V0_RESO, (task->src_h << 16) | task->src_w);
	dpu_writel(hdev->regs, DPU_V0_FORMAT, fmt);
	dpu_writel(hdev->regs, DPU_V0_COODI, (task->dst_y << 16) | task->dst_x);
	dpu_writel(hdev->regs, DPU_V0_ADDR_Y_L, PYHS_ADDR_LOW(task->src_phy_addr_y));
	dpu_writel(hdev->regs, DPU_V0_ADDR_Y_H, PYHS_ADDR_HIGH(task->src_phy_addr_y));
	dpu_writel(hdev->regs, DPU_V0_STRIDE_Y, task->src_stride_y);
	dpu_writel(hdev->regs, DPU_V0_ADDR_C_H, PYHS_ADDR_HIGH(task->src_phy_addr_c));
	dpu_writel(hdev->regs, DPU_V0_ADDR_C_L, PYHS_ADDR_LOW(task->src_phy_addr_c));
	dpu_writel(hdev->regs, DPU_V0_STRIDE_C, task->src_stride_c);

	dpu_writel(hdev->regs, DPU_V0_2YUV_EN, (fmt <= FORMAT_RGB888) ? 0x1 : 0x0);
	dpu_writel(hdev->regs, DPU_V0_2YUV_CTRL, (fmt <= FORMAT_RGB888) ? 0x11 : 0x0);

	dpu_writel(hdev->regs, DPU_V0_BACK_PIXEL, (0x80 << 20) | (0x80 << 10));
	dpu_writel(hdev->regs, DPU_V0_BACK_ALPHA, 0xFF);

	dpu_writel(hdev->regs, DPU_V0_FBDC_EN, 0);
}

static void draw_out_cfg(struct draw_task *task)
{
	u32 fmt = vo_fmt2hw_fmt(task->dst_fmt);
	struct dpu_hw_device *hdev = (struct dpu_hw_device *)task->data;

	dpu_writel(hdev->regs, DPU_WR_2YUV_EN, (fmt <= FORMAT_RGB888) ? 0x0 : 0x1);
	dpu_writel(hdev->regs, DPU_WR_2RGB_CTRL, (fmt <= FORMAT_RGB888) ? 0x3 : 0x2);
	dpu_writel(hdev->regs, DPU_WR_DECIMATION_V0, (fmt <= FORMAT_RGB888) ? 0x0 : 0x1);
	dpu_writel(hdev->regs, DPU_WR_DECIMATION_V1, (fmt <= FORMAT_RGB888) ? 0x0 : 0x1);

	dpu_writel(hdev->regs, DPU_DRAW_RESO, (task->dst_h << 16) | task->dst_w);
	dpu_writel(hdev->regs, DPU_WR_FORMAT, fmt);

	dpu_writel(hdev->regs, DPU_WR_ADDR_Y_H, PYHS_ADDR_HIGH(task->dst_phy_addr_y));
	dpu_writel(hdev->regs, DPU_WR_ADDR_Y_L, PYHS_ADDR_LOW(task->dst_phy_addr_y));
	dpu_writel(hdev->regs, DPU_WR_STRIDE_Y, task->dst_stride_y);

	dpu_writel(hdev->regs, DPU_WR_ADDR_C_H, PYHS_ADDR_HIGH(task->dst_phy_addr_c));
	dpu_writel(hdev->regs, DPU_WR_ADDR_C_L, PYHS_ADDR_LOW(task->dst_phy_addr_c));
	dpu_writel(hdev->regs, DPU_WR_STRIDE_C, task->dst_stride_c);

	dpu_writel(hdev->regs, DPU_WR_FBC_EN, 0);
}

static int ax620e_draw_start(struct draw_task *task)
{
	struct dpu_hw_device *hdev = (struct dpu_hw_device *)task->data;

	dpu_writel(hdev->regs, DPU_DRAW_EN, 0);
	dpu_writel(hdev->regs, DPU_DRAW_UP, 0);

	if (draw_color_space_cfg(task)) {
		VO_ERROR("failed to dpu%d draw csc\n", hdev->id);
		return -EINVAL;
	}

	draw_in_cfg(task);

	draw_out_cfg(task);

	dpu_writel(hdev->regs, DPU_G0_BLENDING, 0);

	dpu_writel(hdev->regs, DPU_DRAW_UP, 1);
	dpu_writel(hdev->regs, DPU_DRAW_EN, 1);

	VO_DEBUG("dpu%d reso: %dx%d, fmt: %d, stride: [%d,%d], phy_addr: [0x%llx,0x%llx\n", hdev->id,
		 task->dst_w, task->dst_h, task->dst_fmt,
		 task->dst_stride_y, task->dst_stride_c,
		 task->dst_phy_addr_y, task->dst_phy_addr_c);

	return 0;
}

static void ax620e_dispc_enable(struct dpu_hw_device *hdev)
{
	dispc_enable(hdev);
}

static void ax620e_dispc_disable(struct dpu_hw_device *hdev)
{
	dispc_disable(hdev);
}

static void ax620e_dispc_set_out_mode(struct dpu_hw_device *hdev, struct dispc_out_mode dispc_out)
{
	u32 out_mode = 0;

	hdev->out_mode = dispc_out;

	switch (dispc_out.mode) {
	case OUT_MODE_LVDS:
	case OUT_MODE_DSI_DPI_VIDEO:
		out_mode |= (0x1 << 0);
		break;

	case OUT_MODE_DSI_SDI_VIDEO:
	case OUT_MODE_DSI_SDI_CMD:
		out_mode |= (0x1 << 1);
		break;

	case OUT_MODE_BT601:
	case OUT_MODE_BT656:
	case OUT_MODE_BT1120:
		if(OUT_MODE_BT601 == dispc_out.mode) {
			dpu_writel(hdev->regs, DPU_PIN_SRC_SEL4, 0x101112);
		} else if (OUT_MODE_BT1120 == dispc_out.mode) {
			dpu_writel(hdev->regs, DPU_PIN_SRC_SEL2, 0x09080908);
			dpu_writel(hdev->regs, DPU_PIN_SRC_SEL3, 0x0d0c0b0a);
			dpu_writel(hdev->regs, DPU_PIN_SRC_SEL4, 0x11100f0e);
		}
		out_mode |= (0x1 << 2);
		break;

	case OUT_MODE_DPI:
		dpu_writel(hdev->regs, DPU_PIN_SRC_SEL4, 0x111210);
		out_mode |= (0x1 << 3);
		break;

	default:
		VO_ERROR("unsupported out_mode,mode = 0x%x\n", dispc_out.mode);
		return;
	}

	dispc_set_format_in(hdev, dispc_out.fmt_in);
	dispc_set_format_out(hdev, dispc_out.fmt_out);
	dispc_set_out_mode(hdev, out_mode);
}

static void dispc_dither(struct dpu_hw_device *dev)
{
	dpu_writel(dev->regs, DPU_DITHER_EN, 1);
	dpu_writel(dev->regs, DPU_DITHER_UP, 1);
}

static int ax620e_dispc_config(struct dpu_hw_device *hdev, struct ax_disp_mode *mode)
{
	int tmp, ret = 0;
	u32 reso, bt_mode, clk;
	struct dispc_out_mode dispc_out;

	hdev->mode = *mode;
	if (hdev->mode.flags & MODE_FLAG_INTERLACE) {
		VO_ERROR("interlace mode is not supported in boot\n");
		ret = -EINVAL;
		goto exit;
	}

	dpu_set_mode(hdev);

	ret = display_out_mode_convert(mode, &dispc_out);
	if (ret)
		goto exit;

	ax620e_dispc_set_out_mode(hdev, dispc_out);

	reso = ((mode->vdisplay << DISPC_RESO_HEIGHT_SHIFT) | mode->hdisplay);

	dispc_set_disp_reso(hdev, reso);

	dispc_set_rgb2yuv_matrix(hdev, hdev->dispc_rgb2yuv_matrix);
	dispc_set_yuv2rgb_matrix(hdev, hdev->dispc_yuv2rgb_matrix);

	dispc_set_timings(hdev, &hdev->mode);

	VO_INFO("mode: %d, fmt_in: %d, fmt_out: %d\n", dispc_out.mode, dispc_out.fmt_in, dispc_out.fmt_out);

	clk = mode->clock * 1000;

	tmp = dispc_out.mode;
	switch (tmp) {
	case OUT_MODE_BT601:
	case OUT_MODE_BT656:
	case OUT_MODE_BT1120:
		if (dispc_out.mode == OUT_MODE_BT656)
			bt_mode = 0;
		else if (dispc_out.mode == OUT_MODE_BT601)
			bt_mode = 1;
		else
			bt_mode = 2;

		if (tmp == OUT_MODE_BT601 || tmp == OUT_MODE_BT656)
			clk *= 2;

		dispc_set_bt_mode(hdev, bt_mode);

		break;
	}

	dispc_dither(hdev);

	dpu_writel(hdev->regs, DPU_DISP_CLK, 0);

	ret = pixel_clk_set_rate(hdev->id, mode->clock * 1000);
	if (ret)
		goto exit;

	ret = dpu_glb_path_config(hdev->id, tmp);
	if (ret)
		goto exit;

exit:
	VO_INFO("dispc%d config %s\n", hdev->id, ret ? "failed" : "success");

	return ret;
}

static void ax620e_dispc_set_buffer(struct dpu_hw_device *hdev, u64 addr_y, u64 addr_uv, u32 stride_y, u32 stride_uv)
{
	VO_DEBUG("dpu%d addr: [0x%llx, 0x%llx], stride: [%d, %d]\n", hdev->id,
		 addr_y, addr_uv, stride_y, stride_uv);

	dispc_update_lock(hdev);

	dpu_writel(hdev->regs, DPU_RD_FBDC_EN, 0);

	dpu_writel(hdev->regs, DPU_RD_ADDR_Y_l, PYHS_ADDR_LOW(addr_y));
	dpu_writel(hdev->regs, DPU_RD_ADDR_Y_H, PYHS_ADDR_HIGH(addr_y));
	dpu_writel(hdev->regs, DPU_RD_ADDR_C_l, PYHS_ADDR_LOW(addr_uv));
	dpu_writel(hdev->regs, DPU_RD_ADDR_C_H, PYHS_ADDR_HIGH(addr_uv));
	dpu_writel(hdev->regs, DPU_RD_STRIDE_Y, stride_y);
	dpu_writel(hdev->regs, DPU_RD_STRIDE_C, stride_uv);

	dispc_update_unlock(hdev);
}

static int ax620e_hw_init(struct dpu_hw_device *hdev)
{
	const struct color_space_cfg *cs_cfg;

	cs_cfg = &cs_cfgs;
	if (!cs_cfg)
		return -EINVAL;

	hdev->dispc_rgb2yuv_matrix = &cs_cfg->rgb2yuv_cfg;
	hdev->dispc_yuv2rgb_matrix = &cs_cfg->yuv2rgb_cfg;

	hdev->is_online = false;

	dpu_glb_init(hdev->id);

	dpu_intr_mask_all(hdev);

	draw_disable(hdev);

	dispc_disable(hdev);

	dpu_intr_sts_clr_all(hdev);

	dpu_writel(hdev->regs, DPU_AXI_EN, 0x1);

	VO_INFO("dpu%d hw init success\n", hdev->id);

	return 0;
}

static int ax620e_dpu_init(struct dpu_hw_device *hdev)
{
	return ax620e_hw_init(hdev);
}

static void ax620e_dpu_deinit(struct dpu_hw_device *hdev)
{
}

struct dpu_hw_ops ax620e_dpu_hw_ops = {
	.dpu_init = ax620e_dpu_init,
	.dpu_deinit = ax620e_dpu_deinit,
	.dispc_config = ax620e_dispc_config,
	.dispc_enable = ax620e_dispc_enable,
	.dispc_disable = ax620e_dispc_disable,
	.dispc_set_buffer = ax620e_dispc_set_buffer,
	.task_valid = ax620e_draw_task_valid,
	.draw_start = ax620e_draw_start,
};
