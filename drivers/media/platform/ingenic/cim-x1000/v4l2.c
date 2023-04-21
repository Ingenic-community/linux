// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2023 SudoMaker, Ltd.
 * Author: Reimu NotMoe <reimu@sudomaker.com>
 *
 * Based on various drivers.
 */

#include <linux/device.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#include "cim.h"

#define CIM_DEFAULT_WIDTH	640
#define CIM_DEFAULT_HEIGHT	480

static const struct x1000_cim_format x1000_cim_formats[] = {
	{
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.bpp		= 16,
		.hsub		= 2,
		.vsub		= 2,
	},

	{
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.bpp		= 16,
		.hsub		= 2,
		.vsub		= 2,
	},

	{
		.mbus_code	= MEDIA_BUS_FMT_RGB565_2X8_LE,
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.bpp		= 16,
		.hsub		= 2,
		.vsub		= 2,
	},

	{
		.mbus_code	= MEDIA_BUS_FMT_RGB565_2X8_BE,
		.fourcc		= V4L2_PIX_FMT_RGB565X,
		.bpp		= 16,
		.hsub		= 2,
		.vsub		= 2,
	},
};

const struct x1000_cim_format *x1000_cim_find_format(const u32 *fourcc,
						     const u32 *mbus)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(x1000_cim_formats); i++) {
		if (fourcc && *fourcc != x1000_cim_formats[i].fourcc)
			continue;

		if (mbus && *mbus != x1000_cim_formats[i].mbus_code)
			continue;

		return &x1000_cim_formats[i];
	}

	return NULL;
}

static int x1000_cim_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strscpy(cap->card, "x1000-cim", sizeof(cap->card));

	return 0;
}

static int x1000_cim_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	if (inp->index != 0)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(inp->name, "Camera", sizeof(inp->name));

	return 0;
}

static int x1000_cim_g_input(struct file *file, void *fh,
			     unsigned int *i)
{
	*i = 0;

	return 0;
}

static int x1000_cim_s_input(struct file *file, void *fh,
			     unsigned int i)
{
	if (i != 0)
		return -EINVAL;

	return 0;
}

static void x1000_cim_try_fmt_impl(struct x1000_cim *cim,
			       struct v4l2_pix_format *pix)
{
	const struct x1000_cim_format *_fmt;
	unsigned int height, width;

	_fmt = x1000_cim_find_format(&pix->pixelformat, NULL);
	if (!_fmt)
		_fmt = &x1000_cim_formats[0];

	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix->colorspace);
	pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	pix->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, pix->colorspace,
							  pix->ycbcr_enc);

	pix->pixelformat = _fmt->fourcc;

	/* Align the width and height on the subsampling */
	width = ALIGN(pix->width, _fmt->hsub);
	height = ALIGN(pix->height, _fmt->vsub);

	/* Clamp the width and height to our capabilities */
	pix->width = clamp(width, _fmt->hsub, CIM_MAX_WIDTH);
	pix->height = clamp(height, _fmt->vsub, CIM_MAX_HEIGHT);

	pix->sizeimage = pix->width * pix->height * 2;

	cim->cim_fmt = _fmt;

#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	pr_info("=> try_fmt_impl: %ux%u, sizeimage=%u\n", pix->width, pix->height, pix->sizeimage);
#endif
}

static int x1000_cim_try_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_format *f)
{
	struct x1000_cim *cim = video_drvdata(file);
		struct v4l2_subdev_pad_config pad_cfg = {};
	struct v4l2_subdev_state pad_state = {
		.pads = &pad_cfg
	};
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	int ret;

#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	pr_info("=> try_fmt_vid_cap\n");
#endif
	x1000_cim_try_fmt_impl(cim, &f->fmt.pix);
	v4l2_fill_mbus_format(&format.format, &f->fmt.pix, cim->cim_fmt->mbus_code);
	ret = v4l2_subdev_call(cim->src_subdev, pad, set_fmt, &pad_state, &format);
	if (ret)
		return ret;

	return 0;
}

static int x1000_cim_s_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct x1000_cim *cim = video_drvdata(file);
	struct v4l2_subdev_pad_config pad_cfg = {};
	struct v4l2_subdev_state pad_state = {
		.pads = &pad_cfg
	};
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;
#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	pr_info("=> s_fmt_vid_cap\n");
#endif
	x1000_cim_try_fmt_impl(cim, &f->fmt.pix);
	v4l2_fill_mbus_format(&format.format, &f->fmt.pix, cim->cim_fmt->mbus_code);
	ret = v4l2_subdev_call(cim->src_subdev, pad, set_fmt, &pad_state, &format);
	if (ret)
		return ret;

	cim->fmt = f->fmt.pix;

	return 0;
}

static int x1000_cim_g_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct x1000_cim *cim = video_drvdata(file);
#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	pr_info("=> g_fmt_vid_cap\n");
#endif
	f->fmt.pix = cim->fmt;

	return 0;
}

static int x1000_cim_enum_fmt_vid_cap(struct file *file, void *priv,
				      struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(x1000_cim_formats))
		return -EINVAL;

	f->pixelformat = x1000_cim_formats[f->index].fourcc;

	return 0;
}

static const struct v4l2_ioctl_ops x1000_cim_ioctl_ops = {
	.vidioc_querycap		= x1000_cim_querycap,

	.vidioc_enum_fmt_vid_cap	= x1000_cim_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= x1000_cim_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= x1000_cim_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= x1000_cim_try_fmt_vid_cap,

	.vidioc_enum_input		= x1000_cim_enum_input,
	.vidioc_g_input			= x1000_cim_g_input,
	.vidioc_s_input			= x1000_cim_s_input,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static int x1000_cim_open(struct file *file)
{
	struct x1000_cim *cim = video_drvdata(file);
	int ret;

	ret = mutex_lock_interruptible(&cim->lock);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(cim->dev);
	if (ret < 0)
		goto err_unlock;

	ret = v4l2_pipeline_pm_get(&cim->vdev.entity);
	if (ret)
		goto err_pm_put;

	ret = v4l2_fh_open(file);
	if (ret)
		goto err_pipeline_pm_put;

	mutex_unlock(&cim->lock);

	return 0;

err_pipeline_pm_put:
	v4l2_pipeline_pm_put(&cim->vdev.entity);

err_pm_put:
	pm_runtime_put(cim->dev);

err_unlock:
	mutex_unlock(&cim->lock);

	return ret;
}

static int x1000_cim_release(struct file *file)
{
	struct x1000_cim *cim = video_drvdata(file);

	mutex_lock(&cim->lock);

	_vb2_fop_release(file, NULL);

	v4l2_pipeline_pm_put(&cim->vdev.entity);
	pm_runtime_put(cim->dev);

	mutex_unlock(&cim->lock);

	return 0;
}

static const struct v4l2_file_operations x1000_cim_fops = {
	.owner		= THIS_MODULE,
	.open		= x1000_cim_open,
	.release	= x1000_cim_release,
	.unlocked_ioctl	= video_ioctl2,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};

static const struct v4l2_mbus_framefmt x1000_cim_pad_fmt_default = {
	.width = CIM_DEFAULT_WIDTH,
	.height = CIM_DEFAULT_HEIGHT,
	.code = MEDIA_BUS_FMT_YUYV8_2X8,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_RAW,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

static int x1000_cim_subdev_init_cfg(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_state *sd_state)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_get_try_format(subdev, sd_state, CSI_SUBDEV_SINK);
	*fmt = x1000_cim_pad_fmt_default;

	return 0;
}

static int x1000_cim_subdev_get_fmt(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_format *fmt)
{
	struct x1000_cim *cim = container_of(subdev, struct x1000_cim, subdev);
	struct v4l2_mbus_framefmt *subdev_fmt;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		subdev_fmt = v4l2_subdev_get_try_format(subdev, sd_state,
							fmt->pad);
	else
		subdev_fmt = &cim->subdev_fmt;

	fmt->format = *subdev_fmt;

	return 0;
}

static int x1000_cim_subdev_set_fmt(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_format *fmt)
{
	struct x1000_cim *cim = container_of(subdev, struct x1000_cim, subdev);
	struct v4l2_mbus_framefmt *subdev_fmt;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		subdev_fmt = v4l2_subdev_get_try_format(subdev, sd_state,
							fmt->pad);
	else
		subdev_fmt = &cim->subdev_fmt;

#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	pr_info("=> subdev_set_fmt: pad=%u\n", fmt->pad);
#endif

	/* We can only set the format on the sink pad */
	if (fmt->pad == CSI_SUBDEV_SINK) {
		/* It's the sink, only allow changing the frame size */
		subdev_fmt->width = fmt->format.width;
		subdev_fmt->height = fmt->format.height;
		subdev_fmt->code = fmt->format.code;
	}

	fmt->format = *subdev_fmt;

	return 0;
}

static int
x1000_cim_subdev_enum_mbus_code(struct v4l2_subdev *subdev,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *mbus)
{
	if (mbus->index >= ARRAY_SIZE(x1000_cim_formats))
		return -EINVAL;

	mbus->code = x1000_cim_formats[mbus->index].mbus_code;

	return 0;
}

static const struct v4l2_subdev_pad_ops x1000_cim_subdev_pad_ops = {
	.link_validate	= v4l2_subdev_link_validate_default,
	.init_cfg	= x1000_cim_subdev_init_cfg,
	.get_fmt	= x1000_cim_subdev_get_fmt,
	.set_fmt	= x1000_cim_subdev_set_fmt,
	.enum_mbus_code	= x1000_cim_subdev_enum_mbus_code,
};

const struct v4l2_subdev_ops x1000_cim_subdev_ops = {
	.pad = &x1000_cim_subdev_pad_ops,
};

int x1000_cim_v4l2_register(struct x1000_cim *cim)
{
	struct video_device *vdev = &cim->vdev;
	int ret;

	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vdev->v4l2_dev = &cim->v4l;
	vdev->queue = &cim->queue;
	strscpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));
	vdev->release = video_device_release_empty;
	vdev->lock = &cim->lock;

	/* Set a default format */
	cim->fmt.pixelformat = x1000_cim_formats[0].fourcc;
	cim->fmt.width = CIM_DEFAULT_WIDTH;
	cim->fmt.height = CIM_DEFAULT_HEIGHT;
	x1000_cim_try_fmt_impl(cim, &cim->fmt);

	vdev->fops = &x1000_cim_fops;
	vdev->ioctl_ops = &x1000_cim_ioctl_ops;
	video_set_drvdata(vdev, cim);

	ret = video_register_device(&cim->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	dev_info(cim->dev, "Device registered as %s\n",
		 video_device_node_name(vdev));

	return 0;
}
