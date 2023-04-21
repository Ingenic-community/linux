// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 * Author: qipengzhen <aric.pzqi@ingenic.com>
 *
 * Copyright (C) 2023 SudoMaker, Ltd.
 * Author: Reimu NotMoe <reimu@sudomaker.com>
 *
 * Based on various drivers.
 */

// TODO list:
// 1. Dynamic clock gating
// 2. JPEG support
// 3. Revisit frame size checking

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/videodev2.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mediabus.h>

#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "cim.h"


#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
void x1000_cim_dump_regs(struct x1000_cim *cim)
{
	BUG_ON(!cim);

#define STRING  "\t=\t0x%08x\n"
	printk("REG_CIM_CFG" 	 STRING, readl(cim->regs + CIM_CFG));
	printk("REG_CIM_CTRL" 	 STRING, readl(cim->regs + CIM_CTRL));
	printk("REG_CIM_CTRL2" 	 STRING, readl(cim->regs + CIM_CTRL2));
	printk("REG_CIM_STATE" 	 STRING, readl(cim->regs + CIM_STATE));

	printk("REG_CIM_IMR" 	 STRING, readl(cim->regs + CIM_IMR));
	printk("REG_CIM_IID" 	 STRING, readl(cim->regs + CIM_IID));
	printk("REG_CIM_DA" 	 STRING, readl(cim->regs + CIM_DA));
	printk("REG_CIM_FA" 	 STRING, readl(cim->regs + CIM_FA));

	printk("REG_CIM_FID" 	 STRING, readl(cim->regs + CIM_FID));
	printk("REG_CIM_CMD" 	 STRING, readl(cim->regs + CIM_CMD));
	printk("REG_CIM_WSIZE" 	 STRING, readl(cim->regs + CIM_SIZE));
	printk("REG_CIM_WOFFSET" STRING, readl(cim->regs + CIM_OFFSET));

	printk("REG_CIM_FS" 	 STRING, readl(cim->regs + CIM_FS));
#undef STRING
}
#endif

static const struct media_entity_operations x1000_cim_video_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int x1000_cim_notify_bound(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
	struct x1000_cim *cim = container_of(notifier, struct x1000_cim,
					     notifier);

	cim->src_subdev = subdev;
	cim->src_pad = media_entity_get_fwnode_pad(&subdev->entity,
						   subdev->fwnode,
						   MEDIA_PAD_FL_SOURCE);
	if (cim->src_pad < 0) {
		dev_err(cim->dev, "Couldn't find output pad for subdev %s\n",
			subdev->name);
		return cim->src_pad;
	}

	dev_dbg(cim->dev, "Bound %s pad: %d\n", subdev->name, cim->src_pad);
	return 0;
}

static int x1000_cim_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct x1000_cim *cim = container_of(notifier, struct x1000_cim,
					     notifier);
	struct v4l2_subdev *subdev = &cim->subdev;
	struct video_device *vdev = &cim->vdev;
	int ret;

	ret = v4l2_device_register_subdev(&cim->v4l, subdev);
	if (ret < 0)
		return ret;

	ret = x1000_cim_v4l2_register(cim);
	if (ret < 0)
		return ret;

	ret = media_device_register(&cim->mdev);
	if (ret)
		return ret;

	/* Create link from subdev to main device */
	ret = media_create_pad_link(&subdev->entity, CSI_SUBDEV_SOURCE,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		goto err_clean_media;

	ret = media_create_pad_link(&cim->src_subdev->entity, cim->src_pad,
				    &subdev->entity, CSI_SUBDEV_SINK,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		goto err_clean_media;

	ret = v4l2_device_register_subdev_nodes(&cim->v4l);
	if (ret < 0)
		goto err_clean_media;

	return 0;

err_clean_media:
	media_device_unregister(&cim->mdev);

	return ret;
}

static const struct v4l2_async_notifier_operations x1000_cim_notify_ops = {
	.bound		= x1000_cim_notify_bound,
	.complete	= x1000_cim_notify_complete,
};

static int x1000_cim_notifier_init(struct x1000_cim *cim)
{
	struct v4l2_fwnode_endpoint vep = {
		.bus_type = V4L2_MBUS_PARALLEL,
	};
	struct v4l2_async_subdev *asd;
	struct fwnode_handle *ep;
	int ret;

	v4l2_async_nf_init(&cim->notifier);

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(cim->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		goto out;

	cim->bus = vep.bus.parallel;

	asd = v4l2_async_nf_add_fwnode_remote(&cim->notifier, ep,
					      struct v4l2_async_subdev);
	if (IS_ERR(asd)) {
		ret = PTR_ERR(asd);
		goto out;
	}

	cim->notifier.ops = &x1000_cim_notify_ops;

out:
	fwnode_handle_put(ep);
	return ret;
}

static int x1000_cim_probe(struct platform_device *pdev)
{
	struct v4l2_subdev *subdev;
	struct video_device *vdev;
	struct x1000_cim *cim;
	int ret;
	int irq;

	cim = devm_kzalloc(&pdev->dev, sizeof(*cim), GFP_KERNEL);
	if (!cim)
		return -ENOMEM;
	platform_set_drvdata(pdev, cim);
	cim->dev = &pdev->dev;
	subdev = &cim->subdev;
	vdev = &cim->vdev;

	cim->max_buffer_cnt = 16;
	device_property_read_u32(cim->dev, "ingenic,max-buffer-count", &cim->max_buffer_cnt);

	cim->max_vmem_bytes = 8 * 1024 * 1024;
	device_property_read_u32(cim->dev, "ingenic,max-vmem-bytes", &cim->max_vmem_bytes);

	cim->mdev.dev = cim->dev;
	strscpy(cim->mdev.model, "Ingenic X1000 Series Video Capture Device",
		sizeof(cim->mdev.model));
	cim->mdev.hw_revision = 0;
	media_device_init(&cim->mdev);
	cim->v4l.mdev = &cim->mdev;

	cim->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cim->regs))
		return PTR_ERR(cim->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	cim->cim_clk = devm_clk_get(&pdev->dev, "cim");
	if (IS_ERR(cim->cim_clk)) {
		dev_err(&pdev->dev, "Couldn't get clock for cim\n");
		return PTR_ERR(cim->cim_clk);
	}

	// Errata: The CIM module requires the LCD clock to be enabled
	// in order to function properly.
	cim->lcd_clk = devm_clk_get(&pdev->dev, "lcd");
	if (IS_ERR(cim->lcd_clk)) {
		dev_err(&pdev->dev, "Couldn't get clock for lcd\n");
		return PTR_ERR(cim->lcd_clk);
	}

	/* Initialize subdev */
	v4l2_subdev_init(subdev, &x1000_cim_subdev_ops);
	subdev->flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	subdev->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	subdev->owner = THIS_MODULE;
	snprintf(subdev->name, sizeof(subdev->name), "x1000-cim-0");
	v4l2_set_subdevdata(subdev, cim);

	cim->subdev_pads[CSI_SUBDEV_SINK].flags =
		MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	cim->subdev_pads[CSI_SUBDEV_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&subdev->entity, CSI_SUBDEV_PADS,
				     cim->subdev_pads);
	if (ret < 0)
		return ret;

	cim->vdev_pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	vdev->entity.ops = &x1000_cim_video_entity_ops;
	ret = media_entity_pads_init(&vdev->entity, 1, &cim->vdev_pad);
	if (ret < 0)
		return ret;

	ret = x1000_cim_dma_register(cim, irq);
	if (ret)
		goto err_clean_pad;

	ret = x1000_cim_notifier_init(cim);
	if (ret)
		goto err_unregister_media;

	ret = v4l2_async_nf_register(&cim->v4l, &cim->notifier);
	if (ret) {
		dev_err(cim->dev, "Couldn't register our notifier.\n");
		goto err_unregister_media;
	}

	pm_runtime_enable(&pdev->dev);

	return 0;

err_unregister_media:
	media_device_unregister(&cim->mdev);
	x1000_cim_dma_unregister(cim);

err_clean_pad:
	media_device_cleanup(&cim->mdev);

	return ret;
}

static int x1000_cim_remove(struct platform_device *pdev)
{
	struct x1000_cim *cim = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&cim->notifier);
	v4l2_async_nf_cleanup(&cim->notifier);
	vb2_video_unregister_device(&cim->vdev);
	media_device_unregister(&cim->mdev);
	x1000_cim_dma_unregister(cim);
	media_device_cleanup(&cim->mdev);

	return 0;
}

static const struct of_device_id x1000_cim_of_match[] = {
	{ .compatible = "ingenic,x1000-cim" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, x1000_cim_of_match);

static int __maybe_unused x1000_cim_runtime_resume(struct device *dev)
{
	struct x1000_cim *cim = dev_get_drvdata(dev);

	clk_prepare_enable(cim->cim_clk);
	clk_prepare_enable(cim->lcd_clk);

	return 0;
}

static int __maybe_unused x1000_cim_runtime_suspend(struct device *dev)
{
	struct x1000_cim *cim = dev_get_drvdata(dev);

	clk_disable_unprepare(cim->lcd_clk);
	clk_disable_unprepare(cim->cim_clk);

	return 0;
}

static const struct dev_pm_ops x1000_cim_pm_ops = {
	SET_RUNTIME_PM_OPS(x1000_cim_runtime_suspend,
			   x1000_cim_runtime_resume,
			   NULL)
};

static struct platform_driver x1000_cim_driver = {
	.probe	= x1000_cim_probe,
	.remove	= x1000_cim_remove,
	.driver	= {
		.name		= "x1000-cim",
		.of_match_table	= x1000_cim_of_match,
		.pm		= &x1000_cim_pm_ops,
	},
};
module_platform_driver(x1000_cim_driver);

MODULE_DESCRIPTION("Ingenic X1000 series Camera Interface Module (CIM) driver");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_LICENSE("GPL");
