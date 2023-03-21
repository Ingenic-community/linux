#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

#include "sdhci-pltfm.h"

/* Software redefinition caps */
#define CAPABILITIES1_SW	0x276dc898
#define CAPABILITIES2_SW	0

#define CPM_MSC0_CLK_R		(0xB0000068)
#define CPM_MSC1_CLK_R		(0xB00000a4)
#define CPM_MSC2_CLK_R		(0xB00000a8)

struct sdhci_ingenic_pdata {
	unsigned int    host_caps;
	unsigned int    host_caps2;
	unsigned int    pm_caps;

	unsigned int	pio_mode;
	unsigned int	enable_autocmd12;
};

struct sdhci_ingenic {
	struct device			*dev;
	struct sdhci_host		*host;
	struct platform_device		*pdev;
	struct sdhci_ingenic_pdata	*pdata;
	struct clk			*clk_cgu;
	struct clk			*clk_ext;
	struct clk			*parent;
};

static unsigned int sdhci_ingenic_get_cpm_msc(struct sdhci_host *host)
{
	char msc_ioaddr[16];
	unsigned int cpm_msc;
	sprintf(msc_ioaddr, "0x%x", (unsigned int)host->ioaddr);

	if (!strcmp(msc_ioaddr ,"0xb3450000"))
		cpm_msc = CPM_MSC0_CLK_R;
	if (!strcmp(msc_ioaddr ,"0xb3460000"))
		cpm_msc = CPM_MSC1_CLK_R;
	if (!strcmp(msc_ioaddr ,"0xb3490000"))
		cpm_msc = CPM_MSC2_CLK_R;

	return cpm_msc;
}

/**
 * sdhci_ingenic_msc_tuning  Enable msc controller tuning
 *
 * Tuning rx phase
 * */
static void sdhci_ingenic_en_msc_tuning(struct sdhci_host *host, unsigned int cpm_msc)
{
	if (host->mmc->ios.timing & MMC_TIMING_UHS_SDR50 ||
		host->mmc->ios.timing & MMC_TIMING_UHS_SDR104 ||
		host->mmc->ios.timing & MMC_TIMING_MMC_HS400)
		*(volatile unsigned int*)cpm_msc &= ~(0x1 << 20);
}

/**
 * sdhci_ingenic_set_clock - callback on clock change
 * @host: The SDHCI host being changed
 * @clock: The clock rate being requested.
 *
 * When the card's clock is going to be changed, look at the new frequency
 * and find the best clock source to go with it.
*/
static void sdhci_ingenic_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);
	unsigned int cpm_msc = sdhci_ingenic_get_cpm_msc(host);

	if (clock == 0)
		return;

	sdhci_set_clock(host, clock);

	if (clock > 400000) {
		clk_disable_unprepare(sdhci_ing->clk_cgu);
		clk_set_parent(sdhci_ing->clk_cgu, sdhci_ing->parent);
		clk_prepare_enable(sdhci_ing->clk_cgu);
	} else {
		clk_disable_unprepare(sdhci_ing->clk_cgu);
		clk_set_parent(sdhci_ing->clk_cgu, sdhci_ing->clk_ext);
		clk_prepare_enable(sdhci_ing->clk_cgu);
	}

	clk_set_rate(sdhci_ing->clk_cgu, clock);

	printk("%s, set clk: %d, get_clk_rate=%ld\n", __func__, clock, clk_get_rate(sdhci_ing->clk_cgu));

	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
		host->mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		sdhci_ingenic_en_msc_tuning(host, cpm_msc);
}

static struct sdhci_ops sdhci_ingenic_ops = {
	.set_clock = sdhci_ingenic_set_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static int sdhci_ingenic_probe(struct platform_device *pdev)
{
	struct sdhci_ingenic_pdata *pdata;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_ingenic *sdhci_ing;
	int ret, irq;

	if (!pdev->dev.of_node) {
		dev_err(dev, "no device data specified\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	host = sdhci_alloc_host(dev, sizeof(struct sdhci_ingenic));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	sdhci_ing = sdhci_priv(host);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		return ret;
	}

	sdhci_ing->clk_cgu = devm_clk_get(&pdev->dev, "sdhci");
	if(!sdhci_ing->clk_cgu) {
		dev_err(&pdev->dev, "Failed to Get MSC clk!\n");
		return PTR_ERR(sdhci_ing->clk_cgu);
	}

	sdhci_ing->clk_ext = devm_clk_get(&pdev->dev, "ext");

	sdhci_ing->parent = clk_get_parent(sdhci_ing->clk_cgu);

	clk_prepare_enable(sdhci_ing->clk_cgu);

	sdhci_ing->host = host;
	sdhci_ing->dev  = &pdev->dev;
	sdhci_ing->pdev = pdev;
	sdhci_ing->pdata = pdata;

	host->ioaddr= of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(host->ioaddr))
		return PTR_ERR(host->ioaddr);

	host->hw_name = "ingenic-sdhci";
	host->ops = &sdhci_ingenic_ops;
	host->quirks = 0;
	host->irq = irq;

	/* Software redefinition caps */
	host->quirks |= SDHCI_QUIRK_MISSING_CAPS;
	host->caps = CAPABILITIES1_SW;
	host->caps1 = CAPABILITIES2_SW;

	/* not check wp */
	host->quirks |= SDHCI_QUIRK_INVERTED_WRITE_PROTECT;

	/* Setup quirks for the controller */
	host->quirks |= SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;
	host->quirks |= SDHCI_QUIRK_NO_HISPD_BIT;

	/* Data Timeout Counter Value */
	host->quirks |= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;

	/* This host supports the Auto CMD12 */
	host->quirks |= SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12;

	host->quirks |= SDHCI_QUIRK_32BIT_DMA_ADDR | SDHCI_QUIRK_32BIT_DMA_SIZE;

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		dev_err(dev, "mmc_of_parse() failed\n");
		return ret;
	}

	sdhci_get_of_property(pdev);

	sdhci_enable_v4_mode(host);
	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_clk;
	}

	return 0;
err_clk:
	clk_disable_unprepare(sdhci_ing->clk_cgu);
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_ingenic_remove(struct platform_device *pdev)
{
	struct sdhci_host *host =  platform_get_drvdata(pdev);
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);

	sdhci_remove_host(host, 1);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	clk_disable_unprepare(sdhci_ing->clk_cgu);

	sdhci_free_host(host);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sdhci_ingenic_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host) ;

	sdhci_suspend_host(host);

	clk_disable_unprepare(sdhci_ing->clk_cgu);

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int sdhci_ingenic_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);

	pinctrl_select_default_state(dev);

	clk_prepare_enable(sdhci_ing->clk_cgu);

	sdhci_resume_host(host);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sdhci_ingenic_pmops, sdhci_ingenic_suspend,
	sdhci_ingenic_resume);

static const struct of_device_id sdhci_ingenic_dt_matches[] = {
	{.compatible = "ingenic,x2000-sdhci", },
	{.compatible = "ingenic,x2500-sdhci", },
	{},
};
MODULE_DEVICE_TABLE(of, sdhci_ingenic_dt_matches);

static struct platform_driver sdhci_ingenic_driver = {
	.probe		= sdhci_ingenic_probe,
	.remove		= sdhci_ingenic_remove,
	.driver		= {
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.name	= "ingenic,sdhci",
		.pm	= &sdhci_ingenic_pmops,
		.of_match_table = of_match_ptr(sdhci_ingenic_dt_matches),
	},
};

module_platform_driver(sdhci_ingenic_driver);

MODULE_DESCRIPTION("Ingenic SDHCI (MSC) driver");
MODULE_AUTHOR("Large Dipper <wangquan.shao@ingenic.cn>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20160808");
