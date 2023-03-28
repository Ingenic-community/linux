# Linux for Ingenic SoCs

[![Kernel Build](https://github.com/Ingenic-community/linux/actions/workflows/kernel-build.yml/badge.svg)](https://github.com/Ingenic-community/linux/actions/workflows/kernel-build.yml)

Linux kernel source tree with the latest features and modifications to unleash the full potential of Ingenic processors.

## Purpose
Adding stuff to upstream is a lengthy and tough job. Maintaining a separate repo can make new features available to people in a timely manner.

## Policy
We will always add support for longterm Linux versions, we will also do stable versions when we have time.

Upstream changes will be merged to each branch whenever needed, or every one or two weeks.

We're free to drop support for old longterm kernels if there is already support for a newer version. Same for stable kernels.

This repo is primarily for Ingenic SoCs, but it's not limited to these. We may incorporate other kinds of improvements as well.

## Status Matrix
| **Chip**                    | JZ series   | X1000 series      | X1600 series       | X2000 series |
|-----------------------------|-------------|-------------------|--------------------|--------------|
| **Booting**                 | ✅           | ✅                 | ❓                  | ✅            |
| **Suspend**                 | ✅           | ✅                 | ❓                  | ❌            |
| **SMP**                     | ✅           | N/A               | N/A                | ✅            |
| **DMA**                     | DMAC✅       | PDMA✅             | ❓                  | PDMA✅        |
| **USB**                     | UHC✅ USBD✅  | OTG✅              | ❓                  | ⏳            |
| **Ethernet**                | N/A         | MAC✅              | ❓                  | ❓            |
| **SPI**                     | SSI✅        | SSI✅ SFC✅         | SSI❓ SFC❓ SSI_SLV❌ | SSI✅ SFC✅    |
| **I2C**                     | I2C✅        | SMB✅              | ❓                  | ❓            |
| **PWM**                     | TCU✅        | TCU✅              | ❓                  | TCU✅ PWM❌    |
| **ADC**                     | ❌           | N/A               | ❌                  | ❌            |
| **SD/MMC**                  | MSC✅        | MSC✅              | ❓                  | SDHCI✅       |
| **Audio**                   | AIC✅ Codec✅ | AIC✅ Codec✅ DMIC❌ | ❌                  | ❌            |
| **Display**                 | LCDC✅       | SLCD✅             | ❌                  | ❌            |
| **Camera**                  | CIM❌        | CIM❌              | ❌                  | ❌            |
| **Heterogeneous multicore** | VPU✅        | MCU✅              |                    |              |
| **Accelerator**             | ❌           | ❌                 | ❌                  | ❌            |

## Notes
Need help? Have a question? Feel free to open an issue.

You're welcome to push the changes here to upstream if not already done by us, but please retain the original author information.

License information are clearly stated in each file. Please respect them.

## FAQ

### No support for T series?

Unlike the good old JZ and X series, the T series is in tight control of a subsidiary company of Ingenic in a distant city. And unfortunately they have a very different philosophy of treating customers. You can urge them to release the related datasheets to public by sending emails to support_hf@ingenic.com and marketing@lumissil.com.
