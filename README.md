# Linux for Ingenic SoCs

[![Kernel Build](https://github.com/Ingenic-community/linux/actions/workflows/kernel-build.yml/badge.svg)](https://github.com/Ingenic-community/linux/actions/workflows/kernel-build.yml)

Linux kernel source tree with the latest features and modifications to unleash the full potential of Ingenic processors.

## Note
Last updated: 2025-03-27

Ingenic has ported Linux 6.6 LTS to XBurst2 processors. You can search on web or ask their sales for the source code.

This repo will only focus on XBurst1 processors from now on and aim to make use of their unique capabilites.

## Purpose
Adding stuff to upstream is a lengthy and tough job. Maintaining a separate repo can make new features available to people in a timely manner.

### Stuff that are broken in the upstream kernel and nobody cared
Last updated: 2023-08-03
| **Problem**                                       | **Affected Chips**                          | Fixed Here |
|---------------------------------------------------|---------------------------------------------|------------|
| Device to Host DMA operations return garbage data | All X series SoCs with a writeback L2 cache | ✅          |
| PDMA operations corrupts RAM randomly             | All X series SoCs                           | ✅          |
| Inaccurate UART baudrate calculation              | All Ingenic SoCs                            | ✅          |
| Suspend to RAM                                    | All X series SoCs                           | ✅          |
| Incorrect I2S MCLK PLL calculation                | X1000 series SoCs                           | ✅          |
| RTC on 12/24MHz clocksource                       | X1000 series SoCs                           | ✅          |
| Ability to use internal analog codec              | X1000 series SoCs                           | ✅          |
| SPI HW CS still enabled when using cs-gpios       | All X series SoCs                           | ✅          |
| dwc2_otg must use PIO mode on X1501               | X1501                                       | ✅          |

## Policy
We will always add support for longterm Linux versions, we will also do stable versions when we have time.

Upstream changes will be merged to each branch whenever needed, or every one or two weeks.

We're free to drop support for old longterm kernels if there is already support for a newer version. Same for stable kernels.

This repo is primarily for Ingenic SoCs, but it's not limited to these. We may incorporate other kinds of improvements as well.

## Status Matrix
| **Chip**                    | JZ series   | X1000 series      | X1600 series       | X2000 series |
|:----------------------------|:------------|:------------------|:-------------------|:-------------|
| **Booting**                 | ✅          | ✅            | ❓            | ✅            |
| **Suspend**                 | ✅          | ✅            | ❓            | ❌            |
| **SMP**                     | ✅          | N/A          | N/A          | ✅            |
| **DMA**                     | DMAC✅      | PDMA✅        | ❓            | PDMA✅        |
| **USB**                     | UHC✅ USBD✅ | OTG✅         | ❓            | ⏳            |
| **Ethernet**                | N/A        | MAC✅         | ❓            | ❓            |
| **SPI Master**              | SSI✅       | SSI✅         | ❓            | SSI✅         |
| **SPI Slave**               | N/A        | N/A          | ❌            | N/A          |
| **Quad SPI**                | N/A        | SFC✅         | ❓            | SFC✅         |
| **I2C**                     | I2C✅       | SMB✅         | ❓            | ❓            |
| **PWM**                     | TCU✅       | TCU✅         | ❓            | TCU✅ PWM❌    |
| **ADC**                     | ❌          | N/A          | ❌            | ❌            |
| **SD/MMC**                  | MSC✅       | MSC✅         | ❓            | SDHCI✅       |
| **I2S**                     | AIC✅       | AIC✅         | ❌            | ❌            |
| **Analog Codec**            | ✅          | ✅            | N/A          | ❌            |
| **PDMIC**                   | ❌          | ❌            | N/A          | ❌            |
| **Display**                 | LCDC✅      | SLCD✅        | ❌            | ❌            |
| **Camera**                  | CIM❓       | CIM✅         | ❌            | ❌            |
| **CAN**                     | N/A        | N/A          | ❌            | N/A          |
| **Parallel i80 bus**        | EMC✅       | ❓            | ❓            | ❓            |
| **CDBUS**                   | N/A        | N/A          | ❌            | N/A          |
| **Heterogeneous multicore** | VPU✅       | MCU✅         | ❓            | ❓            |
| **Accelerator**             | ❌          | ❌            | ❌            | ❌            |

## Notes
Need help? Have a question? Feel free to open an issue.

You're welcome to push the changes here to upstream if not already done by us, but please retain the original author information.

License information are clearly stated in each file. Please respect them.

## FAQ

### No support for T series?

Unlike the good old JZ and X series, the T series is in tight control of a subsidiary company of Ingenic in a distant city. And unfortunately they have a very different philosophy of treating customers. You can urge them to release the related datasheets to public by sending emails to support_hf@ingenic.com and marketing@lumissil.com.
