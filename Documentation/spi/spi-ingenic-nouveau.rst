=================================
Kernel driver spi-ingenic-nouveau
=================================

Supported chips:

  * Ingenic X1000/X2000 series

    Datasheet: https://github.com/Ingenic-community/datasheets/

Author:
	Reimu NotMoe <reimu@sudomaker.com>


Description
-----------

A new driver of the SSI peripheral found in the Ingenic X series SoCs.

Improvements:

  - Interrupt driven

  - Small data optimization

  - DMA fills the entire FIFO space

  - Transmit only mode

  - Caching of frequently accessed registers

  - Correct gpio-cs and active-high chip select handling


Usage Notes
-----------

The SSI peripheral never supported controlling the hardware chip select lines
independently from data transfer logic, and this is true since the JZ47xx era.

Now with the gpio-cs there is a problem: The hardware will always active a CS
pin (SSI_CEx) when the transfer begins. Here are a few strategies:

  - Use only hardware CS (up to 2) and no gpio-cs at all
  - Use only one hardware CS, and some gpio-cs
  - Use gpio-cs for everything (you can disable SSI_CEx pinmux then)

In the current implementation, if you choose to use only one hardware CS, it
must be SSI_CE0 and it must appear in the first of ``cs-gpios`` and your SPI
device definitions. Here's an example:

::

	&ssi {
		status = "okay";

		pinctrl-names = "default";
		pinctrl-0 = <&pins_spi0>;

		num-cs = <3>;
		cs-gpios = <0>, <&gpa 18 GPIO_ACTIVE_LOW>, <&gpa 17 GPIO_ACTIVE_LOW>;

		lcd: lcd@0 {
			reg = <0>;
			compatible = "sitronix,st7735s";

			spi-max-frequency = <100000000>;
			rotate = <270>;
			width = <128>;
			height = <160>;
			txbuflen = <16384>;
			buswidth = <8>;
			fps = <60>;
			dc-gpios = <&gpa 10 0>;
			reset-gpios = <&gpa 11 GPIO_ACTIVE_LOW>;
		};

		ts: tsc2046@1 {
			reg = <1>;
			compatible = "ti,tsc2046";

			pinctrl-names = "default";
			pinctrl-0 = <&pins_xpt2046>;

			interrupt-parent = <&gpa>;
			interrupts = <19 0>;
			wakeup-source;

			spi-max-frequency = <1000000>;
			pendown-gpio = <&gpa 19 0>;
		};

		psram: ly68@2 {
			reg = <2>;
			compatible = "lyontek,ly68l6400";

			spi-max-frequency = <100000000>;
			op-sleep-ratio = <20>;
		};
	}
