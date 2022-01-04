These are linux drivers for ftdi mpsse devices i2c spi.

UPDATE: added property support to the spi driver, and gpio naming. Currently pin numbers (amount avail) are set for ft4232 will do an update for ft2232 someday (uhg) Also with the product (in ftdi eeprom) set to ft2232H-16ton or ft4232H-16ton, and the ftdi_sio patch applied, ftdi_sio will ignore the first 2 interfaces (on ft4232 or both interfaces on ft2232) and the spi driver will attach to the first interface and i2c on second (and ftdi_sio serial on interfaces 3/4 on ft4232 and shit outa luck on serial for ft2232 in this setup) In my x86 kernel source (tons of stuff for i2c spi etc in this kernel) I have brought back fb-device because god help me I cannot get the fb-ili9341 from fbtft to take the platform data it needs. Seemingly all options for platform data with fbtft have been removed except for devicetree...and I cannot fake it yet. I did have the tinydrm ili9341 driver taking the platform data correctly from driver and loading but now it just freezes my machines. I was playing with gpio-lookup instead of  ftdi_spi_bus_dev_io and maybe thats when it was working or kernel changes broke the setup (been a while in between testings with newer kernel versions in between) ill throw a script to make loading the fbtft fb-device fb_ili9341 stuff easier, itll auto detect the gpio pin numbers assigned to dc/reset by your machine but you will have to edit it for the spi bus number. 

my kernel source with these drivers plus lots more already added; https://github.com/bm16ton/linux-kernel

to integrate these into your own kernel source 

cd kernel-source

cp ../ft2232-mpsse-i2c-spi-kern-drivers/i2c-ftdi/mpsse.h drivers/i2c/busses/mpsse.h

cp ../ft2232-mpsse-i2c-spi-kern-drivers/i2c-ftdi/i2c-ftdi.c drivers/i2c/busses/i2c-ftdi.c

edit drivers/i2c/busses/Kconfig

EXAMPLE KCONFIG FOR i2c-ftdi;

config I2C_FTDI
	tristate "ft2232h I2C master support"
	depends on USB
	help
	  Say yes here to access the I2C part of the ftdi
	  ft2232h, and probly others.

EXAMPLE Makefile entry for i2c-ftdi

obj-$(CONFIG_I2C_FTDI)	+= i2c-ftdi.o

for spi;

cp ../ft2232-mpsse-i2c-spi-kern-drivers/spi-ftdi/ft232h-intf.c drivers/usb/misc/

EXAMPLE Kconfig for ft232h-intf; edit drivers/usb/misc/Kconfig;

config USB_FT232H_INTF
	tristate "FTDI FT232H SPI and platform data configuration interface driver"
	help
	  Enable driver support for the FT232H/ft2232H based USB-GPIO/SPI/FIFO
	  interface adapter for SPI and FPGA configuration. Additional drivers
	  such as spi-ftdi-mpsse, etc must be enabled in order to use the
	  functionality of the device.

	  To compile this driver as a module, choose M here: the
	  module will be called ft232h_intf.

EXAMPLE Makefile for ft232h-intf

obj-$(CONFIG_USB_FT232H_INTF)		+= ft232h-intf.o

cp ../ft2232-mpsse-i2c-spi-kern-drivers/spi-ftdi/spi-ftdi-mpsse.c drivers/spi/

Example Kconfig; drivers/spi/Kconfig;

config SPI_FTDI_MPSSE
	tristate "FTDI MPSSE SPI controller"
	depends on USB_FT232H_INTF || COMPILE_TEST || GPIOLIB && SYSFS
 	select GPIO_SYSFS
 	select GPIOLIB_IRQCHIP
	help
	  FT232H supports SPI in MPSSE mode. This driver provides MPSSE
	  SPI controller in master mode.

example Makefile; 

obj-$(CONFIG_SPI_FTDI_MPSSE)		+= spi-ftdi-mpsse.o

cp ../ft2232-mpsse-i2c-spi-kern-drivers/spi-ftdi/ft232h-intf.h ./include/linux/usb/ft232h-intf.h

And that should do it!

the spi driver originally came from;

 http://git-old.denx.de/?p=linux-denx/linux-denx-agust.git

 only slight edits where needed. Add your spi device module alias and cs

 pin info to line 1203 of ft232h-intf.c , then recompile, i use in tree but

 will write a Makefile someday. For in tree cp ft232h-intf.c to drivers/usb/misc

 cp ft232h-int.h to include/linux/usb/ and cp spi-ftdi-mpsse.c to

 drivers/spi/ then edit the Kconfig and Makefiles checkout my kernel for examples

 https://github.com/bm16ton/yoga-c630-linux-kernel

 The i2c came from https://github.com/krinkinmu/bootlin also checkout his

 awesome writeup on reverse engineering mpsse;

 https://krinkinmu.github.io/2020/08/02/ftdi.html

 https://krinkinmu.github.io/2020/09/05/ftdi-protocol.html

 https://krinkinmu.github.io/2020/09/06/ftdi-i2c.html

 This driver I had to change more on to get it to work, the control usb

 packets, Added smbus support, all relatively minor.

 These have been tested on ft2232h, kernel 5.12,and on mpsse bank A only.

 UPDATE i2c-ftdi only binds to bank A now,also patch included for ftdi-sio

 to skip bank A if Product string is ft2232H-16ton (use ftdi_eeprom)

 ---old Even tho these will bind to both banks doubtful bank B will work in

 ---oldcurrent configuration.

 SPI TODO,find better way to supply spi device

 platformdata (mod params? add info to eeprom?) maybe do a dirty hack for irq pin.

 I2C TODO, Add sysfs entry for changing bus speed, was more but dont remember atm.

 TODO BETTER README! Its late uhm or early now and just got it working, ill

 update more later.


UPDATE the rx and tx function ftdi_spi_tx_rx was erroring on spidev devices without

mosi, so its disabled. lines 362 uncomment/recomment that section to bring back

in spi-ftdimpsse.c Also starting sumwhere in 5.13 spi delay_usecs got switched to

delay.value. If on older kernel replace lines starting at 381 in spi-ftdimpsse.c

		if (t->delay.value) {
			u16 us = t->delay.value;

with;

		if (t->delay_usecs) {
			u16 us = t->delay_usecs;

The i2c driver now has a kernel param to switch 400k and 100k

TODO make i2c speed a sysfs option, add gpio to i2c, etc etc
