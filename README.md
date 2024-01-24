# Linux ft2232/mpsse/i2c/spi kernel drivers/modules

These are linux drivers for ftdi mpsse devices i2c spi.

### UPDATES:

Tired of rewritting eeprom of ft232h series between spi/i2c so i wrote a few more kernel parameters, udev rule, systemd service , and a simple first draft gui app that starts when the ft232h is plugged in (make sure product is ft232h-16ton) that lets you select i2c/spi/uart and options. Simply cd ftgui and run ./install-ftgui.sh   this will install all dependencies for gtkdialog then gtkdialogs souce compile it and install it, then install the udev rule, the systemd service, and gui app. Now when ft232h is plugged in it should autostart the app.
Remember to also patch ftdi-sio with the new ftdi-sio/2nd-ftdi_sio.patch patch.

IRQ now passes automatically no more hardcoding it!!

GPIO irq polling now works and has been very stable, obviously irq in spinlocks wont work, it is using the new gpiod instead of legacy the irq type is hardcoded in mpsse gpio add function and until i figureout a better way irq nmber is hard coded in spi device struct. But it works much better then i anticipated, tested with mcp251x and nrf24 irq requird kernel modules. 

Added newer patches for ftdi-sio to ignore the 16ton products, or if adventerus try the frankenstein ftdio driver in same folder with sync mode, all 3 types of gpio, and more, real ruff the gpio versions need a slight code change (one number), it has state machine etc. 
Added ft423hpq and ft232h auto detect so models ft4232h ft232h ft4233hpq or ft2232h auto detected
amount of gpios auto setup for each model.
ft232h i2c now works and I believe (hope) its using open drain lol.
mod params for 1m and 3.4m i2c clock speed no 3.4m setup code tho my ads1115 seems to not care, can only test clk speed up to 1m until pulseview and my fx2 stop being dinks. 
SPI most of the code for a polled irq (polling irq gpio sucks maybe not get back to it) can be enabled even tho not currently working/finnished with modparam.
A firmware for various st32f4's and stm32f103 for ub to i2c with gpio with irq support plus a kernel driver (modified i2c-tiny) is in the USB-GPIO folder and works but it doesnt auto export the gpio/irq so export set edge and direction to enble irq to pin, then unexport, hardcode the irq in ft232h-intf.c spi device section (IRQ will be printed to dmesg when module loads, so plug stm in first, set the irq in ft232h-intf.c and then load spi) This works much better then the polling methood ever will)
Unfortunatly the ft4233hpq is too new for any support from libftdi or pyftdi etc so non of the open source eepom writing utils for linux/opensource currently work. Gotta use ft-prog toset the product name to 16ton*
Maybe other stuff? I dont remember 

'''
model ft4232h or ft2232h auto detected
amount of gpios auto setup for each model
raw eeprom dump to syslog
kernel 5.15 did some funky spi things, IE spidev as modalias no longer works I use spi-petra now
/sys/kernel/mpsse_sysfs/eeprom gets created but isnt implemented (help would be great lol)
auto gpio numbers is a bit hackish so keep CS at GPIOL3 and below (dont go higher then 4)
Added of_tables and spi_tables tho this is a platform driver so not sure how to connect the spi_table
various fixes
'''

__OUT_OF_TREE_COMPILES__ 1st replace #include `<linux/usb/ft232h-intf.h>` with `#include "linux/usb/ft232h-intf.h"` in both `ft232h-intf.c` and `spi-ftdi-mpsse.c`. So model number is now detected ft4232h vs ft2232h and the amount of gpios is setup automatically according to version. This data is shared between the interface driver and the spi driver by exported symbols so not sure how well outa tree compiles will go, make sure your kernel doesnt have module versioning support enabled (MODVERSIONS) and worse come worse you can always manually add the symbols to the Module.symvers. and dont use insmod but cp it to your `lib/modules`. 
I highly recommend integrating the driver into source tree or using one my of kernels `github.com/bm16ton`. These changes are probly very simple easy things for smart people, but all my knowledge comes exclusively from looking at other code/examples/commits even the most basic of structures and wording can take me hour and hundreds of compiles. So things are bound to be wrong (IE gpio lookup tables for dc,reset,interrupts ignore naming of that last one was playing with polled gpio irq) but its working. Any help would be super greatly appreciated. Dumping even the raw hex of the eeprom via the new sysfs file is gonna be a mountain for me, so please. 

PLATFORM DATA; Uhg so diff devices seem to want/need diff ways of providing platform data? At least some drivers dont work with same methods others do like fbtft fb_ili9341 will not take anything gpios, setup data etc, but tinydrm ili9341 takes right off with the dc reset pins. 5.15 has not helped this situation, devices/drivers load no errors, but nothing populated in /dev IE spidev no longer works with modalias spidev I just picked a random off the list "spi-petra" and it enumerates. If a driver doesnt work look at its spi_device_id list and choose from it. If it doesnt have a spi_device_id list most likely thats why it aint working so create one. 

FBTFT FB_ILI9341; As mentioned no idea how to pass the platform data to this driver (without hard coding the path to it main headerfile) So I just up-ported ?that the right word? the old fbtft folder with fbtft_device (a kernel module that just loads the required data to the various fbtft modules) and it works slick. I use the following script to load it, everything is auto detected except the spi bus num that u need to edit manually.

```sh
#!/bin/bash
sudo modprobe fbtft_device custom name=fb_ili9341 \
busnum=0 gpios=dc:$(sudo cat /sys/kernel/debug/gpio \
| grep -i dc | awk '{print $1}' | sed -e 's/gpio-//g'),reset:$(sudo cat /sys/kernel/debug/gpio | \
grep -i reset | awk '{print $1}' | sed -e 's/gpio-//g') speed=30000000 rotate=90 bgr=1 fps=60
```

### UPDATE: 
Added property support to the spi driver, and gpio naming. Currently pin numbers (amount avail) are set for ft4232 will do an update for ft2232 someday (uhg) Also with the product (in ftdi eeprom) set to ft2232H-16ton or ft4232H-16ton, and the ftdi_sio patch applied, ftdi_sio will ignore the first 2 interfaces (on ft4232 or both interfaces on ft2232) and the spi driver will attach to the first interface and i2c on second (and ftdi_sio serial on interfaces 3/4 on ft4232 and shit outa luck on serial for ft2232 in this setup) In my x86 kernel source (tons of stuff for i2c spi etc in this kernel) I have brought back fb-device because god help me I cannot get the fb-ili9341 from fbtft to take the platform data it needs. Seemingly all options for platform data with fbtft have been removed except for devicetree...and I cannot fake it yet. I did have the tinydrm ili9341 driver taking the platform data correctly from driver and loading but now it just freezes my machines. I was playing with gpio-lookup instead of  ftdi_spi_bus_dev_io and maybe thats when it was working or kernel changes broke the setup (been a while in between testings with newer kernel versions in between) ill throw a script to make loading the fbtft fb-device fb_ili9341 stuff easier, itll auto detect the gpio pin numbers assigned to dc/reset by your machine but you will have to edit it for the spi bus number. 

my kernel source with these drivers plus lots more already added; https://github.com/bm16ton/linux-kernel

To integrate these into your own kernel source 
```sh
cd kernel-source

cp ../ft2232-mpsse-i2c-spi-kern-drivers/i2c-ftdi/mpsse.h drivers/i2c/busses/mpsse.h

cp ../ft2232-mpsse-i2c-spi-kern-drivers/i2c-ftdi/i2c-ftdi.c drivers/i2c/busses/i2c-ftdi.c
```

#### Edit Kconfig file located at `drivers/i2c/busses/Kconfig`

Example KCONFIG FOR i2c-ftdi;

```make
config I2C_FTDI
	tristate "ft2232h I2C master support"
	depends on USB
	help
	  Say yes here to access the I2C part of the ftdi
	  ft2232h, and probly others.
```

Example Makefile entry for i2c-ftdi
```make
obj-$(CONFIG_I2C_FTDI)	+= i2c-ftdi.o
````
for spi;
And follow with shell copy command
```sh
cp ../ft2232-mpsse-i2c-spi-kern-drivers/spi-ftdi/ft232h-intf.c drivers/usb/misc/
```
Example Kconfig for ft232h-intf; edit drivers/usb/misc/Kconfig;
```make
config USB_FT232H_INTF
	tristate "FTDI FT232H SPI and platform data configuration interface driver"
	help
	  Enable driver support for the FT232H/ft2232H based USB-GPIO/SPI/FIFO
	  interface adapter for SPI and FPGA configuration. Additional drivers
	  such as spi-ftdi-mpsse, etc must be enabled in order to use the
	  functionality of the device.

	 To compile this driver as a module, choose M here: the
	 module will be called ft232h_intf.
```
Example Makefile for ft232h-intf
```make
obj-$(CONFIG_USB_FT232H_INTF)		+= ft232h-intf.o
```
And follow with shell copy command
```
cp ../ft2232-mpsse-i2c-spi-kern-drivers/spi-ftdi/spi-ftdi-mpsse.c drivers/spi/
```

Example Kconfig; drivers/spi/Kconfig;
```
config SPI_FTDI_MPSSE
	tristate "FTDI MPSSE SPI controller"
	depends on USB_FT232H_INTF || COMPILE_TEST || GPIOLIB && SYSFS
 	select GPIO_SYSFS
 	select GPIOLIB_IRQCHIP
	help
	  FT232H supports SPI in MPSSE mode. This driver provides MPSSE
	  SPI controller in master mode.
```
Example Makefile; 
```make
obj-$(CONFIG_SPI_FTDI_MPSSE)		+= spi-ftdi-mpsse.o
```
And follow with shell copy command
```sh
cp ../ft2232-mpsse-i2c-spi-kern-drivers/spi-ftdi/ft232h-intf.h ./include/linux/usb/ft232h-intf.h
```
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
