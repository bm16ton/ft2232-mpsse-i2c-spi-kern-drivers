These are linux drivers for ftdi mpsse devices i2c spi.

the spi driver originaly came from;

 http://git-old.denx.de/?p=linux-denx/linux-denx-agust.git

 only slight edits where needed. Add your spi device module alias and cs

 pin info to line 1203 of ft232h-intf.c , then recompile, i use in tree but

 will write a Make file someday. For in tree cp ft232h-intf.c to drivers/usb/misc

 cp ft232h-int.h to include/linux/usb/ and cp spi-ftdi-mpsse.c to

 drivers/spi/ the eit the Kconfig and Makefiles chechout my kernel for examples

 https://github.com/bm16ton/yoga-c630-linux-kernel

 The i2c came from https://github.com/krinkinmu/bootlin also checkout his

 awesome writeup on reverse engineering mpsse;

 https://krinkinmu.github.io/2020/08/02/ftdi.html

 https://krinkinmu.github.io/2020/09/05/ftdi-protocol.html

 https://krinkinmu.github.io/2020/09/06/ftdi-i2c.html

 This driver I had to change more on to get it to work, the control usb

 packets, Added smbus support, all reletivly minor.

 These have been tested on ft2232h, kernel 5.12,and on mpsse bank A only.

 Even tho these will bind to both banks doubtful bank B will work in

 current configuration.

 SPI TODO, write makefile (ez enuff) find better way to supply spi device

 platformdata (mod params? add info to eeprom?) maybe do a dirty hack for irq pin.

 I@C TODO, Add sysfs entry for changing bus speed, was more but dont remember atm.

 TODO BETTER README! Its late uhm or early now and just got it workng, ill

 update more later.




