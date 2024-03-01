ifneq ($(KERNELRELEASE),)

obj-m := i2c-ftdi.o

else

KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

endif

.PHONY: clean

clean:
		rm -f modules.order Module.symvers *.o *.ko *.mod.* *.mod .*.cmd *.d
