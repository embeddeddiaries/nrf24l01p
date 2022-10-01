obj-m += nrf24l01.o 
nrf24l01-objs := nrf24l01p_core.o nrf24l01p.o 
#To generate preprocessor output
#obj-m := bma400_core.i bma400_i2c.i bma400_spi.i

KERNEL_SRC := /home/jaggu/workspace/nRF24L01/rpi_kernel

SRC := $(shell pwd)
all:
	$(MAKE) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *.i *~ core .depend .*.cmd *.ko *.mod.c *.mod
	rm -f Module.makers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

