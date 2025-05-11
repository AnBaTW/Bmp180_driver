obj-m += bmp180_driver.o
KDIR = /lib/modules/$(shell uname -r)/build

all: bmp180_driver.ko
	make -C $(KDIR) M=$(shell pwd) modules
	sudo insmod bmp180_driver.ko
clean: bmp180_driver.ko
	make -C $(KDIR) M=$(shell pwd) clean
	sudo rmmod bmp180_driver.ko
