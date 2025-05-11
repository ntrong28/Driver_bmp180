obj-m += bmp180_ioctl.o
KDIR = /lib/modules/$(shell uname -r)/build

all:
	make -C $(KDIR) M=$(shell pwd) modules
clean: 
	make -C $(KDIR) M=$(shell pwd) clean	
unload:
	sudo rmmod bmp280_i2c
install:
	sudo insmod bmp180_ioctl.ko
uninstall:
	sudo rmmod bmp180_ioctl
gcc:
	gcc bmp180_run.c -o run -lm