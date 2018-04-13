CODE_DIR = ./code

obj-m += gpio_bmp085.o
gpio_bmp085-objs := ${CODE_DIR}/main.o

KDIR = /root/Kernel_headers/linux-headers-3.4.113-sun8i

# Make rules
all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean