
# local kernel build dir
# KERN_DIR ?= /lib/modules/$(shell uname -r)/build

# users kernel dir
ARCH := arm
CROSS_COMPILE := ${HOME}/luckfox/lyra/prebuilts/gcc/linux-x86/arm/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-
KERN_DIR := ${HOME}/luckfox/lyra/kernel

MODULE_NAME = ili9488_fb

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=`pwd` modules
	$(CROSS_COMPILE)gcc tests/test_fb.c -o tests/test_fb
clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=`pwd` modules clean

clena: clean

#CFLAGS_$(MODULE_NAME).o := -DDEBUG
obj-m += ili9488_fb.o
