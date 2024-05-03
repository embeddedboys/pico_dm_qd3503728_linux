<!--
 Copyright (c) 2024 embeddedboys(writeforever@foxmail.com)
 
 This software is released under the MIT License.
 https://opensource.org/licenses/MIT
-->

# Steps on milk-v duo

**1. setup duo-buildroot-sdk**
```bash
sudo apt install -y pkg-config build-essential ninja-build automake autoconf libtool wget curl git gcc libssl-dev bc slib squashfs-tools android-sdk-libsparse-utils jq python3-distutils scons parallel tree python3-dev python3-pip device-tree-compiler ssh cpio fakeroot libncurses5 flex bison libncurses5-dev genext2fs rsync unzip dosfstools mtools tcl openssh-client cmake expect
```

```bash
git clone https://github.com/milkv-duo/duo-buildroot-sdk.git --depth=1
```

run `build.sh` at lease once.
```bash
./build.sh lunch

# choose milkv-duo

./build.sh
```

**2. source sdk envs**
```bash
source device/milkv-duo/boardconfig.sh

source build/milkvsetup.sh
defconfig cv1800b_milkv_duo_sd
```

**3. build kernel dtb**
```bash
cd linux_5.10

cp /home/developer/embeddedboys/pico_dm_qd3503728_linux/milk-v-duo/cv1800b_milkv_duo_sd.dts \
    linux_5.10/arch/riscv/boot/dts/cvitek/

make dtbs
```

**4. flash the new dtb**
```bash
cd ramdisk/build/cv1800b_milkv_duo_sd/workspace
cp ~/sources/duo-buildroot-sdk/linux_5.10/arch/riscv/boot/dts/cvitek/cv1800b_milkv_duo_sd.dtb .
cp ~/sources/duo-buildroot-sdk/linux_5.10/arch/riscv/boot/Image .

lzma -k Image
mkimage -f multi.its boot.sd

# at device side start
mkdir -p /boot
mount /dev/mmcblk0p1 /boot/
# at device side end

scp boot.sd root@192.168.42.1:/boot/
```

Then reboot the duo to apply the new kernel and dtb.

**5. build and insmod and test**
```bash
cd pico_dm_qd3503728_linux
make -f Makefile.milk-v-duo

scp milk-v-duo/test.sh root@192.168.42.1:~/
scp ili9488_fb.ko root@192.168.42.1:~/

# at device side start
./test.sh
# at device side end
```

The content of `test.sh`
```bash
#!/bin/sh

duo-pinmux -w GP0/GP0 > /dev/null 2>&1
duo-pinmux -w GP1/GP1 > /dev/null 2>&1
duo-pinmux -w GP2/GP2 > /dev/null 2>&1
duo-pinmux -w GP3/GP3 > /dev/null 2>&1
duo-pinmux -w GP4/GP4 > /dev/null 2>&1
duo-pinmux -w GP5/GP5 > /dev/null 2>&1
duo-pinmux -w GP6/GP6 > /dev/null 2>&1
duo-pinmux -w GP7/GP7 > /dev/null 2>&1
duo-pinmux -w GP8/GP8 > /dev/null 2>&1
duo-pinmux -w GP9/GP9 > /dev/null 2>&1
duo-pinmux -w GP10/GP10 > /dev/null 2>&1
duo-pinmux -w GP11/GP11 > /dev/null 2>&1
duo-pinmux -w GP12/GP12 > /dev/null 2>&1
duo-pinmux -w GP13/GP13 > /dev/null 2>&1
duo-pinmux -w GP14/GP14 > /dev/null 2>&1
duo-pinmux -w GP15/GP15 > /dev/null 2>&1

rmmod ili9488_fb.ko
insmod ili9488_fb.ko
```

**6. build & run lvgl demo**

Create a musl libc symbol link
```bash
cd /lib
ln -sf ../usr/lib64v0p7_xthead/lp64d/libc.so ./ld-musl-r
iscv64xthead.so.1
```

```bash
git clone https://github.com/lvgl/lv_port_linux_frame_buffer.git

cd lv_port_linux_frame_buffer

# disp_drv.hor_res    = 480;
# disp_drv.ver_res    = 320;

make CC=/home/developer/sources/duo-buildroot-sdk/host-tools/gcc/riscv64-linux-musl-x86_64/bin/riscv64-unknown-linux-musl-gcc -j20

scp demo root@192.168.42.1:~/

# at device side start
./demo
# at device side end
```

The lvgl demo should run instantly on the screen.

**7. benmark, performance upgrade**

TODO
