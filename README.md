A GPIO based Linux FB driver for Pico_DM_QD3503728.

This driver is based on fbtft.

## FIXME

- [ ] The wrong color appeared when the fb console was displayed for the first time.

## TODO list
- [x] Luckfox Pico
- [x] Milk-V Duo

## Device list

### Luckfox Pico

### Luckfox Pico Max

### Luckfox Lyra

#### Booting from SDcard(eMMC)

Load the luckfox lyra sdmmc config first.
```shell
./build.sh lunch

1. luckfox_lyra_buildroot_sdmmc_defconfig
2. luckfox_lyra_buildroot_spinand_defconfig
3. luckfox_lyra_plus_buildroot_sdmmc_defconfig
4. luckfox_lyra_plus_buildroot_spinand_defconfig
5. luckfox_lyra_plus_ubuntu_sdmmc_defconfig
6. luckfox_lyra_ubuntu_sdmmc_defconfig
7. luckfox_lyra_ultra-w_buildroot_emmc_defconfig
8. luckfox_lyra_ultra-w_ubuntu_emmc_defconfig
9. luckfox_lyra_ultra_buildroot_emmc_defconfig
10. luckfox_lyra_ultra_ubuntu_emmc_defconfig

# then type 1 to select it
Which would you like? [1]: 1
```

Patch the dts and build the zboot.img before flashing.
```shell

cp luckfox-lyra/rk3506g-luckfox-lyra-sd.dts /home/developer/luckfox/lyra/kernel/arch/arm/boot/dts/rk3506g-luckfox-lyra-sd.dts

./build.sh kernel

adb push kernel-6.1/zboot.img /root
```

Come to the device side and run:
```shell
dd if=/root/zboot.img of=/dev/mmcblk0p2 bs=1M && reboot
```
this will flash the new kernel and dts and reboot the device.

After the device rebooted, you can build and push the driver to the device:
```shell
make -f Makefile.luckfox-lyra
adb push ili9488_fb.ko /root
adb shell "insmod /root/ili9488_fb.ko"
```