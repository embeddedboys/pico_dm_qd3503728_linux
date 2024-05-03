<!--
 Copyright (c) 2024 embeddedboys(writeforever@foxmail.com)
 
 This software is released under the MIT License.
 https://opensource.org/licenses/MIT
-->

# Steps

1. setup duo-buildroot-sdk

2. build sdk manually
```bash
```

3. build kernel dtb

4. flash the new dtb

5. insmod and test

6. build & run lvgl demo
```bash
cd /lib
ln -sf ../usr/lib64v0p7_xthead/lp64d/libc.so ./ld-musl-r
iscv64xthead.so.1
```