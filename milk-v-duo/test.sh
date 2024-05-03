#!/bin/sh

# pinmux

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
