
Driver for Linux RTL8188GU (RTL8710B) (VID:PID = 0x0BDA:0xB711)
===============================================================

Is not original driver from Realtek, is build from source code from others official Realtek drivers.

Compiling & Building
--------------------
### Dependencies
To compile the driver, you need to have make and a compiler installed. In addition,
you must have the kernel headers installed. If you do not understand what this means,
consult your distro.

## Automatic (DKMS = automatically rebuild after kernel upgrade)
### Install

> sudo dkms add ./rtl8188gu

> sudo dkms build rtl8188gu/1.0

> sudo dkms install rtl8188fu/1.0

### Uninstall

> sudo dkms uninstall rtl8188fu/1.0

## Manual
### Install

> sudo make

> sudo make install

### Uninstall

> sudo modprobe -r rtl8188gu

> sudo make uninstall

## Load new wifi driver

> sudo modprobe rtl8188gu

## Disable CDROM mode and select in WiFi mode. (not for Ubuntu 20.04)

> eject /dev/cdrom0

## If your device is recognized as a CD

> lsusb

Write down your XXXX:YYYY adress.  (In my case, is 0bda:1a2b)

> sudo usb_modeswitch -KW -v XXXX -p YYYY



===============================================================



### Known problems

1. Very slow upload speed.
2. Problem reconnect from KNetworkManager in Kubuntu 20.04.
3. In Ubuntu 20.04 detected as GSM modem, need remove option driver as "sudo rmmod option".

### Testing
I tested on Ubuntu 16.04, 20.04 and last version OpenWRT, it's work...
