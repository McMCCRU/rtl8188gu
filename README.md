
Driver for Linux RTL8188GU (RTL8710B) (VID:PID = 0x0BDA:0xB711)
===============================================================

Is not original driver from Realtek, is build from source code from others official Realtek drivers.

Compiling & Building
--------------------
### Dependencies
To compile the driver, you need to have make and a compiler installed. In addition,
you must have the kernel headers installed. If you do not understand what this means,
consult your distro.

### Via DKMS

> sudo dkms install .

### Arch

> paru -S rtl8188gu-dkms-git

See [for Ubuntu and others](https://tutorialforlinux.com/2022/01/07/how-to-add-realtek-rtl8188gu-driver-ppa-for-ubuntu-based-systems/)

### Manually

#### Compiling

> make

#### Installing

> sudo make install

### Disable CDROM mode and select in WiFi mode. (not for Ubuntu 20.04)

> eject /dev/cdrom0

### Known problems

1. Very slow upload speed.
2. Problem reconnect from KNetworkManager in Kubuntu 20.04.
3. In Ubuntu 20.04 detected as GSM modem, need remove option driver as "sudo rmmod option".

### Testing
The original author [McMCCRU](https://github.com/McMCCRU) tested on Ubuntu 16.04, 20.04 and last version OpenWRT, it's work...
I have tested this on Arch, Manjaro 22, Ubuntu 21.10, Debian 11, Fedora 35, openSUSE 15.3.
