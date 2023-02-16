
Driver for Linux RTL8188GU (RTL8710B) (VID:PID = 0x0BDA:0xB711)
===============================================================

Is not original driver from Realtek, is build from source code from others official Realtek drivers.

Compiling & Building
--------------------
### Dependencies
To compile the driver, you need to have make and a compiler installed. In addition,
you must have the kernel headers installed. If you do not understand what this means,
consult your distro.
### Compiling

> make

### Installing

> sudo make install

### Disable CDROM mode and select in WiFi mode. (not for Ubuntu 20.04)

> eject /dev/cdrom0

### Known problems

1. Very slow upload speed.
2. Problem reconnect from KNetworkManager in Kubuntu 20.04.
3. In Ubuntu 20.04 detected as GSM modem, need remove option driver as "sudo rmmod option".
4. In OrangePI debian 11 will get error: 'modpost: module 8723ds uses symbol kernel_write from namespace VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver, but does not import it.' import it in os_dep/linux/os_intfs.c to fix
```
sed -i '31i MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);' os_dep/linux/os_intfs.c
```

### Testing
I tested on Ubuntu 16.04, 20.04 and last version OpenWRT, it's work...
I tested ARM64 on Orange PI 5(Debian 11), ARM on Raspberry PI 2( Debian 11), it's work...