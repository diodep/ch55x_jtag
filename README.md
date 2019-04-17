Features
======

An USB to Serial Converter firmware running on CH552T.

It also can automatically detect ESP32 & K210 bootloader message, force it enter ISP mode without help from FLOW CONTROL PINs (DTR/RTS).

Issues
--------------

Baudrate range: 1200bps - 1.5Mbps

USB endpoints are not configured as ping-pong mode, it makes it even slower. The real performance of this dongle is about 978Kbps, measured by esptool.

This usb to serial converter does not contains any hardware flow control featire and it can only works with 8N1 line coding.

Warning: High baudrate is not accurate at all, 115200 is the last traditional baudrate that it supports. For higher baudrate, please use 250Kbps, 500Kbps, 750Kbps.

So, my advice is... Don't use this puppy to make USB to Serial Bridge? CH330, CH340, HT42B534 and GD32F150/STM32F103're better choice than CH55x series MCU. Optimizing this code (by rewritten it in assembly) is just like real hell working.


Boards
--------------

Sipeed Maixduino

Build
--------------

It requires make, sdcc and binutils.

If you got all of them, enter the src directory, and type "make" to generate binary file.

Programming
--------------

By WinchipHead's official WCHISPTOOL or LibreCH551 from rgwan.

License
--------------

MIT

Authors
--------------

Kongou Hikari <kongouhikari@qq.com>


