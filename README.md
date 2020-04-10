Features
======

An USB to JTAG Converter firmware running on CH552T, with some Gowin Quirks.

Issues
--------------

Serial port is working but with some limitation. Because the CPU is running at 16MHz, generate 115200 baudrate is not possible. The fastest standard baudrate is 57600, you can use 125000 and up to 1Mbps baudrate.

Pin layout
--------------

P1.0 - TMS

P1.4 & P1.7 - TCK

P1.5 - TDI

P1.6 - TDO

P3.0 - RXD

P3.1 - TXD


Boards
--------------

Sipeed Lichee Tang Nano.

Build
--------------

It requires make, sdcc and binutils.

If you got all of them, enter the src directory, and type "make" to generate binary file.

Programming
--------------

By WinchipHead's official WCHISPTOOL or LibreCH551 from rgwan. New chip with bootloader version > 2.30 may not working with librech551, it needs some fix.

License
--------------

MIT

Authors
--------------

Kongou Hikari <kongouhikari@qq.com>


