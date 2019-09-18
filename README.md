Features
======

An USB to JTAG Converter firmware running on CH552T, with some Gowin Quirks.

Issues
--------------

Serial port is not working now.

Pin layout
--------------

P1.0 - TMS

P1.4 & P1.7 - TCK

P1.5 - TDI

P1.6 - TDO


Boards
--------------

Sipeed Lichee Tang Nano.

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


