Texas Instruments, Inc.

ZAP-MSP430 Release Notes

-------------------------------------------------------------------------------
-------------------------------------------------------------------------------

Version 2.5.1
April 11, 2012


Notices:
 - ZAP-MSP430 supports ZigBee Application Processor (ZAP) development on
   Texas Instruments EXP5438 and MSP2618 platforms. ZAP devices communicate
   with ZigBee Network Processor (ZNP) devices to form a "host + wireless
   modem" solution using ZigBee technology.

 - ZAP project files have been built and tested with IAR's EW430 version
   5.40.3 (5.40.3.50385) of the IAR Embedded Workbench toolset.

 - ZAP has been built and tested with IAR's CLIB library, which provides
   a light-weight C library which does not support Embedded C++. Use of
   DLIB is not recommended since Z-Stack is not tested with that library.

 - When programming a target for the first time release, make sure to
   select the "Erase main memory" option in the Debugger->FET Debugger
   category of IAR project options. When programming completes, select
   "Retain unchanged memory" so that NV items are retained during later
   re-programming of the device.


Changes:
 - The CC2530ZNP hex files in this release have been rebuilt to utilize
   improvements incorporated in the ZStack-CC2530-2.5.1 release.


Bug Fixes:
 - None.


Known Issues:
 - None.

-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
