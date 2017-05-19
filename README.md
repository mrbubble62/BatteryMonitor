# BatteryMonitor
## NMEA2000 DC Monitor

**High accuracy current and battery voltage monitoring with
2500V isolation from N2K bus**

Derived from and example by ttlappalainen
NMEA2000_CAN - NMEA2000 library for Arduino 
https://github.com/ttlappalainen/NMEA2000


###Parts list:

Teensy 3.1/3.2 board

[INA226AIDGSR 36-V, Bi-Directional, Ultra-High Accuracy, Low-/High-Side, I2C Out Current/Power Monitor](http://www.ti.com/product/INA226)

[ISO1050DUB  Isolated CAN Transceiver](http://www.ti.com/lit/ds/symlink/iso1050.pdf)

78L05 linear regulator for CAN output driver

7805C linear regulator for Teensy


###Notes:

12V > Polyswitch fuse > 7805C linear regulator > Teensy
3.3V power for INA226 and ISO1050 from Teensy
5V power ISO1050 CAN output driver from N2K bus

INA226 is tiny device use MSOP10 SMD to DIP10 Adapter PCB Board (ebay)



