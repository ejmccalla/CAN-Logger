# CAN-Logger

This project is all about logging/parsing/plotting FRC robot CAN traffic.

## Project Goals and Purpse

The primary goal of this project is to make CAN-bus logging simple and robust.
The logger should be as simple as loading firmware onto the device, connecting,
it to the CAN-bus, and adding a few lines of code to the robot project.  The
only interaction with the CAN logger once it has been setup will be to remove
the log files for data processing.

## Hardware

The $30 [Teensy 3.6](https://www.pjrc.com/teensy/) was choosen along with a $16
[Dual CAN-bus Adapter] (https://www.tindie.com/products/Fusion/dual-can-bus-adapter-for-teensy-35-36/).
Only a single CAN-bus is needed for this project, but the second bus could be
used for future expansion.  The Dual CAN-bus Adapter will need to be soldered to
the Teensy using [header pins](https://www.pjrc.com/store/header_20x1.html).
Also, a micro-SD will be needed for log file storage and a
[USB cable](https://www.pjrc.com/store/cable_usb_micro_b.html) is needed to load
sketches (programs) onto the Teensy.

## Firmware

The firmware that gets loaded onto the Teensy is stored in the src/logger/
directory.  The sole purpose of the firmware is to grab CAN frames off of the
bus and store them onto the SD card.  To do this, the
[FlexCAN library](https://github.com/collin80/FlexCAN_Library) will be used to
capture frames from the CAN-bus and the
[SDFat Library](https://github.com/greiman/SdFat) will be used to write the
captured data to the micro SD card.  In order to manage the data being captured
by the FLEXCan libraray and written by the SDFat library, the 
[CHIBIOS-Arduino library](https://github.com/greiman/ChibiOS-Arduino) is used.
This library provides the tools necessay to safely work with the shared memory
space.

## Software

The software for log file decoding is stored in the src/decoder/ directory.
There are three classes which can be used by an application to decode the log
file.  The `MotorControllerCANStatus` class is used to decode status frames for
the CTRE Talon SRX / Victor SPX.  The `PneumaticsControllerCANStatus` class is
used to decode status frames from the CTRE Pneumatics Conrol Module.  Finally,
the `PowerDistributionCANStatus` class is used to decode the status frames from
the CTRE Power Distribution Panel.
