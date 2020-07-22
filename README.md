# CAN-Logger

This project is all about logging/parsing/plotting FRC robot CAN-bus traffic.

## Hardware

Initial development was done using the $30 [Teensy 3.6](https://www.pjrc.com/teensy/)
along with the $16 [Dual CAN-bus Adapter](https://www.tindie.com/products/Fusion/dual-can-bus-adapter-for-teensy-35-36/).
The Dual CAN-bus Adapter needs to be soldered to the Teensy using [header pins](https://www.pjrc.com/store/header_20x1.html).
Also, a [micro-SD](https://www.amazon.com/SanDisk-Ultra-microSDXC-Memory-Adapter/dp/B073K14CVB/ref=asc_df_B073K14CVB/?tag=hyprod-20&linkCode=df0&hvadid=309776868400&hvpos=&hvnetw=g&hvrand=12376106667856342225&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9019575&hvtargid=pla-348080513499&language=en_US&th=1)
will be needed for log file storage. Finally, a [USB cable](https://www.pjrc.com/store/cable_usb_micro_b.html)
is needed to load firmware onto the Teensy.

## Firmware

Pre-built Teensy [images](https://github.com/ejmccalla/CAN-Logger/tree/master/src/hardware/Teensy/images) are provided for hardware which has been tested.  In order to generate images for other Teensy devices, follow the [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) instructions.

## FRC Robot

To use the Java CANLogger class, which allows robot code to interface with the CANLogger hardware, add the folling to the robot projects build.gradle file:

* repositories {
    jcenter()
    maven { url "https://jitpack.io" }
}

Also, add the following line to the end of the dependencies block in the build.gradle file:

* compile 'com.github.ejmccalla:CAN-Logger:v1.0'

Import the class by adding the following to your robot code:

* import org.BotsnBrews.CANLogger.CANLogger;

This class has 2 methods: a method which will start recording CAN-bus traffic and a method which stops recording.  First, create a CANLogger object by doing the following:

* CANLogger mCANLogger = new CANLogger();

Now, both methods can be run with the following:

* mCANLogger.StartLogging();
* mCANLogger.StopLogging();

The logging can be started/stopped multiple times per power cycle.  Each start command will create a new file on the micro-SD card with a name which follows the "YYYY-MM-DD:HH-MM-SS".bin naming convention.
