# CAN-Logger

This project is all about logging/parsing/plotting FRC robot CAN-bus traffic.

## Hardware

Initial development was done using the $30 [Teensy 3.6](https://www.pjrc.com/teensy/)
along with the $16 [Dual CAN-bus Adapter](https://www.tindie.com/products/Fusion/dual-can-bus-adapter-for-teensy-35-36/).
The Dual CAN-bus Adapter needs to be soldered to the Teensy using [header pins](https://www.pjrc.com/store/header_20x1.html).Also, a [micro-SD](https://www.amazon.com/SanDisk-Ultra-microSDXC-Memory-Adapter/dp/B073K14CVB/ref=asc_df_B073K14CVB/?tag=hyprod-20&linkCode=df0&hvadid=309776868400&hvpos=&hvnetw=g&hvrand=12376106667856342225&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9019575&hvtargid=pla-348080513499&language=en_US&th=1)
will be needed for log file storage and a [USB cable](https://www.pjrc.com/store/cable_usb_micro_b.html)
is needed to load firmware onto the Teensy.
