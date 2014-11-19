NeoGPS
======

NeoGPS is a generic GPS parsing library for the Arduino [Cosa](https://github.com/mikaelpatel/Cosa) environment.
The following protocols are supported:
* Basic NMEA 0183
* u-blox NEO-6

Goals
======
In an attempt to be reusable in a variety of different programming styles, this library supports:
* sync or async operation (main loop() vs interrupt)
* event or polling (Event::Handler vs. fix() call)
* fused or not fused (multiple reports into one fix)
* additional protocols from same device
* buffering of fixes
* ATTINY environments with very limited RAM and program space
* floating point optional

Data Model
==========
Rather than holding onto individual fields, the concept of a **fix** is used to group data members of the GPS acquisition.
This also facilitates the merging of separately received packets into a coherent position.  The members of `gps_fix_t` include 
* time
* latitude and longitude
* altitude
* speed
* heading
* fix status
* number of satellites
* horizontal dilution of precision (HDOP)
There is a separate validity flag for each of those members.  Integers are used for all
members, retaining full precision of the original data.  Optional floating-point accessors are provided.
Operators are defined which allow the developer to merge two fixes:
```
NMEAGPS gps_device;
gps_fix_t merged;
.
.
.
merged |= gps_device.fix();
```

RAM requirements
=======
As data is received from the device, various portions of a `fix` are set.  In fact, no buffering RAM is required.  
Each character affects the internal state machine and may also contribute to a member (e.g., latitude).
A complete `fix` requires only 31 bytes, and the NMEA state machine requires 7 bytes.

Examples
======
Several programs are provided to demonstrate how to use the classes in these different styles.
