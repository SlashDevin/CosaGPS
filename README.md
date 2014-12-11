NeoGPS
======

NeoGPS is a generic GPS parsing library for the Arduino [Cosa](https://github.com/mikaelpatel/Cosa) environment.
The following protocols are supported:
* NMEA 0183
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
* optional floating point
* configurable message sets, including hooks for implementing proprietary NMEA messages
* configurable message fields

Data Model
==========
Rather than holding onto individual fields, the concept of a **fix** is used to group data members of the GPS acquisition.
This also facilitates the merging of separately received packets into a coherent position.  The members of `gps_fix` include 
* fix status
* time
* latitude and longitude
* altitude
* speed
* heading
* number of satellites
* horizontal dilution of precision (HDOP)

Except for `status`, each member is conditionally compiled; any, all, or *no* members can be selected for parsing, storing and fusing.  This allows configuring an application to use the minimum amount of RAM for the particular `fix` members of interest.
There is a separate validity flag for each of those members.
Integers are used for all members, retaining full precision of the original data.   
Optional floating-point accessors are provided.
`fix` operators are defined which allow the developer to merge two fixes:
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
As data is received from the device, various portions of a `fix` are modified.  In 
fact, no buffering RAM is required.  
Each character affects the internal state machine and may also contribute to a data 
member (e.g., latitude).
A fully-configured `fix` requires only 32 bytes, and the NMEA state machine requires 
7 bytes, for a total of **39 bytes**.  The minimally-configured `fix` requires only 
2 bytes, for a total of only **10 bytes** (structure alignment may add 1 byte).

For example, if your application only requires an accurate one pulse-per-second, you 
can configure it to parse *no* sentence types and retain *no* data members.  Even 
though no sentences are parsed and no data members are stored, the application will 
still receive a `decoded` message type once per second:
```
while (uart1.available())
  if (gps.decode( uart1.getchar() )) {
    if (gps.nmeaMessage == NMEAGPS::NMEA_RMC)
      sentenceReceived();
  }
```
Although the `fix().status` can be checked, no valid flags are available.

The `ubloxGPS` derived class adds 18 bytes to handle the more-complicated protocol, 
plus 5 static bytes for converting GPS time and Time Of Week to UTC, for a total of 
**57 bytes**.

Examples
======
Several programs are provided to demonstrate how to use the classes in these different styles:

* [CosaNMEAGPS](CosaNMEAGPS.ino) - sync, polled, not fused, standard NMEA only
* [CosaGPSDevice](CosaGPSDevice.ino) - async, polled, fused, standard NMEA only
* [CosaGPSEvent](CosaGPSEvent.ino) - async, event, fused, standard NMEA only
* [CosaUBXGPS](CosaUBXGPS.ino) - sync, polled, fused, standard NMEA + ublox proprietary NMEA + ublox protocol

Preprocessor symbol `USE_FLOAT` can be used to select integer or floating-point output.

`CosaGPSTest.ino is a self-test program.  Various strings are passed to `decode` and the expected pass or fail results are displayed.