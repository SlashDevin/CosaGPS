/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

#include "NMEAGPS.h"

#if !defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
static uint32_t seconds = 0L;
#endif

static NMEAGPS gps;

//--------------------------

static void sentenceReceived()
{
#if !defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
  trace << seconds << ',';
#endif

  trace << gps.fix() <<'\n';

} // sentenceReceived

//--------------------------

void setup()
{
  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEAGPS: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("gps object size = ") << sizeof(gps) << endl;
  
  // Start the UART for the GPS device
  uart1.begin(9600);
}

//--------------------------

void loop()
{
  while (uart1.available())
    if (gps.decode( uart1.getchar() ) == NMEAGPS::DECODE_COMPLETED) {
//      trace << (uint8_t) gps.nmeaMessage << ' ';

// Make sure that the only sentence we care about is enabled
#ifndef NMEAGPS_PARSE_RMC
#error NMEAGPS_PARSE_RMC must be defined in NMEAGPS.h!
#endif

      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC) {
        sentenceReceived();
#if !defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
        //  No date/time fields enabled, use received GPRMC sentence as a pulse
        seconds++;
#endif
      }
    }

  Power::sleep();
}
