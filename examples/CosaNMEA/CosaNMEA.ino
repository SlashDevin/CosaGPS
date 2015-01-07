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
#include "Streamers.h"

static NMEAGPS gps;

//--------------------------

void setup()
{
  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEAGPS: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("gps object size = ") << sizeof(gps) << endl;

  trace_header();

  uart.flush();
  
  // Start the UART for the GPS device
  uart1.begin(9600);
}

//--------------------------

void loop()
{
  while (uart1.available())
    if (gps.decode( uart1.getchar() ) == NMEAGPS::DECODE_COMPLETED) {
//      trace << (uint8_t) gps.nmeaMessage << ' ';

      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC) {
        trace_all( gps, gps.fix() );

        //  Use received GPRMC sentence as a pulse
        seconds++;
      }
    }

  Power::sleep();
}
