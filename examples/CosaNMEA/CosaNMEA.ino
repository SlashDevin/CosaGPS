/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"
#include "Cosa/RTC.hh"

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

#include "NMEAGPS.h"
#include "Streamers.h"

static NMEAGPS gps;

//--------------------------

void setup()
{
  RTC::begin();

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
  static uint32_t last_rx = 0L;
  static gps_fix rmc_data;

  while (uart1.available()) {
    last_rx = RTC::millis();

    if (gps.decode( uart1.getchar() ) == NMEAGPS::DECODE_COMPLETED) {

//#ifdef NMEAGPS_SAVE_TALKER_ID
//trace << gps.talker_id[0] << gps.talker_id[1] << ' ';
//#endif
//trace << (uint8_t) gps.nmeaMessage << ' ';

// Make sure that the only sentence we care about is enabled
#ifndef NMEAGPS_PARSE_RMC
#error NMEAGPS_PARSE_RMC must be defined in NMEAGPS.h!
#endif

      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC) {
        rmc_data = gps.fix(); // copied for printing later...
        
        //  Use received GPRMC sentence as a pulse
        seconds++;
      }
    }
  }

  // Print things out once per second, after the serial input has died down.
  // This prevents input buffer overflow during printing.

  static uint32_t last_trace = 0L;

  if ((last_trace != seconds) && (RTC::millis() - last_rx > 5)) {
    last_trace = seconds;

    // It's been 5ms since we received anything, log what we have so far...
    trace_all( gps, rmc_data );
  }

  Power::sleep();
}
