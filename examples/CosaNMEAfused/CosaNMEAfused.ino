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

#if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
    !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
    !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
    !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

#if defined(GPS_FIX_DATE)| defined(GPS_FIX_TIME)
#error No NMEA sentences enabled: no fix data available for fusing.
#else
#warning No NMEA sentences enabled: no fix data available for fusing,\n\
 only pulse-per-second is available.
#endif

#endif

static NMEAGPS gps;

static gps_fix fused;

//--------------------------

void setup()
{
  RTC::begin();

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEAfused: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("gps object size = ") << sizeof(gps) << endl;
  trace_header();

  uart.flush();
  
  // Start the UART for the GPS device
  uart1.begin(9600);
  
  gps.poll( &uart1, NMEAGPS::NMEA_GST );
}

//--------------------------

void loop()
{
  static uint32_t last_rx = 0L;

  while (uart1.available()) {
    last_rx = RTC::millis();

    if (gps.decode( uart1.getchar() ) == NMEAGPS::DECODE_COMPLETED) {

      // All enabled sentence types will be merged into one fix
      fused |= gps.fix();

      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC)
        //  Use received GPRMC sentence as a pulse
        seconds++;
    }
  }

  // Print things out once per second, after the serial input has died down.
  // This prevents input buffer overflow during printing.

  static uint32_t last_trace = 0L;

  if ((last_trace != seconds) && (RTC::millis() - last_rx > 5)) {
    last_trace = seconds;

    // It's been 5ms since we received anything, log what we have so far...
    trace_all( gps, fused );
  }

  Power::sleep();
}
