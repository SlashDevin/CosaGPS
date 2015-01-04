/*
  Use GPGGA and GPRMC sentences to test the parser's performance.
  uart is for trace output.
*/

#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Trace.hh"
#include "Cosa/RTC.hh"

#include "NMEAGPS.h"

/*
 * Make sure gpsfix.h and NMEAGPS.h are configured properly.
 */

#ifndef NMEAGPS_PARSE_GGA
#error NMEAGPS_PARSE_GGA must be defined in NMEAGPS.h!
#endif

#ifndef NMEAGPS_PARSE_RMC
#error NMEAGPS_PARSE_RMC must be defined in NMEAGPS.h!
#endif

static NMEAGPS gps;

//--------------------------

static uint32_t time_it( const char *data )
{
  const uint16_t ITERATIONS = 1024;
  uint32_t start, end;
  
  uart.flush();
  start = RTC::micros();
  for (uint16_t i=ITERATIONS; i > 0; i--) {
    char *ptr = (char *) data;
    while (*ptr)
      gps.decode( *ptr++ );
  }
  end = RTC::micros();

  return (end-start)/ITERATIONS;
}

//--------------------------

void setup()
{
  RTC::begin();

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEAbenchmark: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("NMEAGPS object size = ") << sizeof(NMEAGPS) << endl;
  uart.flush();
}

//--------------------------

void loop()
{
  const char *gga =
    "$GPGGA,092725.00,4717.11399,N,00833.91590,E,"
    "1,8,1.01,499.6,M,48.0,M,,0*5B\r\n";
  const char *rmc =
    "$GPRMC,083559.00,A,4717.11437,N,00833.91522,E,"
    "0.004,77.52,091202,,,A*57\r\n";

  trace << PSTR("GGA time = ") << time_it( gga ) << endl;
  trace << PSTR("RMC time = ") << time_it( rmc ) << endl;

  for (;;);
}
