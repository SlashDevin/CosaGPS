/*
  Use GPGGA and GPRMC sentences to test the parser's performance.
  uart is for trace output.
*/

#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Trace.hh"
#include "Cosa/RTC.hh"

#include "NMEAGPS.h"

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
#ifdef NMEAGPS_PARSE_GGA
  const char *gga =
    "$GPGGA,092725.00,4717.11399,N,00833.91590,E,"
    "1,8,1.01,499.6,M,48.0,M,,0*5B\r\n";
  const char *gga_no_lat =
    "$GPGGA,092725.00,,,00833.91590,E,"
    "1,8,1.01,499.6,M,48.0,M,,0*5B\r\n";

  trace << PSTR("GGA time = ") << time_it( gga ) << endl;
  trace << PSTR("GGA no lat time = ") << time_it( gga_no_lat ) << '\n';
#endif

  const char *rmc =
    "$GPRMC,083559.00,A,4717.11437,N,00833.91522,E,"
    "0.004,77.52,091202,,,A*57\r\n";

  trace << PSTR("RMC time = ") << time_it( rmc ) << endl;

#ifdef NMEAGPS_PARSE_GSV
  const char *gsv = 
    "$GPGSV,3,1,10,23,38,230,44,29,71,156,47,07,29,116,41,08,09,081,36*7F\r\n"
    "$GPGSV,3,2,10,10,07,189,,05,05,220,,09,34,274,42,18,25,309,44*72\r\n"
    "$GPGSV,3,3,10,26,82,187,47,28,43,056,46*77\r\n";
  trace << PSTR("GSV time = ") << time_it( gsv ) << '\n';
#endif

  for (;;);
}
