/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/RTC.hh"
#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Watchdog.hh"

#include "./NMEAGPS.h"

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

static NMEAGPS gps( &uart1 );
static NMEAGPS::gps_fix_t merged;

static clock_t now = 0;
static clock_t lastTrace = 0;

//--------------------------

static void processSentence()
{
  // See if we stepped into a different time interval,
  //   or if it has finally become valid after a cold start.
  if (merged.valid.dateTime && gps.fix().valid.dateTime &&
      (merged.dateTime != gps.fix().dateTime)) {

    // Log the previous interval
    traceIt();

    //  Since we're into the next time interval, we throw away
    //     all of the previous fix and start with what we
    //     just received.
    merged = gps.fix();

  } else {
    // Accumulate all the reports in this time interval
    merged |= gps.fix();
  }
}

//--------------------------

//#define USE_FLOAT

static void traceIt()
{
  if (merged.valid.dateTime) {
    trace << merged.dateTime << PSTR(".");
    if (merged.dateTime_cs < 10)
      trace << '0';
    trace << merged.dateTime_cs;
  } else {
    //  Apparently we don't have a fix yet, ask for a ZDA (Zulu Date and Time)
    gps.poll( NMEAGPS::NMEA_ZDA );
  }
  trace << PSTR(",");

#ifdef USE_FLOAT
  trace.width(3);
  trace.precision(6);
  if (merged.valid.location)
    trace << merged.latitude() << PSTR(",") << merged.longitude();
  else
    trace << PSTR(",");
  trace << PSTR(",");
  trace.precision(2);
  if (merged.valid.heading)
    trace << merged.heading();
  trace << PSTR(",");
  trace.precision(3);
  if (merged.valid.speed)
    trace << merged.speed();
  trace << PSTR(",");
  trace.precision(2);
  if (merged.valid.altitude)
    trace << merged.altitude();
#else
  if (merged.valid.location)
    trace << merged.latitudeL() << PSTR(",") << merged.longitudeL();
  else
    trace << PSTR(",");
  trace << PSTR(",");
  if (merged.valid.heading)
    trace << merged.heading_cd();
  trace << PSTR(",");
  if (merged.valid.speed)
    trace << merged.speed_mkn();
  trace << PSTR(",");
  if (merged.valid.altitude)
    trace << merged.altitude_cm();
#endif
  trace << endl;

  lastTrace = now;

} // traceIt

//--------------------------

void setup()
{
  // Watchdog for sleeping
  Watchdog::begin( 16, Watchdog::push_timeout_events );
  RTC::begin();

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEAGPS: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("NMEAGPS object size = ") << sizeof(NMEAGPS) << endl;
  uart.flush();

  // Start the UART for the GPS device
  uart1.begin(9600);
}

//--------------------------

void loop()
{
  while (uart1.available())
    if (gps.decode( uart1.getchar() ))
      processSentence();

  now = RTC::time();

  if (lastTrace + 5 <= now)
    traceIt();
}
