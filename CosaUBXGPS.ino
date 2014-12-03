/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Watchdog.hh"

#include "./ubxGPS.h"

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

static ubloxGPS gps( &uart1 );
static gps_fix merged;

//--------------------------

static void processSentence()
{
  // See if we stepped into a different time interval,
  //   or if it has finally become valid after a cold start.

  bool newInterval;
#if defined(GPS_FIX_TIME)
  newInterval = gps.fix().valid.time &&
                (!merged.valid.time ||
                 (merged.dateTime.seconds != gps.fix().dateTime.seconds) ||
                 (merged.dateTime.minutes != gps.fix().dateTime.minutes) ||
                 (merged.dateTime.hours   != gps.fix().dateTime.hours));
#elif defined(GPS_FIX_DATE)
  newInterval = gps.fix().valid.date &&
                (!merged.valid.date ||
                 (merged.dateTime.date  != gps.fix().dateTime.date) ||
                 (merged.dateTime.month != gps.fix().dateTime.month) ||
                 (merged.dateTime.year  != gps.fix().dateTime.year));
#endif
//trace << PSTR("ps mvd ") << merged.valid.date << PSTR("/") << gps.fix().valid.date;
//trace << PSTR(", mvt ") << merged.valid.time << PSTR("/") << gps.fix().valid.time;
//trace << merged.dateTime << PSTR("/") << gps.fix().dateTime;
//trace << PSTR(", ni = ") << newInterval << endl;
  if (newInterval) {

    // Log the previous interval
    traceIt();

#ifdef UBLOX_PARSE_NMEA
    NMEAGPS::send( &uart1, PSTR("PUBX,00") );
    NMEAGPS::send( &uart1, PSTR("PUBX,04") );
#endif

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

#define USE_FLOAT

static void traceIt()
{
#if defined(GPS_FIX_DATE) | defined(GPS_FIX_TIME)
  bool someTime = false;
#if defined(GPS_FIX_DATE)
  someTime |= merged.valid.date;
#endif
#if defined(GPS_FIX_TIME)
  someTime |= merged.valid.time;
#endif

  if (someTime) {
    trace << merged.dateTime << PSTR(".");
    if (merged.dateTime_cs < 10)
      trace << '0';
    trace << merged.dateTime_cs;
  }
  trace << PSTR(",");
#endif

#ifdef USE_FLOAT
  trace.width(3);
  trace.precision(6);
#ifdef GPS_FIX_LOCATION
  if (merged.valid.location)
    trace << merged.latitude() << PSTR(",") << merged.longitude();
  else
    trace << PSTR(",");
  trace << PSTR(",");
#endif
#ifdef GPS_FIX_HEADING
  trace.precision(2);
  if (merged.valid.heading)
    trace << merged.heading();
  trace << PSTR(",");
#endif
#ifdef GPS_FIX_SPEED
  trace.precision(3);
  if (merged.valid.speed)
    trace << merged.speed();
  trace << PSTR(",");
#endif
#ifdef GPS_FIX_ALTITUDE
  trace.precision(2);
  if (merged.valid.altitude)
    trace << merged.altitude();
  trace << PSTR(",");
#endif

#else

#ifdef GPS_FIX_LOCATION
  if (merged.valid.location)
    trace << merged.latitudeL() << PSTR(",") << merged.longitudeL();
  else
    trace << PSTR(",");
  trace << PSTR(",");
#endif
#ifdef GPS_FIX_HEADING
  if (merged.valid.heading)
    trace << merged.heading_cd();
  trace << PSTR(",");
#endif
#ifdef GPS_FIX_SPEED
  if (merged.valid.speed)
    trace << merged.speed_mkn();
  trace << PSTR(",");
#endif
#ifdef GPS_FIX_ALTITUDE
  if (merged.valid.altitude)
    trace << merged.altitude_cm();
  trace << PSTR(",");
#endif
#endif

#ifdef GPS_FIX_SATELLITES
  if (merged.valid.satellites)
    trace << merged.satellites;
  trace << PSTR(",");
#endif

#ifdef USE_FLOAT
  trace.width(5);
  trace.precision(3);
#ifdef GPS_FIX_HDOP
  if (merged.valid.hdop)
    trace << (merged.hdop * 0.001);
#endif

#else

#ifdef GPS_FIX_HDOP
  if (merged.valid.hdop)
    trace << merged.hdop;
#endif
#endif
  trace << endl;

} // traceIt

//--------------------------

void setup()
{
  // Watchdog for sleeping
  Watchdog::begin( 64, Watchdog::push_timeout_events );

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaUBXGPS: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("ubloxGPS object size = ") << sizeof(ubloxGPS) << endl;
  trace << PSTR("gps object size = ") << sizeof(gps) << endl;
  uart.flush();

  // Start the UART for the GPS device
  uart1.begin(9600);
}

//--------------------------

void loop()
{
  static uint8_t since = 16;

  if (since++ >= 16) {
    since = 0;
trace << PSTR("Hey!\n");
#ifdef UBLOX_PARSE_NMEA
    NMEAGPS::send( &uart1, PSTR("PUBX,00") );
    NMEAGPS::send( &uart1, PSTR("PUBX,04") );
#endif
  }
  Watchdog::await();

  while (uart1.available())
    if (gps.decode( uart1.getchar() )) {
//trace << gps.nmeaMessage;
      bool ok = true;
#ifdef UBLOX_PARSE_NMEA
      ok = (gps.nmeaMessage >= ubloxGPS::PUBX_00);
#endif
      if (ok) {
        processSentence();
        since = 0;
      }
    }
}
