/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "./NMEAGPS.h"
//#define USE_FLOAT

#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

static NMEAGPS gps;

//--------------------------

static void sentenceReceived()
{
  if (gps.fix().valid.status)
    trace << gps.fix().status;
  trace << ',';

#if defined(GPS_FIX_DATE) | defined(GPS_FIX_TIME)
  bool someTime = false;
#if defined(GPS_FIX_DATE)
  someTime |= gps.fix().valid.date;
#endif
#if defined(GPS_FIX_TIME)
  someTime |= gps.fix().valid.time;
#endif

  if (someTime) {
    trace << gps.fix().dateTime << PSTR(".");
    if (gps.fix().dateTime_cs < 10)
      trace << '0';
    trace << gps.fix().dateTime_cs;
  }
#else
  static clock_t now = 0L;
  trace << now++;
#endif
  trace << ',';

#ifdef USE_FLOAT
  trace.width(3);
  trace.precision(6);
#ifdef GPS_FIX_LOCATION
  if (gps.fix().valid.location)
    trace << gps.fix().latitude() << ',' << gps.fix().longitude();
  else
    trace << ',';
  trace << ',';
#endif
#ifdef GPS_FIX_HEADING
  trace.precision(2);
  if (gps.fix().valid.heading)
    trace << gps.fix().heading();
  trace << ',';
#endif
#ifdef GPS_FIX_SPEED
  trace.precision(3);
  if (gps.fix().valid.speed)
    trace << gps.fix().speed();
  trace << ',';
#endif
#ifdef GPS_FIX_ALTITUDE
  trace.precision(2);
  if (gps.fix().valid.altitude)
    trace << gps.fix().altitude();
  trace << ',';
#endif

#else

#ifdef GPS_FIX_LOCATION
  if (gps.fix().valid.location)
    trace << gps.fix().latitudeL() << ',' << gps.fix().longitudeL();
  else
    trace << ',';
  trace << ',';
#endif
#ifdef GPS_FIX_HEADING
  if (gps.fix().valid.heading)
    trace << gps.fix().heading_cd();
  trace << ',';
#endif
#ifdef GPS_FIX_SPEED
  if (gps.fix().valid.speed)
    trace << gps.fix().speed_mkn();
  trace << ',';
#endif
#ifdef GPS_FIX_ALTITUDE
  if (gps.fix().valid.altitude)
    trace << gps.fix().altitude_cm();
  trace << ',';
#endif
#endif

#ifdef GPS_FIX_SATELLITES
  if (gps.fix().valid.satellites)
    trace << gps.fix().satellites;
  trace << ',';
#endif

#ifdef USE_FLOAT
  trace.width(5);
  trace.precision(3);
#ifdef GPS_FIX_HDOP
  if (gps.fix().valid.hdop)
    trace << (gps.fix().hdop * 0.001);
#endif

#else

#ifdef GPS_FIX_HDOP
  if (gps.fix().valid.hdop)
    trace << gps.fix().hdop;
#endif
#endif

  trace << endl;

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
      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC)
        sentenceReceived();
    }

  Power::sleep();
}
