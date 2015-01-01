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

static gps_fix fused;

//--------------------------

static void traceIt()
{
#if !defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
  //  Date/Time not enabled, just output the interval number
  trace << seconds << ',';
#endif

  trace << fused;

#ifdef NMEAGPS_PARSE_SATELLITES
  trace << ',' << '[';
  for (uint8_t i=0; i < fused.satellites; i++) {
    trace << gps.satellites[i].id;
#ifdef NMEAGPS_PARSE_GSV
    trace << ' ' << 
      gps.satellites[i].elevation << '/' << gps.satellites[i].azimuth;
    trace << '@';
    if (gps.satellites[i].tracked)
      trace << gps.satellites[i].snr;
    else
      trace << '-';
#endif
    trace << ',';
  }
  trace << ']';
#endif

  trace << '\n';

} // traceIt

//--------------------------

static void sentenceReceived()
{
  // See if we stepped into a different time interval,
  //   or if it has finally become valid after a cold start.

  bool newInterval;
#if defined(GPS_FIX_TIME)
  newInterval = (gps.fix().valid.time &&
                (!fused.valid.time ||
                 (fused.dateTime.seconds != gps.fix().dateTime.seconds) ||
                 (fused.dateTime.minutes != gps.fix().dateTime.minutes) ||
                 (fused.dateTime.hours   != gps.fix().dateTime.hours)));
#elif defined(GPS_FIX_DATE)
  newInterval = (gps.fix().valid.date &&
                (!fused.valid.date ||
                 (fused.dateTime.date  != gps.fix().dateTime.date) ||
                 (fused.dateTime.month != gps.fix().dateTime.month) ||
                 (fused.dateTime.year  != gps.fix().dateTime.year)));
#else
  //  No date/time configured, so let's assume it's a new interval
  //  if it has been a while since the last sentence was received.
  static uint32_t last_sentence = 0L;
  
  newInterval = (seconds != last_sentence);
  last_sentence = seconds;
#endif
//trace << PSTR("ps mvd ") << fused.valid.date << PSTR("/") << gps.fix().valid.date;
//trace << PSTR(", mvt ") << fused.valid.time << PSTR("/") << gps.fix().valid.time;
//trace << fused.dateTime << PSTR("/") << gps.fix().dateTime;
//trace.print( F("ni = ") ); trace << newInterval << '\n';
//trace << 'v' << gps.fix().valid.as_byte << '\n';

  if (newInterval) {

    // Log the previous interval
    traceIt();

    //  Since we're into the next time interval, we throw away
    //     all of the previous fix and start with what we
    //     just received.
    fused = gps.fix();

    gps.poll( &uart1, NMEAGPS::NMEA_GST );

  } else {
    // Accumulate all the reports in this time interval
    fused |= gps.fix();
  }

} // sentenceReceived

//--------------------------

void setup()
{
  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEAfused: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("gps object size = ") << sizeof(gps) << endl;
  
  // Start the UART for the GPS device
  uart1.begin(9600);
  
  gps.poll( &uart1, NMEAGPS::NMEA_GST );
}

//--------------------------

void loop()
{
  while (uart1.available())
    if (gps.decode( uart1.getchar() ) == NMEAGPS::DECODE_COMPLETED) {
//      trace << (uint8_t) gps.nmeaMessage << ' ';

      sentenceReceived();

#if !defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)

// Make sure that the only sentence we care about is enabled
#ifndef NMEAGPS_PARSE_RMC
#error NMEAGPS_PARSE_RMC must be defined in NMEAGPS.h!
#endif
      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC)
        //  No date/time fields enabled, use received GPRMC sentence as a pulse
        seconds++;
#endif

    }

  Power::sleep();
}
