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

#if defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
// uncomment this to display just one pulse-per-day.
//#define PULSE_PER_DAY
#endif

static uint32_t seconds = 0L;

static NMEAGPS gps;

static gps_fix fused;

//--------------------------

static void traceIt()
{
#if !defined(GPS_FIX_TIME) & !defined(PULSE_PER_DAY)
  //  Date/Time not enabled, just output the interval number
  trace << seconds << ',';
#endif

  trace << fused;

#if defined(NMEAGPS_PARSE_SATELLITES)
  if (fused.valid.satellites) {
    trace << ',' << '[';

    uint8_t i_max = fused.satellites;
    if (i_max > NMEAGPS::MAX_SATELLITES)
      i_max = NMEAGPS::MAX_SATELLITES;

    for (uint8_t i=0; i < i_max; i++) {
      trace << gps.satellites[i].id;
#if defined(NMEAGPS_PARSE_SATELLITE_INFO)
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
  }
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
#elif defined(PULSE_PER_DAY)
  newInterval = (gps.fix().valid.date &&
                (!fused.valid.date ||
                 (fused.dateTime.date  != gps.fix().dateTime.date) ||
                 (fused.dateTime.month != gps.fix().dateTime.month) ||
                 (fused.dateTime.year  != gps.fix().dateTime.year)));
#else
  //  No date/time configured, so let's assume it's a new interval
  //  if the seconds have changed.
  static uint32_t last_sentence = 0L;
  
  newInterval = (seconds != last_sentence);
  last_sentence = seconds;
#endif

  if (newInterval) {

    //  Since we're into the next time interval, we throw away
    //     all of the previous fix and start with what we
    //     just received.
    fused = gps.fix();

    gps.poll( &uart1, NMEAGPS::NMEA_GST );

  } else {
    // Accumulate all the reports in this time interval into a /coherent/ fix
    fused |= gps.fix();
  }

} // sentenceReceived

//--------------------------

void setup()
{
  RTC::begin();

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
  static uint32_t last_rx = 0L;

  while (uart1.available()) {
    last_rx = RTC::millis();

    if (gps.decode( uart1.getchar() ) == NMEAGPS::DECODE_COMPLETED) {

      // All enabled sentence types will be merged into one fix
      sentenceReceived();

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
    traceIt();
  }

  Power::sleep();
}
