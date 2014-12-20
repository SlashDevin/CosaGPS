/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"

#include "./ubxGPS.h"

#ifdef UBLOX_PARSE_UBLOX
#include "Cosa/RTC.hh"
#endif

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

class MyGPS : public ubloxGPS
{
public:

    gps_fix merged;


#ifdef  UBLOX_PARSE_UBLOX
    MyGPS( IOStream::Device *device ) : ubloxGPS( device ) { };
#else
    MyGPS() : ubloxGPS() { };
#endif

    //--------------------------

    void run()
    {
      static uint16_t since = 0;

      if (since++ >= 2048) {
        since = 0;
trace << PSTR("Hey!\n");
        ask_for_fix();
      }

      while (uart1.available()) {
        if (decode( uart1.getchar() ) == DECODE_COMPLETED) {
          if (processSentence())
            since = 0;
        }
      }
      
      Power::sleep();
    }

    //--------------------------

    bool processSentence()
      {
        bool ok = false;

#ifdef UBLOX_PARSE_NMEA
        if (!ok && (nmeaMessage >= ubloxGPS::PUBX_00)) {
//trace << PSTR("n ") << nmeaMessage << endl;
          ok = true;
        }
#endif

#ifdef UBLOX_PARSE_UBLOX
        if (!ok && (rx().msg_class != ublox::UBX_UNK)) {
//trace << PSTR("u ") << rx().msg_class << PSTR("/") << rx().msg_id << endl;
          ok = true;
        }
#endif

        if (ok) {

          // See if we stepped into a different time interval,
          //   or if it has finally become valid after a cold start.

          bool newInterval = true;
#if defined(GPS_FIX_TIME)
          newInterval = fix().valid.time &&
                        (!merged.valid.time ||
                         (merged.dateTime.seconds != fix().dateTime.seconds) ||
                         (merged.dateTime.minutes != fix().dateTime.minutes) ||
                         (merged.dateTime.hours   != fix().dateTime.hours));
#elif defined(GPS_FIX_DATE)
          newInterval = fix().valid.date &&
                        (!merged.valid.date ||
                         (merged.dateTime.date  != fix().dateTime.date) ||
                         (merged.dateTime.month != fix().dateTime.month) ||
                         (merged.dateTime.year  != fix().dateTime.year));
#endif
//trace << PSTR("ps mvd ") << merged.valid.date << PSTR("/") << fix().valid.date;
//trace << PSTR(", mvt ") << merged.valid.time << PSTR("/") << fix().valid.time;
//trace << merged.dateTime << PSTR("/") << fix().dateTime;
//trace << PSTR(", ni = ") << newInterval << endl;
          if (newInterval) {

            // Log the previous interval
            traceIt();

            ask_for_fix();
            
            //  Since we're into the next time interval, we throw away
            //     all of the previous fix and start with what we
            //     just received.
            merged = fix();

          } else {
            // Accumulate all the reports in this time interval
            merged |= fix();
          }
        }

        return ok;
      }

    void ask_for_fix()
      {
#ifdef UBLOX_PARSE_NMEA
        NMEAGPS::send( &uart1, PSTR("PUBX,00") );
        NMEAGPS::send( &uart1, PSTR("PUBX,04") );
#endif

#ifdef UBLOX_PARSE_UBLOX
        ublox::nav_status_t navstat;
        poll_request( navstat );

#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
        if (fix().status != gps_fix::STATUS_NONE) {

          if (leap_seconds == 0) {
            ublox::nav_timegps_t timegps;
            poll_request( timegps );

          } else if (start_of_week() == 0) {
            ublox::nav_timeutc_t timeutc;
            poll_request( timeutc );

          } else {
#if defined(GPS_FIX_LOCATION) | defined(GPS_FIX_ALTITUDE)
            ublox::nav_posllh_t posllh;
            poll_request( posllh );
#endif

#if defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)
            ublox::nav_velned_t velned;
            poll_request( velned );
#endif
          }
        }
#endif

#endif
      }

    //--------------------------

//#define USE_FLOAT

    void traceIt()
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
#else
      static uint16_t gps_seconds = 2;
      trace << gps_seconds++;
#ifdef UBLOX_PARSE_UBLOX
      trace << ',' << RTC::seconds();
#endif

#endif
      trace << ',';

#ifdef USE_FLOAT
      trace.width(3);
      trace.precision(6);
#ifdef GPS_FIX_LOCATION
      if (merged.valid.location)
        trace << merged.latitude() << ',' << merged.longitude();
      else
        trace << ',';
      trace << ',';
#endif
#ifdef GPS_FIX_HEADING
      trace.precision(2);
      if (merged.valid.heading)
        trace << merged.heading();
      trace << ',';
#endif
#ifdef GPS_FIX_SPEED
      trace.precision(3);
      if (merged.valid.speed)
        trace << merged.speed();
      trace << ',';
#endif
#ifdef GPS_FIX_ALTITUDE
      trace.precision(2);
      if (merged.valid.altitude)
        trace << merged.altitude();
      trace << ',';
#endif

#else

#ifdef GPS_FIX_LOCATION
      if (merged.valid.location)
        trace << merged.latitudeL() << ',' << merged.longitudeL();
      else
        trace << ',';
      trace << ',';
#endif
#ifdef GPS_FIX_HEADING
      if (merged.valid.heading)
        trace << merged.heading_cd();
      trace << ',';
#endif
#ifdef GPS_FIX_SPEED
      if (merged.valid.speed)
        trace << merged.speed_mkn();
      trace << ',';
#endif
#ifdef GPS_FIX_ALTITUDE
      if (merged.valid.altitude)
        trace << merged.altitude_cm();
      trace << ',';
#endif
#endif

#ifdef GPS_FIX_SATELLITES
      if (merged.valid.satellites)
        trace << merged.satellites;
      trace << ',';
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

};

#ifdef  UBLOX_PARSE_UBLOX
static MyGPS gps( &uart1 );
#else
static MyGPS gps;
#endif

//--------------------------

void setup()
{
#ifdef UBLOX_PARSE_UBLOX
  RTC::begin();
#endif

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaUBXGPS: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("ubloxGPS object size = ") << sizeof(ubloxGPS) << endl;
  trace << PSTR("gps object size = ") << sizeof(gps) << endl;
  uart.flush();

  // Start the UART for the GPS device
  uart1.begin(9600);

  // Turn off the predefined standard messages
  for (uint8_t i=NMEAGPS::NMEA_FIRST_MSG; i<=NMEAGPS::NMEA_LAST_MSG; i++) {
    ublox::configNMEA( gps, static_cast<NMEAGPS::nmea_msg_t>(i), 0 );
  }

}

//--------------------------

void loop()
{
  gps.run();

//  Power::sleep();
}
