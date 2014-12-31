/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "ubxGPS.h"

#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"
#include "Cosa/RTC.hh"
#include "Cosa/Trace.hh"

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

static uint32_t seconds = 0L;

//--------------------------

class MyGPS : public ubloxGPS
{
public:

    gps_fix merged;

    enum
      {
        GETTING_STATUS, 
        GETTING_LEAP_SECONDS, 
        GETTING_UTC, 
        RUNNING
      }
        state:8;

    bool ok_to_process;
    
    MyGPS( IOStream::Device *device ) : ubloxGPS( device )
      {
        state = GETTING_STATUS;
        ok_to_process = false;
      };

    //--------------------------

    void run()
    {
      static uint32_t last = 0;
      if (last == 0) last = seconds;

      if ((seconds - last) > 2L) {
        last = seconds;
        trace << PSTR("RESTART!\n");
        if (state != GETTING_STATUS) {
          state = GETTING_STATUS;
          enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_STATUS );
        }
      } else {
        while (uart1.available()) {
          if (decode( uart1.getchar() ) == DECODE_COMPLETED) {
            last = seconds;
            if (ok_to_process)
              processSentence();
          }
        }
      }
      
      Power::sleep();
    }

    //--------------------------

    bool processSentence()
      {
        bool old_otp = ok_to_process;
        ok_to_process = false;

        bool ok = false;

        if (!ok && (nmeaMessage >= (nmea_msg_t)ubloxGPS::PUBX_00)) {
//trace << PSTR("n ") << nmeaMessage << endl;
          ok = true;
        }
        if (!ok && (rx().msg_class != ublox::UBX_UNK)) {
//trace << PSTR("u ") << rx().msg_class << PSTR("/") << rx().msg_id << endl;
          ok = true;

          if ((rx().msg_class == ublox::UBX_NAV) &&
              (rx().msg_id == ublox::UBX_NAV_STATUS))
            seconds++;
        }

        if (ok) {

          switch (state) {
            case GETTING_STATUS:
              if (fix().status != gps_fix::STATUS_NONE) {
                trace << PSTR("Acquired status: ") << fix().status << endl;
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
                if (enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS ))
                  state = GETTING_LEAP_SECONDS;
                else
                  trace << PSTR("enable TIMEGPS failed!\n");
              }
              break;

            case GETTING_LEAP_SECONDS:
              if (GPSTime::leap_seconds != 0) {
                trace << PSTR("Acquired leap seconds: ") << GPSTime::leap_seconds << endl;
              }
              if (!disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS ))
                trace << PSTR("disable TIMEGPS failed!\n");
              else if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
                trace << PSTR("enable TIMEUTC failed!\n");
              else
                state = GETTING_UTC;
              break;

            case GETTING_UTC:
              if (GPSTime::start_of_week() != 0) {
                trace << PSTR("Acquired UTC: ") << fix().dateTime << endl;
                trace << PSTR("Acquired Start-of-Week: ") << GPSTime::start_of_week() << endl;

#if defined(GPS_FIX_LOCATION) | defined(GPS_FIX_ALTITUDE) | \
    defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)
                if (!disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
                  trace << PSTR("disable TIMEUTC failed!\n");
                else
                  state = RUNNING;
#else
                state = RUNNING;
#endif

#else

#if defined(GPS_FIX_TIME) | defined(GPS_FIX_DATE)
                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
                  trace << PSTR("enable TIMEUTC failed!\n");
                else
                  state = RUNNING;
#else
                state = RUNNING;
#endif

#endif

#if defined(GPS_FIX_LOCATION) | defined(GPS_FIX_ALTITUDE)
                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_POSLLH ))
                  trace << PSTR("enable POSLLH failed!\n");
#endif

#if defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)
                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_VELNED ))
                  trace << PSTR("enable VELNED failed!\n");
#endif
              }
              break;

            default:
              // See if we stepped into a different time interval,
              //   or if it has finally become valid after a cold start.

              bool newInterval;
#if defined(GPS_FIX_TIME)
              newInterval = (fix().valid.time &&
                            (!merged.valid.time ||
                             (merged.dateTime.seconds != fix().dateTime.seconds) ||
                             (merged.dateTime.minutes != fix().dateTime.minutes) ||
                             (merged.dateTime.hours   != fix().dateTime.hours)));
#elif defined(GPS_FIX_DATE)
              newInterval = (fix().valid.date &&
                            (!merged.valid.date ||
                             (merged.dateTime.date  != fix().dateTime.date) ||
                             (merged.dateTime.month != fix().dateTime.month) ||
                             (merged.dateTime.year  != fix().dateTime.year)));
#else
              //  No date/time configured, so let's assume it's a new interval
              //  if it has been a while since the last sentence was received.
              static uint32_t last_sentence = 0L;

              newInterval = (seconds != last_sentence);
              last_sentence = seconds;
#endif
//trace << PSTR("ps mvd ") << merged.valid.date << PSTR("/") << fix().valid.date;
//trace << PSTR(", mvt ") << merged.valid.time << PSTR("/") << fix().valid.time;
//trace << merged.dateTime << PSTR("/") << fix().dateTime;
//trace << PSTR(", ni = ") << newInterval << endl;
              if (newInterval) {

                // Log the previous interval
                traceIt();

                //  Since we're into the next time interval, we throw away
                //     all of the previous fix and start with what we
                //     just received.
                merged = fix();

              } else {
                // Accumulate all the reports in this time interval
                merged |= fix();
              }
              break;
          }
        }

        ok_to_process = old_otp;

        return ok;
      }

    //--------------------------

    void traceIt()
    {
#if !defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
      trace << seconds << ',';
#endif

      trace << merged << '\n';

    } // traceIt


} __attribute__((packed));

static MyGPS gps( &uart1 );

//--------------------------

void setup()
{
  RTC::begin();

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaUBXGPS: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("ubloxGPS object size = ") << sizeof(ubloxGPS) << endl;
  trace << PSTR("MyGPS object size = ") << sizeof(gps) << endl;
  uart.flush();

  // Start the UART for the GPS device
  uart1.begin(9600);

  // Turn off the preconfigured NMEA standard messages
  for (uint8_t i=NMEAGPS::NMEA_FIRST_MSG; i<=NMEAGPS::NMEA_LAST_MSG; i++) {
    ublox::configNMEA( gps, (NMEAGPS::nmea_msg_t) i, 0 );
  }

  // Turn on the UBX status message
  gps.enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_STATUS );

  // Turn off things that may be left on by a previous build
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_VELNED );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_POSLLH );

  gps.ok_to_process = true;
}

//--------------------------

void loop()
{
  gps.run();
}
