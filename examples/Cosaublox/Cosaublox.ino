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

#if defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
// uncomment this to display just one pulse-per-day.
//#define PULSE_PER_DAY
#endif

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

    uint32_t last_rx;
    uint32_t last_trace;
    uint32_t last_sentence;


    bool ok_to_process;
    
    MyGPS( IOStream::Device *device ) : ubloxGPS( device )
      {
        state = GETTING_STATUS;
        last_rx = 0L;
        last_trace = 0L;
        last_sentence = 0L;
        ok_to_process = false;
      };

    //--------------------------

    void begin()
      {
        last_rx = RTC::millis();
        last_trace = seconds;
      }

    //--------------------------

    void run()
    {
      bool rx = false;

      while (uart1.available()) {
        rx = true;
        if (decode( uart1.getchar() ) == DECODE_COMPLETED) {
          if (ok_to_process)
            processSentence();
        }
      }

      uint32_t ms = RTC::millis();

      if (rx)
        last_rx = ms;

      else {
        if (ok_to_process && ((ms - last_rx) > 2000L)) {
          last_rx = ms;
          trace << PSTR("RESTART!\n");
          if (state != GETTING_STATUS) {
            state = GETTING_STATUS;
            enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_STATUS );
          }
        }

        Power::sleep();
      }
      
    }

    //--------------------------

    bool processSentence()
      {
        bool old_otp = ok_to_process;
        ok_to_process = false;

        bool ok = false;

        if (!ok && (nmeaMessage >= (nmea_msg_t)ubloxGPS::PUBX_00)) {
          ok = true;
        }

        if (!ok && (rx().msg_class != ublox::UBX_UNK)) {
          ok = true;

          // Use the STATUS message as a pulse-per-second
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

#if (defined(GPS_FIX_LOCATION) | defined(GPS_FIX_ALTITUDE)) & \
    defined(UBLOX_PARSE_POSLLH)

                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_POSLLH ))
                  trace << PSTR("enable POSLLH failed!\n");
#endif

#if (defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)) & \
    defined(UBLOX_PARSE_VELNED)
                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_VELNED ))
                  trace << PSTR("enable VELNED failed!\n");
#endif

#if defined(NMEAGPS_PARSE_SATELLITES) & \
    defined(UBLOX_PARSE_SVINFO)
                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_SVINFO ))
                  trace << PSTR("enable SVINFO failed!\n");
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
#elif defined(PULSE_PER_DAY)
              newInterval = (fix().valid.date &&
                            (!merged.valid.date ||
                             (merged.dateTime.date  != fix().dateTime.date) ||
                             (merged.dateTime.month != fix().dateTime.month) ||
                             (merged.dateTime.year  != fix().dateTime.year)));
#else
              //  No date/time configured, so let's assume it's a new
              //  if the seconds have changed.
              newInterval = (seconds != last_sentence);
#endif

              if (newInterval) {

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

        last_sentence = seconds;

        ok_to_process = old_otp;

        return ok;
      }

    //--------------------------

    void traceIt()
    {
      if ((state == RUNNING) && (last_trace != 0)) {

#if !defined(GPS_FIX_TIME) & !defined(PULSE_PER_DAY)
        trace << seconds << ',';
#endif

        trace << merged;

#if defined(NMEAGPS_PARSE_SATELLITES)
        if (merged.valid.satellites) {
          trace << ',' << '[';

          uint8_t i_max = merged.satellites;
          if (i_max > MAX_SATELLITES)
            i_max = MAX_SATELLITES;

          for (uint8_t i=0; i < i_max; i++) {
            trace << satellites[i].id;
#if defined(NMEAGPS_PARSE_SATELLITE_INFO)
            trace << ' ' << 
              satellites[i].elevation << '/' << satellites[i].azimuth;
            trace << '@';
            if (satellites[i].tracked)
              trace << satellites[i].snr;
            else
              trace << '-';
#endif
            trace << ',';
          }
          trace << ']';
        }
#endif

        trace << '\n';
      }

      last_trace = seconds;

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

  gps.begin();

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

  if ((gps.last_trace != seconds) &&
      (RTC::millis() - gps.last_rx > 5)) {
    // It's been 5ms since we received anything,
    // log what we have so far...
    gps.traceIt();
  }
}
