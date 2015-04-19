/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"
#include "Cosa/RTC.hh"
#include "Cosa/Trace.hh"

#include "ubxGPS.h"
#include "Streamers.h"

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

// uncomment this to display just one pulse-per-day.
//#define PULSE_PER_DAY

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
        state NEOGPS_BF(8);

    uint32_t last_rx;
    uint32_t last_trace;
    uint32_t last_sentence;

    // Prevent recursive sentence processing while waiting for ACKs
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

      while (Device()->available()) {
        rx = true;
        if (decode( Device()->getchar() ) == DECODE_COMPLETED) {
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

    void get_status()
    {
      static bool acquiring = false;

      if (fix().status == gps_fix::STATUS_NONE) {
        if (!acquiring) {
          acquiring = true;
          trace << PSTR("Acquiring...");
        } else
          trace << '.';

      } else {
        if (acquiring)
          trace << '\n';
        trace << PSTR("Acquired status: ") << (uint8_t) fix().status << '\n';

#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & defined(UBLOX_PARSE_TIMEGPS)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS ))
          trace << PSTR("enable TIMEGPS failed!\n");

        state = GETTING_LEAP_SECONDS;
#else
        start_running();
        state = RUNNING;
#endif
      }
    } // get_status

    //--------------------------

    void get_leap_seconds()
    {
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & defined(UBLOX_PARSE_TIMEGPS)
        if (GPSTime::leap_seconds != 0) {
          trace << PSTR("Acquired leap seconds: ") << GPSTime::leap_seconds << '\n';

          if (!disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS ))
            trace << PSTR("disable TIMEGPS failed!\n");

#if defined(UBLOX_PARSE_TIMEUTC)
          if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
            trace << PSTR("enable TIMEUTC failed!\n");
          state = GETTING_UTC;
#else
          start_running();
#endif
        }
#endif
    } // get_leap_seconds

    //--------------------------

    void get_utc()
    {
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & defined(UBLOX_PARSE_TIMEUTC)
        if (GPSTime::start_of_week() != 0) {
          trace << PSTR("Acquired UTC: ") << fix().dateTime << '\n';
          trace << PSTR("Acquired Start-of-Week: ") << GPSTime::start_of_week() << '\n';

          start_running();
        }
#endif
    } // get_utc

    //--------------------------

    void start_running()
    {
      bool enabled_msg_with_time = false;

#if (defined(GPS_FIX_LOCATION) | defined(GPS_FIX_ALTITUDE)) & \
    defined(UBLOX_PARSE_POSLLH)
                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_POSLLH ))
                  trace << PSTR("enable POSLLH failed!\n");
        enabled_msg_with_time = true;
#endif

#if (defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)) & \
    defined(UBLOX_PARSE_VELNED)
                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_VELNED ))
                  trace << PSTR("enable VELNED failed!\n");
        enabled_msg_with_time = true;
#endif

#if defined(NMEAGPS_PARSE_SATELLITES) & \
    defined(UBLOX_PARSE_SVINFO)
                if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_SVINFO ))
                  trace << PSTR("enable SVINFO failed!\n");
        enabled_msg_with_time = true;
#endif

#if defined(UBLOX_PARSE_TIMEUTC)
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
        if (enabled_msg_with_time &&
            !disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
          trace << PSTR("disable TIMEUTC failed!\n");

#elif defined(GPS_FIX_TIME) | defined(GPS_FIX_DATE)
        // If both aren't defined, we can't convert TOW to UTC,
        // so ask for the separate UTC message.
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
          trace << PSTR("enable TIMEUTC failed!\n");
#endif
#endif
        state = RUNNING;
        trace_header();
    }

    //--------------------------

    bool is_new_interval()
    {
        // See if we stepped into a different time interval,
        //   or if it has finally become valid after a cold start.

        bool new_interval;
#if defined(GPS_FIX_TIME)
        new_interval = (fix().valid.time &&
                      (!merged.valid.time ||
                       (merged.dateTime.seconds != fix().dateTime.seconds) ||
                       (merged.dateTime.minutes != fix().dateTime.minutes) ||
                       (merged.dateTime.hours   != fix().dateTime.hours)));
#elif defined(PULSE_PER_DAY)
        new_interval = (fix().valid.date &&
                      (!merged.valid.date ||
                       (merged.dateTime.date  != fix().dateTime.date) ||
                       (merged.dateTime.month != fix().dateTime.month) ||
                       (merged.dateTime.year  != fix().dateTime.year)));
#else
        //  No date/time configured, so let's assume it's a new
        //  if the seconds have changed.
        new_interval = (seconds != last_sentence);
#endif
      return new_interval;

    } // is_new_interval

    //--------------------------

    bool processSentence()
      {
        bool old_otp = ok_to_process;
        ok_to_process = false;

        bool ok = false;

        if (!ok && (nmeaMessage >= (nmea_msg_t)ubloxGPS::PUBX_00))
          ok = true;

        if (!ok && (rx().msg_class != ublox::UBX_UNK)) {
          ok = true;

          // Use the STATUS message as a pulse-per-second
          if ((rx().msg_class == ublox::UBX_NAV) &&
              (rx().msg_id == ublox::UBX_NAV_STATUS))
            seconds++;
        }

        if (ok) {

          switch (state) {
            case GETTING_STATUS      : get_status      (); break;

            case GETTING_LEAP_SECONDS: get_leap_seconds(); break;

            case GETTING_UTC         : get_utc         (); break;

            case RUNNING:
              if (is_new_interval()) {

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
      if ((state == RUNNING) && (last_trace != 0))
        trace_all( *this, merged );

      last_trace = seconds;

    } // traceIt


} NEOGPS_PACKED;

// Construct the GPS object and hook it to the appropriate serial device
static MyGPS gps( &uart1 );

//--------------------------

void setup()
{
  RTC::begin();

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("Cosaublox: started"));
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

#if 0
  // Test a Neo M8 message -- should be rejected by Neo-6 and Neo7
  ublox::cfg_nmea_v1_t test;

  test.always_output_pos  = false; // invalid or failed
  test.output_invalid_pos = false;
  test.output_invalid_time= false;
  test.output_invalid_date= false;
  test.use_GPS_only       = false;
  test.output_heading     = false; // even if frozen
  test.__not_used__       = false;

  test.nmea_version = ublox::cfg_nmea_v1_t::NMEA_V_4_0;
  test.num_sats_per_talker_id = ublox::cfg_nmea_v1_t::SV_PER_TALKERID_UNLIMITED;

  test.compatibility_mode = false;
  test.considering_mode   = true;
  test.max_line_length_82 = false;
  test.__not_used_1__     = 0;

  test.filter_gps    = false;
  test.filter_sbas   = false;
  test.__not_used_2__= 0;
  test.filter_qzss   = false;
  test.filter_glonass= false;
  test.filter_beidou = false;
  test.__not_used_3__= 0;

  test.proprietary_sat_numbering = false;
  test.main_talker_id = ublox::cfg_nmea_v1_t::MAIN_TALKER_ID_GP;
  test.gsv_uses_main_talker_id = true;
  test.beidou_talker_id[0] = 'G';
  test.beidou_talker_id[1] = 'P';

  trace << PSTR("CFG_NMEA result = ") << gps.send( test );
#endif

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
