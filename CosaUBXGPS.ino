/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"

#include "./ubxGPS.h"

static IOBuffer<UART::BUFFER_MAX> obuf;
static IOBuffer<UART::BUFFER_MAX> ibuf;
UART uart1(1, &ibuf, &obuf);

class MyGPS : public ubloxGPS
{
public:

    gps_fix merged;

#ifndef  UBLOX_PARSE_UBLOX
    MyGPS() : ubloxGPS() { };
#else
    MyGPS( IOStream::Device *device ) : ubloxGPS( device ) { };

    static const uint16_t msg_length = sizeof(ublox::nav_velned_t)-sizeof(ublox::msg_t);

    ublox::msg_t storage;
    uint8_t body[msg_length];

    ublox::nav_posllh_t  & posllh()
      {
        storage.msg_class = ublox::UBX_NAV;
        storage.msg_id = ublox::UBX_NAV_POSLLH;
        return *((ublox::nav_posllh_t  *)&storage);
      }
    ublox::nav_velned_t  & velned()
      {
        storage.msg_class = ublox::UBX_NAV;
        storage.msg_id = ublox::UBX_NAV_VELNED;
        return *((ublox::nav_velned_t  *)&storage);
      }
    ublox::nav_timegps_t & timegps()
      {
        storage.msg_class = ublox::UBX_NAV;
        storage.msg_id = ublox::UBX_NAV_TIMEGPS;
        return *((ublox::nav_timegps_t *)&storage);
      }
    ublox::nav_timeutc_t & timeutc()
      {
        storage.msg_class = ublox::UBX_NAV;
        storage.msg_id = ublox::UBX_NAV_TIMEUTC;
        return *((ublox::nav_timeutc_t *)&storage);
      }

    //--------------------------

    virtual ublox::msg_t *storage_for( const ublox::msg_t & rx_msg )
      {
        if ((rx_msg.msg_class == ublox::UBX_NAV) &&
            ((rx_msg.msg_id == ublox::UBX_NAV_POSLLH) ||
             (rx_msg.msg_id == ublox::UBX_NAV_VELNED) ||
             (rx_msg.msg_id == ublox::UBX_NAV_TIMEGPS) ||
             (rx_msg.msg_id == ublox::UBX_NAV_TIMEUTC))) {
//trace << PSTR(" -> ");
          storage.msg_id = rx_msg.msg_id;
          storage.length = msg_length;
          return &storage;
        }
        return (ublox::msg_t *) NULL;
      }

    //--------------------------
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)

    void parse_posllh()
      {
//trace << PSTR( " posllh ");
        if ((start_of_week != 0) && (leap_seconds != 0)) {
//trace << PSTR("@ ") << posllh().time_of_week;
          from_TOWms( posllh().time_of_week );
//trace << PSTR(".") << m_fix.dateTime_cs;
        }
#ifdef GPS_FIX_LOCATION
        m_fix.lat = posllh().lat;
        m_fix.lon = posllh().lon;
        m_fix.valid.location = true;
#endif
#ifdef GPS_FIX_ALTITUDE
//trace << PSTR(" alt = ") << posllh().height_MSL;
        m_fix.alt.whole = posllh().height_MSL / 1000UL;
        m_fix.alt.frac  = ((uint16_t)(posllh().height_MSL - (m_fix.alt.whole * 1000UL)))/10;
//trace << PSTR(" = ") << m_fix.alt.whole << PSTR(".");
//if (m_fix.alt.frac < 10) trace << '0';
//trace << m_fix.alt.frac;
        m_fix.valid.altitude = true;
#endif
//trace << endl;
      }

    //--------------------------

    void parse_velned()
      {
//trace << PSTR( " velned ");
        if ((start_of_week != 0) && (leap_seconds != 0))
//{ trace << PSTR("@ ") << velned().time_of_week;
          from_TOWms( velned().time_of_week );
//trace << PSTR(".") << m_fix.dateTime_cs; }
#ifdef GPS_FIX_SPEED
//trace << PSTR(" spd = ") << velned().speed_2D;
        uint32_t t = (velned().speed_2D * 36UL);
        m_fix.spd.whole = t/1000UL; // kph = cm/s * 3600/100000
        m_fix.spd.frac = t - (m_fix.spd.whole * 1000UL);
        m_fix.valid.speed = true;
#endif
#ifdef GPS_FIX_HEADING
//trace << PSTR(" hdg = ") << velned().heading;
        m_fix.hdg.whole = velned().heading / 100000UL;
        m_fix.hdg.frac  = (velned().heading - (m_fix.hdg.whole * 100000UL))/1000UL;
        m_fix.valid.heading = true;
#endif
//trace << endl;
      }

    //--------------------------

    void parse_timegps()
      {
//trace << PSTR( " timegps ");
        if (timegps().valid.leap_seconds)
          leap_seconds = timegps().leap_seconds;
        if (leap_seconds != 0) {
          if ((start_of_week == 0) && m_fix.valid.time && m_fix.valid.date)
            setup_start_of_week( m_fix.dateTime );
          if ((start_of_week != 0) && timegps().valid.time_of_week)
//{ trace << timegps().time_of_week;
            from_TOWms( timegps().time_of_week );
//trace << PSTR(".") << m_fix.dateTime_cs; }
        }
//trace << endl;
      }

    //--------------------------

    void parse_timeutc()
      {
//trace << PSTR( " timeutc ");
        if (timeutc().valid.UTC) {
          m_fix.dateTime.year    = timeutc().year % 100;
          m_fix.dateTime.month   = timeutc().month;
          m_fix.dateTime.date    = timeutc().day;
          m_fix.dateTime.hours   = timeutc().hour;
          m_fix.dateTime.minutes = timeutc().minute;
          m_fix.dateTime.seconds = timeutc().second;
          if (timeutc().valid.time_of_week)
//{
            m_fix.dateTime_cs = (timeutc().time_of_week/10UL) % 100;
//trace << timeutc().time_of_week << PSTR(".") << m_fix.dateTime_cs; }
          m_fix.valid.date = true;
          m_fix.valid.time = true;

          if ((start_of_week == 0) && (leap_seconds != 0))
            setup_start_of_week( m_fix.dateTime );
        }
//trace << endl;
      }
#endif
#endif

    //--------------------------

    bool processSentence()
      {
        bool ok = false;

#ifdef UBLOX_PARSE_NMEA
        if (!ok && (nmeaMessage >= ubloxGPS::PUBX_00)) {
//trace << PSTR("nmeaMsg = ") << nmeaMessage << endl;
          ok = true;
        }
#endif

#ifdef UBLOX_PARSE_UBLOX
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
#if defined(GPS_FIX_LOCATION) | defined(GPS_FIX_ALTITUDE)
        if (!ok && rx().same_kind( posllh() )) {
          ok = true;
          parse_posllh();
        }
#endif
#if defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)
        if (!ok && rx().same_kind( velned() )) {
          ok = true;
          parse_velned();
        }
#endif
        if (rx().same_kind( timegps() )) {
          ok = true;
          parse_timegps();
        }
        if (rx().same_kind( timeutc() )) {
          ok = true;
          parse_timeutc();
        }
#endif
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
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
        if (leap_seconds == 0)
          poll_request( timegps() );
        else if (start_of_week == 0)
          poll_request( timeutc() );
        else {
#if defined(GPS_FIX_LOCATION) | defined(GPS_FIX_ALTITUDE)
          poll_request( posllh() );
#endif
#if defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)
          poll_request( velned() );
#endif
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

};

#ifdef  UBLOX_PARSE_UBLOX
static MyGPS gps( &uart1 );
#else
static MyGPS gps;
#endif

//--------------------------

void setup()
{
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
  static uint16_t since = 0;

  if (since++ >= 2048) {
    since = 0;
trace << PSTR("Hey!\n");
    gps.ask_for_fix();
  }

  while (uart1.available()) {
    if (gps.decode( uart1.getchar() ) == ubloxGPS::DECODE_COMPLETED) {
      if (gps.processSentence())
        since = 0;
    }
  }

  Power::sleep();

}
