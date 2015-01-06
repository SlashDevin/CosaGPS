/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"

#include "NMEAGPS.h"

#if defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
// uncomment this to display just one pulse-per-day.
//#define PULSE_PER_DAY
#endif

static uint32_t seconds = 0L;

gps_fix safe_fix;

/**
 * Hook the parser directly to the UART so that it processes characters in
 * the interrupt.  The time window for safely accessing a /fix/ is fairly
 * narrow, about 9 character times, or about 10mS on a 9600-baud connection.
 */

class MyGPS : public IOStream::Device
{
protected:
  NMEAGPS gps;

public:
    gps_fix merged;

    volatile bool frame_received;

    /**
     * Constructor
     */
    MyGPS()
      : frame_received( false )
        {}

    /**
     * Written to by UART driver as soon as a new char is received.
     * Called inside IRQ handler.
     */
    virtual int putchar( char c )
    {
      if (gps.decode(c) == NMEAGPS::DECODE_COMPLETED)
        frame_received = true;

      return c;
    };

    const volatile gps_fix & fix() const { return gps.fix(); };

    bool is_safe() const { return gps.is_safe(); }
    void poll( IOStream::Device *device, NMEAGPS::nmea_msg_t msg ) const
      { gps.poll(device,msg); };
    void send( IOStream::Device *device, const char *msg ) const
      { gps.send(device,msg); };
    void send( IOStream::Device *device, str_P msg ) const
      { gps.send(device,msg); };

    //--------------------------

    void sentenceReceived()
    {
      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC)
        // Use received GPRMC sentence as a pulse
        seconds++;

      // See if we stepped into a different time interval,
      //   or if it has finally become valid after a cold start.

      bool newInterval;
#if defined(GPS_FIX_TIME)
      newInterval = (safe_fix.valid.time &&
                    (!merged.valid.time ||
                     (merged.dateTime.seconds != safe_fix.dateTime.seconds) ||
                     (merged.dateTime.minutes != safe_fix.dateTime.minutes) ||
                     (merged.dateTime.hours   != safe_fix.dateTime.hours)));
#elif defined(PULSE_PER_DAY)
      newInterval = (safe_fix.valid.date &&
                    (!merged.valid.date ||
                     (merged.dateTime.date  != safe_fix.dateTime.date) ||
                     (merged.dateTime.month != safe_fix.dateTime.month) ||
                     (merged.dateTime.year  != safe_fix.dateTime.year)));
#else
      //  No date/time configured, so let's assume it's a new interval
      //  if the seconds have changed.
      static uint32_t last_sentence = 0L;
      
      newInterval = (seconds != last_sentence);
      last_sentence = seconds;
#endif

      if (newInterval) {

        // Log the previous interval
        traceIt();

        //  Since we're into the next time interval, we throw away
        //     all of the previous fix and start with what we
        //     just received.
        merged = safe_fix;

      } else {
        // Accumulate all the reports in this time interval
        merged |= safe_fix;
      }

    } // sentenceReceived

    //--------------------------

    void traceIt()
    {
#if !defined(GPS_FIX_TIME) & !defined(PULSE_PER_DAY)
      //  Date/Time not enabled, just output the interval number
      trace << seconds << ',';
#endif

      trace << merged;

#if defined(NMEAGPS_PARSE_SATELLITES)
      if (merged.valid.satellites) {
        trace << ',' << '[';

        // This is a little dangerous because gps.satellites is volatile.
        // If you need to access the satellites array, be sure to
        // use one of the mitigation techniques: is_safe+synchronized 
        // (as below) and copy a safe_array.
        uint8_t i_max = merged.satellites;
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

};

static MyGPS gps;

static IOBuffer<UART::BUFFER_MAX> obuf;
UART uart1(1, &gps, &obuf);

//--------------------------

void setup()
{
  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaGPSDevice: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("GPS object size = ") << sizeof(gps) << endl;
  uart.flush();

  // Start the UART for the GPS device
  uart1.begin(9600);
}

//--------------------------

void loop()
{
  bool new_safe_fix = false;

  synchronized {
    if (gps.frame_received) {
      gps.frame_received = false;

      // This can be susceptible to processing delays; missing an /is_safe/
      // means that some data may not get copied into safe_fix.
      if (gps.is_safe()) {
        safe_fix = *const_cast<const gps_fix *>(&gps.fix());
        new_safe_fix = true;
      }
    }
  }

  if (new_safe_fix)
    gps.sentenceReceived();

  Power::sleep();
}
