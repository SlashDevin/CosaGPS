/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/Trace.hh"
#include "Cosa/Event.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"

#include "NMEAGPS.h"

extern UART uart1;

class MyGPS : public IOStream::Device, public Event::Handler
{
protected:
  NMEAGPS gps;
  gps_fix merged;
  gps_fix safe_fix;


public:
    /**
     * Constructor
     */
    MyGPS() {};

    /**
     * Written to by UART driver as soon as a new char is received.
     * Called inside Irq handler.
     */
    virtual int putchar( char c )
    {
      if (gps.decode(c) == NMEAGPS::DECODE_COMPLETED)
        Event::push( Event::RECEIVE_COMPLETED_TYPE, this );
      return c;
    };

    //--------------------------

    void sentenceReceived()
    {
#if !defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
      // No date/time fields enabled, use received GPRMC sentence as a pulse
      // Make sure it's enabled.
#ifndef NMEAGPS_PARSE_RMC
#error NMEAGPS_PARSE_RMC must be defined in NMEAGPS.h!
#endif
      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC) {
        seconds++;
      }
#endif

      // See if we stepped into a different time interval,
      //   or if it has finally become valid after a cold start.

      bool newInterval;
#if defined(GPS_FIX_TIME)
      newInterval = (safe_fix.valid.time &&
                    (!merged.valid.time ||
                     (merged.dateTime.seconds != safe_fix.dateTime.seconds) ||
                     (merged.dateTime.minutes != safe_fix.dateTime.minutes) ||
                     (merged.dateTime.hours   != safe_fix.dateTime.hours)));
#elif defined(GPS_FIX_DATE)
      newInterval = (safe_fix.valid.date &&
                    (!merged.valid.date ||
                     (merged.dateTime.date  != safe_fix.dateTime.date) ||
                     (merged.dateTime.month != safe_fix.dateTime.month) ||
                     (merged.dateTime.year  != safe_fix.dateTime.year)));
#else
      //  No date/time configured, so let's assume it's a new interval
      //  if it has been a while since the last sentence was received.
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
#if !defined(GPS_FIX_DATE) & !defined(GPS_FIX_TIME)
      trace << seconds << ',';
#endif

      trace << merged <<'\n';

    } // traceIt

    //--------------------------

    void on_event( uint8_t, uint16_t )
    {
      bool new_safe_fix = false;

      // This is susceptible to event processing delays; other kinds of
      // events may delay getting to /fix/ while it is still coherent.

      synchronized {
        if (gps.is_coherent()) {
          safe_fix = *const_cast<const gps_fix *>(&gps.fix());
          new_safe_fix = true;
        }
      }

      if (new_safe_fix)
        sentenceReceived();

    }
    
};

static MyGPS gps;

static IOBuffer<UART::BUFFER_MAX> obuf;
UART uart1(1, &gps, &obuf);

//--------------------------

void setup()
{
  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaGPSEvent: started"));
  trace << PSTR("fix object size = ") << sizeof(gps_fix) << endl;
  trace << PSTR("GPS object size = ") << sizeof(gps) << endl;
  uart.flush();

  // Start the UART for the GPS device
  uart1.begin(9600);
}

//--------------------------

void loop()
{
  Event event;
  while (Event::queue.dequeue( &event ))
    event.dispatch();
    
  Power::sleep();
}
