/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/RTC.hh"
#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Watchdog.hh"

#include "NMEAGPS.h"

class MyGPS : public IOStream::Device, public Event::Handler
{
protected:
  NMEAGPS gps;
  NMEAGPS::gps_fix_t merged;

public:
    /**
     * Constructor
     */
    MyGPS() {};

    void on_event( uint8_t, uint16_t )
    {
      bool new_fix = false;
      bool new_safe_fix = false;
      NMEAGPS::gps_fix_t safe_fix;

      // This is susceptible to event processing delays; other kinds of
      // events may delay getting to /fix/ while it is still coherent.

      synchronized {
        if (gps.is_coherent()) {
          if (gps.fix().valid.dateTime && merged.valid.dateTime &&
              (merged.dateTime != *const_cast<const time_t *>(&gps.fix().dateTime)))
            new_fix = true;
          safe_fix = *const_cast<const NMEAGPS::gps_fix_t *>(&gps.fix());
          new_safe_fix = true;
        }
      }

      if (new_fix) {
        traceIt();
        merged = safe_fix;
      } else if (new_safe_fix)
        merged |= safe_fix;
    };
    
    /**
     * Written to by UART driver as soon as a new char is received.
     * Called inside Irq handler.
     */
    virtual int putchar( char c )
    {
      if (gps.decode(c))
        Event::push( Event::RECEIVE_COMPLETED_TYPE, this );
      return c;
    };

    const NMEAGPS::gps_fix_t & fix() const { return merged; };
};

static MyGPS gps;

static IOBuffer<UART::BUFFER_MAX> obuf;
UART uart1(1, &gps, &obuf);

static clock_t now = 0;
static clock_t lastTrace = 0;

//--------------------------

//#define USE_FLOAT

static void traceIt()
{
  const NMEAGPS::gps_fix_t & fix = gps.fix();

  if (fix.valid.dateTime) {
    time_t copy = fix.dateTime;
    trace << copy << PSTR(".");
    if (fix.dateTime_cs < 10)
      trace << '0';
    trace << fix.dateTime_cs;
  } else {
    //  Apparently we don't have a fix yet, ask for a ZDA (Zulu Date and Time)
    NMEAGPS::poll( &uart1, NMEAGPS::NMEA_ZDA );
  }
  trace << PSTR(",");

#ifdef USE_FLOAT
  trace.width(3);
  trace.precision(6);
  if (fix.valid.location)
    trace << fix.latitude() << PSTR(",") << fix.longitude();
  else
    trace << PSTR(",");
  trace << PSTR(",");
  trace.precision(2);
  if (fix.valid.heading)
    trace << fix.heading();
  trace << PSTR(",");
  trace.precision(3);
  if (fix.valid.speed)
    trace << fix.speed();
  trace << PSTR(",");
  trace.precision(2);
  if (fix.valid.altitude)
    trace << fix.altitude();
#else
  if (fix.valid.location)
    trace << fix.latitudeL() << PSTR(",") << fix.longitudeL();
  else
    trace << PSTR(",");
  trace << PSTR(",");
  if (fix.valid.heading)
    trace << fix.heading_cd();
  trace << PSTR(",");
  if (fix.valid.speed)
    trace << fix.speed_mkn();
  trace << PSTR(",");
  if (fix.valid.altitude)
    trace << fix.altitude_cm();
#endif
  trace << endl;

  lastTrace = now;

} // traceIt

//--------------------------

void setup()
{
  // Watchdog for sleeping
  Watchdog::begin( 16, Watchdog::push_timeout_events );
  RTC::begin();

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaGPSEvent: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
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
    
  now = RTC::time();

  if (lastTrace + 5 <= now)
    traceIt();
}
