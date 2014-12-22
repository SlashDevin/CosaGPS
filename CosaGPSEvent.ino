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

public:
    /**
     * Constructor
     */
    MyGPS() {};

    const gps_fix & fix() const { return merged; };

    void on_event( uint8_t, uint16_t )
    {
      bool new_safe_fix = false;
      static gps_fix safe_fix;

      // This is susceptible to event processing delays; other kinds of
      // events may delay getting to /fix/ while it is still coherent.

      synchronized {
        if (gps.is_coherent()) {
          safe_fix = *const_cast<const gps_fix *>(&gps.fix());
          new_safe_fix = true;
        }
      }

      if (new_safe_fix) {
        if (safe_fix.valid.date && safe_fix.valid.time &&
            merged.valid.date && merged.valid.time &&
            (merged.dateTime != safe_fix.dateTime)) {
          traceIt();
          merged = safe_fix;
        } else
          merged |= safe_fix;
      }
    };
    
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

//#define USE_FLOAT

    void traceIt()
    {
      if (fix().valid.date || fix().valid.time) {
        time_t copy = fix().dateTime;
        trace << copy << PSTR(".");
        if (fix().dateTime_cs < 10)
          trace << '0';
        trace << fix().dateTime_cs;
      } else {
        //  Apparently we don't have a fix yet, ask for a ZDA (Zulu Date and Time)
        NMEAGPS::poll( &uart1, NMEAGPS::NMEA_ZDA );
      }
      trace << PSTR(",");

#ifdef USE_FLOAT
      trace.width(3);
      trace.precision(6);
      if (fix().valid.location)
        trace << fix().latitude() << PSTR(",") << fix().longitude();
      else
        trace << PSTR(",");
      trace << PSTR(",");
      trace.precision(2);
      if (fix().valid.heading)
        trace << fix().heading();
      trace << PSTR(",");
      trace.precision(3);
      if (fix().valid.speed)
        trace << fix().speed();
      trace << PSTR(",");
      trace.precision(2);
      if (fix().valid.altitude)
        trace << fix().altitude();
#else
      if (fix().valid.location)
        trace << fix().latitudeL() << PSTR(",") << fix().longitudeL();
      else
        trace << PSTR(",");
      trace << PSTR(",");
      if (fix().valid.heading)
        trace << fix().heading_cd();
      trace << PSTR(",");
      if (fix().valid.speed)
        trace << fix().speed_mkn();
      trace << PSTR(",");
      if (fix().valid.altitude)
        trace << fix().altitude_cm();
#endif
      trace << endl;

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
    
  Power::sleep();
}
