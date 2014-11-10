/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/RTC.hh"
#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/linkage.hh"
#include "Cosa/Watchdog.hh"

#include "./NMEAGPS.h"

static clock_t now = 0;
static clock_t lastTrace = 0;

//----------------------------------

bool same( const time_t & l, const volatile time_t & r )
{
  return (memcmp( &l, const_cast<const time_t *>(&r), sizeof(l) ) == 0);
}

class MyGPS : public NMEAGPS
{
public:
    gps_fix_t merged;

    MyGPS( IOStream::Device *tx ) : NMEAGPS( tx ) { };

    virtual void on_event ( uint8_t type, uint16_t value )
      {
        if ((NMEA_GGA == (nmea_msg_t)value) ||
            (NMEA_RMC == (nmea_msg_t)value)) {

          // See if we stepped into a different time interval,
          //   or if it has finally become valid after a cold start.

          if (merged.valid.dateTime && fix().valid.dateTime &&
              !same( merged.dateTime, fix().dateTime )) {
            
            //  Set the RTC to this new fix
            RTC::time( (clock_t) (const_cast<const gps_fix_t *>(&fix()))->dateTime );

            // Log the previous interval
            if (merged.valid.location && 
                merged.valid.altitude &&
                merged.valid.speed && 
                merged.valid.heading &&
                merged.valid.dateTime) {

              //  /traceIt/ takes a long time, so the /fix/ data
              //     would probably get overwritten.  Save it first.
              gps_fix_t local_fix = *(const_cast<const gps_fix_t *>(&fix()));
              traceIt( true );

              //  Since we're into the next time interval, we throw away
              //     all of the previous fix and start with what we
              //     just received.
              merged = local_fix;
            } else
              merged = *(const_cast<const gps_fix_t *>(&fix()));

          } else {
            // Accumulate all the reports in this time interval
            merged |= *(const_cast<const gps_fix_t *>(&fix()));
          }
        }
      };

};

extern UART uart1; // forward declaration
MyGPS gps( &uart1 );

static IOBuffer<UART::BUFFER_MAX> obuf;
UART uart1(1, &gps, &obuf);

//--------------------------------

static void print2( uint8_t val )
{
  char buf[4];
  if (val < 10) {
    buf[0] = '0';
    buf[1] = val + '0';
  } else {
    buf[0] = val/10;
    buf[1] = (val-(buf[0]*10)) + '0';
    buf[0]+= '0';
  }
  buf[2] = NULL;

  trace << buf;
}

static void traceIt( bool gotFix )
{
  if (gps.merged.valid.dateTime) {
    print2( gps.merged.dateTime.date );
    print2( gps.merged.dateTime.month );
    print2( gps.merged.dateTime.year  );

    trace << PSTR(",");

    print2( gps.merged.dateTime.hours );
    print2( gps.merged.dateTime.minutes );
    print2( gps.merged.dateTime.seconds );
    print2( gps.merged.dateTime_cs );
    trace << PSTR(",");
  } else {
    trace << PSTR(",,");
    //  Apparently we don't have a fix yet, ask for a ZDA (Zulu Date and Time)
    gps.poll( NMEAGPS::NMEA_ZDA );
  }

  if (gotFix) {
    trace << gps.merged.latitudeL() << PSTR(",") << gps.merged.longitudeL() << PSTR(",");
    trace << gps.merged.heading_cd() << PSTR(",");
    trace << gps.merged.speed_mkn() << PSTR(",");
    trace << gps.merged.altitude_cm() << PSTR(",");

  } else {
    trace << PSTR(",,,,,"); // lat, lon, hdg, spd, alt,
  }

  trace << endl;

  lastTrace = now;

} // traceIt

//--------------

void setup()
{
  // Watchdog for sleeping
  Watchdog::begin( 16, Watchdog::push_timeout_events );
  RTC::begin();

  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEAGPS: started"));

  // Start the additional UART
  delay(200);
  uart1.begin(9600);
  
  trace << PSTR("velocity object size = ") << sizeof(gps.fix().velocity) << endl;
  trace << PSTR("position object size = ") << sizeof(gps.fix().position) << endl;
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("NMEAGPS object size = ") << sizeof(NMEAGPS) << endl;
}

//--------------------------

void loop()
{
  now = RTC::time();

  Event event;
  while (Event::queue.dequeue( &event ))
    event.dispatch();

  if (lastTrace + 5 <= now)
    traceIt( false );
}
