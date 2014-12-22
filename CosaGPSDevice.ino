/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/Trace.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Power.hh"

#include "NMEAGPS.h"

//  The NMEAGPS member is hooked directly to the UART so that it processes
//  characters in the interrupt.
//  The time window for accessing a coherent /fix/ is fairly narrow, 
//  about 9 character times, or about 10mS on a 9600-baud connection.

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
     * Called inside Irq handler.
     */
    virtual int putchar( char c )
    {
      if (gps.decode(c) == NMEAGPS::DECODE_COMPLETED)
        frame_received = true;

      return c;
    };

    const volatile gps_fix & fix() const { return gps.fix(); };

    bool is_coherent() const { return gps.is_coherent(); }
    void poll( IOStream::Device *device, NMEAGPS::nmea_msg_t msg ) const
      { gps.poll(device,msg); };
    void send( IOStream::Device *device, const char *msg ) const
      { gps.send(device,msg); };
    void send( IOStream::Device *device, str_P msg ) const
      { gps.send(device,msg); };
};

static MyGPS gps;

static IOBuffer<UART::BUFFER_MAX> obuf;
UART uart1(1, &gps, &obuf);

//--------------------------

//#define USE_FLOAT

static void traceIt()
{
  if (gps.merged.valid.date || gps.merged.valid.time) {
    trace << gps.merged.dateTime << PSTR(".");
    if (gps.merged.dateTime_cs < 10)
      trace << '0';
    trace << gps.merged.dateTime_cs;
  } else {
    //  Apparently we don't have a fix yet, ask for a ZDA (Zulu Date and Time)
    gps.poll( &uart1, NMEAGPS::NMEA_ZDA );
  }
  trace << PSTR(",");

#ifdef USE_FLOAT
  trace.width(3);
  trace.precision(6);
  if (gps.merged.valid.location)
    trace << gps.merged.latitude() << PSTR(",") << gps.merged.longitude();
  else
    trace << PSTR(",");
  trace << PSTR(",");
  trace.precision(2);
  if (gps.merged.valid.heading)
    trace << gps.merged.heading();
  trace << PSTR(",");
  trace.precision(3);
  if (gps.merged.valid.speed)
    trace << gps.merged.speed();
  trace << PSTR(",");
  trace.precision(2);
  if (gps.merged.valid.altitude)
    trace << gps.merged.altitude();
#else
  if (gps.merged.valid.location)
    trace << gps.merged.latitudeL() << PSTR(",") << gps.merged.longitudeL();
  else
    trace << PSTR(",");
  trace << PSTR(",");
  if (gps.merged.valid.heading)
    trace << gps.merged.heading_cd();
  trace << PSTR(",");
  if (gps.merged.valid.speed)
    trace << gps.merged.speed_mkn();
  trace << PSTR(",");
  if (gps.merged.valid.altitude)
    trace << gps.merged.altitude_cm();
#endif
  trace << endl;

} // traceIt

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
  static gps_fix safe_fix;

  synchronized {
    if (gps.frame_received) {
      gps.frame_received = false;

      // This can be susceptible to processing delays; missing an /is_coherent/
      // means that some data may not get copied into safe_fix.
      if (gps.is_coherent()) {
        safe_fix = *const_cast<const gps_fix *>(&gps.fix());
        new_safe_fix = true;
      }
    }
  }

  if (new_safe_fix) {
    if (safe_fix.valid.date && safe_fix.valid.time &&
        gps.merged.valid.date && gps.merged.valid.time &&
        (gps.merged.dateTime != safe_fix.dateTime)) {
      traceIt();
      gps.merged = safe_fix;
    } else
      gps.merged |= safe_fix;
  }

  Power::sleep();
}
