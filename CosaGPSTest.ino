/*
  uart is for trace output.
  uart1 should be connected to the GPS device.
*/

#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Trace.hh"
#include "./NMEAGPS.h"

static NMEAGPS gps;

//--------------------------

//#define USE_FLOAT

static void traceIt()
{
  const gps_fix fix = gps.fix();

  if (fix.valid.date || fix.valid.time) {
    trace << fix.dateTime << PSTR(".");
    if (fix.dateTime_cs < 10)
      trace << '0';
    trace << fix.dateTime_cs;
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

} // traceIt

//--------------------------

static uint8_t passed = 0;
static uint8_t failed = 0;

void setup()
{
  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEAGPS: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("NMEAGPS object size = ") << sizeof(NMEAGPS) << endl;
  uart.flush();


  //  Some basic rejection tests
  for (uint16_t c=0; c < 256; c++) {
    if (c != '$') {
      if (NMEAGPS::DECODE_COMPLETED == gps.decode( (char)c )) {
        trace << PSTR("FAILED to reject single character ") << hex << c << endl;
        failed++;
        return;
      }
    }
  }
  passed++;

  for (uint16_t i=0; i < 256; i++) {
    if (NMEAGPS::DECODE_COMPLETED == gps.decode( '$' )) {
      trace << PSTR("FAILED to reject multiple '$' characters\n");
      failed++;
      return;
    }
  }
  passed++;

  const char *validGGA = (const char *) PSTR("$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,8,1.01,499.6,M,48.0,M,,0*5B\r\n");
  uint8_t validGGA_len = 0;

  uint16_t insert_at=1;
  do {
    const char *ptr = validGGA;
    uint8_t j = 0;
    for (;;) {
      if (j++ == insert_at) {
        if (NMEAGPS::DECODE_COMPLETED == gps.decode( ' ' )) {
          trace << PSTR("FAILED inserting ' ' @ pos ") << insert_at << endl;
          failed++;
          return;
        }
      }
      char c = pgm_read_byte( ptr++ );
      if (!c) {
        if (validGGA_len == 0) {
          validGGA_len = j-1;
          trace << PSTR("Test string length = ") << validGGA_len << endl;
        }
        break;
      }
      if (NMEAGPS::DECODE_COMPLETED == gps.decode( c )) {
        trace << PSTR("FAILED inserting @ pos ") << insert_at << endl;
        failed++;
        return;
      }
    }
  } while (++insert_at < validGGA_len-2);
  passed++;

  for (uint16_t i=0; i < validGGA_len-3; i++) {
    const char *ptr = validGGA;
    uint8_t j = 0;
    char dropped = 0;
    for (;;) {
      char c = pgm_read_byte( ptr++ );
      if (!c || (c == '*')) break;
      if (j == i) dropped = c;
      if ((j++ != i) && (gps.decode( c ) == NMEAGPS::DECODE_COMPLETED)) {
        trace << PSTR("FAILED dropping '") << dropped << PSTR("' at pos ") << i << endl;
        failed++;
        break;
        //return;
      }
    }
  }
  passed++;

  for (uint16_t i=0; i < validGGA_len-3; i++) {
    const char *ptr = validGGA;
    uint8_t j = 0;
    char replaced = 0;
    for (;;) {
      char c = pgm_read_byte( ptr++ );
      if (!c || (c == '*')) break;
      if (j++ == i)
        replaced = c;
      if (NMEAGPS::DECODE_COMPLETED == gps.decode( c )) {
        trace << PSTR("FAILED replacing '") << replaced++ << PSTR("' with '");
        trace << replaced << PSTR("' at pos ") << i << endl;
        failed++;
        break;
        //return;
      }
    }
  }
  passed++;

  {
    const char *ptr = validGGA;
    for (;;) {
      char c = pgm_read_byte( ptr++ );
      if (!c) {
        trace << PSTR("FAILED to parse \"") << ((str_P) validGGA) << PSTR("\"\n");
        failed++;
        break;
      }
      if (NMEAGPS::DECODE_COMPLETED == gps.decode( c )) {
        gps_fix expected;
        expected.dateTime.parse( PSTR("2002-12-09 09:27:25") );
        expected.dateTime_cs = 0;
// $GPRMC,092725.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A
// $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,8,1.01,499.6,M,48.0,M,,0*5B
        if (gps.nmeaMessage != NMEAGPS::NMEA_GGA) {
          trace << PSTR("FAILED wrong message type ") << gps.nmeaMessage << endl;
          failed++;
          break;
        }
        if ((gps.fix().dateTime.hours != expected.dateTime.hours) ||
            (gps.fix().dateTime.minutes != expected.dateTime.minutes) ||
            (gps.fix().dateTime.seconds != expected.dateTime.seconds) ||
            (gps.fix().dateTime_cs != expected.dateTime_cs)) {
          trace << PSTR("FAILED wrong time ") << gps.fix().dateTime << PSTR(".")  << gps.fix().dateTime_cs << PSTR(" != ") << expected.dateTime << PSTR(".") << expected.dateTime_cs << endl;
          failed++;
          break;
        }
        if (gps.fix().latitudeL() != 472852332) {
          trace << PSTR("FAILED wrong latitude ") << gps.fix().latitudeL() << endl;
          failed++;
          break;
        }
        if (gps.fix().longitudeL() != 85652650) {
          trace << PSTR("FAILED wrong longitude ") << gps.fix().longitudeL() << endl;
          failed++;
          break;
        }
        if (gps.fix().status != gps_fix::STATUS_STD) {
          trace << PSTR("FAILED wrong status ") << gps.fix().status << endl;
          failed++;
          break;
        }
        if (gps.fix().satellites != 8) {
          trace << PSTR("FAILED wrong satellites ") << gps.fix().satellites << endl;
          failed++;
//          break;
        }
        if (gps.fix().hdop != 1010) {
          trace << PSTR("FAILED wrong HDOP ") << gps.fix().hdop << endl;
          failed++;
          break;
        }
        if (gps.fix().altitude_cm() != 49960) {
          trace << PSTR("FAILED wrong altitude ") << gps.fix().longitudeL() << endl;
          failed++;
          break;
        }
        break;
      }
    }
  }
  passed++;
}

//--------------------------

void loop()
{
  trace << PSTR("PASSED ") << passed << PSTR(" tests.\n");
  if (failed)
    trace << PSTR("FAILED ") << failed << PSTR(" tests.\n");

  for (;;);
}
