/*
  This test program uses one GPGGA sentence to test the parser's:
  1) robustness WRT dropped, inserted, and mangled characters
  2) correctness WRT values extracted from the input stream
  
  Some care in testing must be taken because
  1) The XOR-style checksum is not very good at catching errors.  
  2) The '*' is a special character for delimiting the CRC.  If
     it is changed, a CR/LF will allow the sentence to pass.

  uart is for trace output.
*/

#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Trace.hh"

#include "NMEAGPS.h"
#include "Streamers.h"

/*
 * Make sure gpsfix.h and NMEAGPS.h are configured properly.
 */

#ifndef NMEAGPS_PARSE_GGA
#error NMEAGPS_PARSE_GGA must be defined in NMEAGPS.h!
#endif

#ifndef GPS_FIX_DATE
#error GPS_FIX_DATE must be defined in gpsfix.h!
#endif

#ifndef GPS_FIX_TIME
#error GPS_FIX_TIME must be defined in gpsfix.h!
#endif

#ifndef GPS_FIX_LOCATION
#error GPS_FIX_LOCATION must be defined in gpsfix.h!
#endif

#ifndef GPS_FIX_ALTITUDE
#error GPS_FIX_ALTITUDE must be defined in gpsfix.h!
#endif

#ifndef GPS_FIX_SPEED
#error GPS_FIX_SPEED must be defined in gpsfix.h!
#endif

#ifndef GPS_FIX_HEADING
#error GPS_FIX_HEADING must be defined in gpsfix.h!
#endif

#ifndef GPS_FIX_SATELLITES
#error GPS_FIX_SATELLITES must be defined in gpsfix.h!
#endif

#ifndef GPS_FIX_HDOP
#error GPS_FIX_HDOP must be defined in gpsfix.h!
#endif

static NMEAGPS gps;

//--------------------------
// Example sentences

const char validGGA[] __PROGMEM =
  "$GPGGA,092725.00,4717.11399,N,00833.91590,E,"
    "1,8,1.01,499.6,M,48.0,M,,0*5B\r\n";
const char validRMC[] __PROGMEM =
  "$GPRMC,092725.00,A,4717.11437,N,00833.91522,E,"
    "0.004,77.52,091202,,,A*5E\r\n";
const char mtk1[] __PROGMEM =
"$GPGGA,064951.000,2307.1256,N,12016.4438,E,1,8,0.95,39.9,M,17.8,M,,*63\r\n";
const char mtk2[] __PROGMEM =
"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C\r\n";
const char mtk3[] __PROGMEM =
"$GPVTG,165.48,T,,M,0.03,N,0.06,K,A*36\r\n";
const char mtk4[] __PROGMEM =
"$GPGSA,A,3,29,21,26,15,18,09,06,10,,,,,2.32,0.95,2.11*00\r\n";
const char mtk5[] __PROGMEM =
"$GPGSV,3,1,09,29,36,029,42,21,46,314,43,26,44,020,43,15,21,321,39*7D\r\n";
const char mtk6[] __PROGMEM =
"$GPGSV,3,2,09,18,26,314,40,09,57,170,44,06,20,229,37,10,26,084,37*77\r\n";
const char mtk7[] __PROGMEM =
"$GPGSV,3,3,09,07,,,26*73\r\n";

const char fpGGA1[] __PROGMEM = "$GPGGA,092725.00,3242.9000,N,11705.8169,W,"
  "1,8,1.01,499.6,M,48.0,M,,0*49\r\n";
const char fpGGA2[] __PROGMEM = "$GPGGA,092725.00,3242.9000,N,11705.8170,W,"
  "1,8,1.01,499.6,M,48.0,M,,0*41\r\n";
const char fpGGA3[] __PROGMEM = "$GPGGA,092725.00,3242.9000,N,11705.8171,W,"
  "1,8,1.01,499.6,M,48.0,M,,0*40\r\n";
const char fpGGA4[] __PROGMEM = "$GPGGA,092725.00,3242.9000,N,11705.8172,W,"
  "1,8,1.01,499.6,M,48.0,M,,0*43\r\n";
const char fpGGA5[] __PROGMEM = "$GPGGA,092725.00,3242.9000,N,11705.8173,W,"
  "1,8,1.01,499.6,M,48.0,M,,0*42\r\n";
const char fpGGA6[] __PROGMEM = "$GPGGA,092725.00,3242.9000,N,11705.8174,W,"
  "1,8,1.01,499.6,M,48.0,M,,0*45\r\n";
const char fpGGA7[] __PROGMEM = "$GPGGA,092725.00,3242.9000,N,11705.8175,W,"
  "1,8,1.01,499.6,M,48.0,M,,0*44\r\n";
const char fpGGA8[] __PROGMEM = "$GPGGA,092725.00,3242.9000,N,11705.8176,W,"
  "1,8,1.01,499.6,M,48.0,M,,0*47\r\n";

//--------------------------

static bool parse_P( str_P ptr )
{
    bool decoded = false;
    char c;

    gps.fix().init();
    char *p = (char *) ptr;
    while ( c = pgm_read_byte( p++ ) ) {
      if (NMEAGPS::DECODE_COMPLETED == gps.decode( c )) {
        decoded = true;
      }
    }

    return decoded;
}

//--------------------------

static void traceSample( str_P ptr )
{
    trace << PSTR("Input:  ") << ptr;

    bool decoded = parse_P( ptr );

    if (decoded)
      trace << PSTR("Results:  ");
    else
      trace << PSTR("Failed to decode!  ");

    trace_all( gps, gps.fix() );
    trace << '\n';
}

//--------------------------

static uint8_t passed = 0;
static uint8_t failed = 0;

void setup()
{
  // Start the normal trace output
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaNMEATtest: started"));
  trace << PSTR("fix object size = ") << sizeof(gps.fix()) << endl;
  trace << PSTR("NMEAGPS object size = ") << sizeof(NMEAGPS) << endl;
  uart.flush();


  //  Some basic rejection tests
  for (uint16_t c=0; c < 256; c++) {
    if (c != '$') {
      if (NMEAGPS::DECODE_CHR_INVALID != gps.decode( (char)c )) {
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

  uint8_t validGGA_len = 0;

  // Insert a ' ' at each position of the test sentence
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

  // Drop one character from each position in example sentence
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

  // Mangle one character from each position in example sentence
  for (uint16_t i=0; i < validGGA_len-3; i++) {
    const char *ptr = validGGA;
    uint8_t j = 0;
    char replaced = 0;
    for (;;) {
      char c = pgm_read_byte( ptr++ );
      if (!c || (c == '*')) break;
      if (j++ == i)
        replaced = c++; // mangle means increment
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

  //  Verify that exact values are extracted
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

        if (gps.nmeaMessage != NMEAGPS::NMEA_GGA) {
          trace << PSTR("FAILED wrong message type ") << gps.nmeaMessage << endl;
          failed++;
          break;
        }
        if ((gps.fix().dateTime.hours   != expected.dateTime.hours  ) ||
            (gps.fix().dateTime.minutes != expected.dateTime.minutes) ||
            (gps.fix().dateTime.seconds != expected.dateTime.seconds) ||
            (gps.fix().dateTime_cs      != expected.dateTime_cs)) {
          trace << PSTR("FAILED wrong time ") << gps.fix().dateTime << '.' << gps.fix().dateTime_cs << PSTR(" != ") << expected.dateTime << '.' << expected.dateTime_cs << endl;
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
          break;
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
  if (failed) {
    trace << PSTR("FAILED ") << failed << PSTR(" tests.\n");
  } else {
    trace << PSTR("------ Samples ------\nResults format:\n  ");
    trace_header();
    trace << '\n';

#ifdef NMEAGPS_STATS
    gps.statistics.ok         = 0L;
    gps.statistics.crc_errors = 0L;
#endif

    traceSample( (str_P) validGGA );
    traceSample( (str_P) validRMC );
    traceSample( (str_P) mtk1 );
    traceSample( (str_P) mtk2 );
    traceSample( (str_P) mtk3 );
    traceSample( (str_P) mtk4 );
    traceSample( (str_P) mtk5 );
    traceSample( (str_P) mtk6 );
    traceSample( (str_P) mtk7 );

    /**
     * This next section displays incremental longitudes.
     * If you have defined USE_FLOAT in Streamers.cpp, this will show
     * how the conversion to /float/ causes loss of accuracy compared 
     * to the /uint32_t/ values.
     */
    trace << PSTR("--- floating point conversion tests ---\n\n");

    traceSample( (str_P) fpGGA1 );
    traceSample( (str_P) fpGGA2 );
    traceSample( (str_P) fpGGA3 );
    traceSample( (str_P) fpGGA4 );
    traceSample( (str_P) fpGGA5 );
    traceSample( (str_P) fpGGA6 );
    traceSample( (str_P) fpGGA7 );
    traceSample( (str_P) fpGGA8 );
  }

  for (;;);
}
