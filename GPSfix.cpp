#include "GPSfix.h"

#include "Cosa/IOStream.hh"

//#define USE_FLOAT

IOStream & operator <<( IOStream &outs, const gps_fix &fix )
{
  if (fix.valid.status)
    outs << (uint8_t) fix.status;
  outs << ',';

#if defined(GPS_FIX_DATE) | defined(GPS_FIX_TIME)
  bool someTime = false;
#if defined(GPS_FIX_DATE)
  someTime |= fix.valid.date;
#endif
#if defined(GPS_FIX_TIME)
  someTime |= fix.valid.time;
#endif

  if (someTime) {
    outs << fix.dateTime << '.';
    if (fix.dateTime_cs < 10)
      outs << '0';
    outs << fix.dateTime_cs;
  }
  outs << ',';
#endif

#ifdef USE_FLOAT
#ifdef GPS_FIX_LOCATION
  if (fix.valid.location) {
    outs.print( fix.latitude(), 10, 6 );
    outs << ',';
    outs.print( fix.longitude(), 10, 6 );
  } else
    outs << ',';
  outs << ',';
#endif
#ifdef GPS_FIX_HEADING
  if (fix.valid.heading)
    outs.print( fix.heading(), 6, 2 );
  outs << ',';
#endif
#ifdef GPS_FIX_SPEED
  if (fix.valid.speed)
    outs.print( fix.speed(), 8, 3 );
  outs << ',';
#endif
#ifdef GPS_FIX_ALTITUDE
  if (fix.valid.altitude)
    outs.print( fix.altitude(), 7, 2 );
  outs << ',';
#endif

#else /* not USE_FLOAT */

#ifdef GPS_FIX_LOCATION
  if (fix.valid.location)
    outs << fix.latitudeL() << ',' << fix.longitudeL();
  else
    outs << ',';
  outs << ',';
#endif
#ifdef GPS_FIX_HEADING
  if (fix.valid.heading)
    outs << fix.heading_cd();
  outs << ',';
#endif
#ifdef GPS_FIX_SPEED
  if (fix.valid.speed)
    outs << fix.speed_mkn();
  outs << ',';
#endif
#ifdef GPS_FIX_ALTITUDE
  if (fix.valid.altitude)
    outs << fix.altitude_cm();
  outs << ',';
#endif
#endif

#ifdef GPS_FIX_SATELLITES
  if (fix.valid.satellites)
    outs << fix.satellites;
  outs << ',';
#endif

#ifdef USE_FLOAT

#ifdef GPS_FIX_HDOP
  if (fix.valid.hdop)
    outs.print( (fix.hdop * 0.001), 6, 3 );
  outs << ',';
#endif
#ifdef GPS_FIX_VDOP
  if (fix.valid.vdop)
    outs.print( (fix.vdop * 0.001), 6, 3 );
  outs << ',';
#endif
#ifdef GPS_FIX_PDOP
  if (fix.valid.pdop)
    outs.print( (fix.pdop * 0.001), 6, 3 );
  outs << ',';
#endif

#ifdef GPS_FIX_LAT_ERR
  if (fix.valid.lat_err)
    outs.print( fix.lat_err(), 5, 2 );
  outs << ',';
#endif
#ifdef GPS_FIX_LON_ERR
  if (fix.valid.lon_err)
    outs.print( fix.lon_err(), 5, 2 );
  outs << ',';
#endif
#ifdef GPS_FIX_ALT_ERR
  if (fix.valid.alt_err)
    outs.print( fix.alt_err(), 5, 2 );
  outs << ',';
#endif

#else

#ifdef GPS_FIX_HDOP
  if (fix.valid.hdop)
    outs << fix.hdop;
  outs << ',';
#endif
#ifdef GPS_FIX_VDOP
  if (fix.valid.vdop)
    outs << fix.vdop;
  outs << ',';
#endif
#ifdef GPS_FIX_PDOP
  if (fix.valid.pdop)
    outs << fix.pdop;
  outs << ',';
#endif

#ifdef GPS_FIX_LAT_ERR
  if (fix.valid.lat_err)
    outs << fix.lat_err_cm;
  outs << ',';
#endif
#ifdef GPS_FIX_LON_ERR
  if (fix.valid.lon_err)
    outs << fix.lon_err_cm;
  outs << ',';
#endif
#ifdef GPS_FIX_ALT_ERR
  if (fix.valid.alt_err)
    outs << fix.alt_err_cm;
  outs << ',';
#endif

#endif

  return outs;
}
