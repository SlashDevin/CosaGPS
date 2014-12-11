#ifndef GPSFIX_H
#define GPSFIX_H

// Include core libraries
#include "Cosa/Time.hh"

#define GPS_FIX_LOCATION
#define GPS_FIX_ALTITUDE
#define GPS_FIX_SPEED
#define GPS_FIX_HEADING
#define GPS_FIX_SATELLITES
#define GPS_FIX_HDOP
#define GPS_FIX_DATE
#define GPS_FIX_TIME

class gps_fix {
public:

  gps_fix() { init(); };

  /**
   * A structure for holding the two parts of a floating-point number.
   * This is used for Altitude, Heading and Speed, which require more
   * significant digits than a 16-bit number.  The decimal point is
   * used as a field separator for these two parts.  This is more efficient
   * than calling the 32-bit math subroutines on a single scaled long integer.
   */

  struct whole_frac {
    int16_t whole;
    int16_t frac;
    void init() { whole = 0; frac = 0; };
    int32_t int32_00() const { return ((int32_t)whole) * 100L + frac; };
    int16_t int16_00() const { return whole * 100 + frac; };
    int32_t int32_000() const { return whole * 1000L + frac; };
    float float_00() const { return ((float)whole) + ((float)frac)*0.01; };
    double double_00() const { return ((double)whole) + ((double)frac)*0.01; };
    float float_000() const { return ((float)whole) + ((float)frac)*0.001; };
  } __attribute__((packed));

  /**
   * A structure for holding a GPS fix: time, position, velocity.
   */

#ifdef GPS_FIX_LOCATION
    int32_t       lat;  // degree * 1e7, negative is South
    int32_t       lon;  // degree * 1e7, negative is West

    int32_t latitudeL() const { return lat; };
    float latitudeF() const { return ((float) lat) * 1.0e-7; };
    double latitude() const { return ((double) lat) * 1.0e-7; };

    int32_t longitudeL() const { return lon; };
    float longitudeF() const { return ((float) lon) * 1.0e-7; };
    double longitude() const { return ((double) lon) * 1.0e-7; };
#endif

#ifdef GPS_FIX_ALTITUDE
    whole_frac    alt; // .01

    int32_t altitude_cm() const { return alt.int32_00(); };
    float altitudeF() const { return alt.float_00(); };
    double altitude() const { return alt.double_00(); };
#endif

#ifdef GPS_FIX_SPEED
    whole_frac    spd; // .001

    uint32_t speed_mkn() const { return spd.int32_000(); };
    float speed() const { return spd.float_000(); };
#endif

#ifdef GPS_FIX_HEADING
    whole_frac    hdg; //  .01

    uint16_t heading_cd() const { return hdg.int16_00(); };
    float heading() const { return hdg.float_00(); };
#endif

#ifdef GPS_FIX_HDOP
  uint16_t           hdop; // x 1000
#endif

#if defined(GPS_FIX_DATE) | defined(GPS_FIX_TIME)
    struct time_t dateTime;
    uint8_t       dateTime_cs; // hundredths of a second
#endif

  /**
   * The current fix status or mode of the GPS device.  Unfortunately, the NMEA
   * sentences are a little inconsistent in their use of "status" and "mode".
   * Both fields are mapped onto this enumerated type.  Be aware that
   * different manufacturers interpret them differently.  This can cause 
   * problems in sentences which include both types (e.g., GPGLL).
   */
   
  enum status_t {
    STATUS_NONE,
    STATUS_TIME_ONLY,
    STATUS_STD,
    STATUS_DGPS,
    STATUS_EST
  } __attribute__((packed));

  union {
    struct {
      status_t  status    :3;
#ifdef GPS_FIX_SATELLITES
      uint8_t   satellites:5;
#endif
    } __attribute__((packed));
    uint8_t status_satellites;
  } __attribute__((packed));

  //  Flags to indicate which members of this fix are valid.

  union valid_t {
    uint8_t as_byte;
    struct {

#if defined(GPS_FIX_DATE)
      bool date:1;
#endif

#if defined(GPS_FIX_TIME)
      bool time:1;
#endif

#ifdef GPS_FIX_LOCATION
      bool location:1;
#endif

#ifdef GPS_FIX_ALTITUDE
      bool altitude:1;
#endif

#ifdef GPS_FIX_SPEED
      bool speed:1;
#endif

#ifdef GPS_FIX_HEADING
      bool heading:1;
#endif

#ifdef GPS_FIX_SATELLITES
      bool satellites:1;
#endif

#ifdef GPS_FIX_HDOP
      bool hdop:1;
#endif
    } __attribute__((packed));
  } __attribute__((packed))
      valid;

  void init()
  {
#ifdef GPS_FIX_LOCATION
    lat = lon = 0;
#endif

#ifdef GPS_FIX_ALTITUDE
    alt.init();
#endif

#ifdef GPS_FIX_SPEED
    spd.init();
#endif

#ifdef GPS_FIX_HEADING
    hdg.init();
#endif

    status_satellites = 0;

#ifdef GPS_FIX_HDOP
    hdop = 0;
#endif

    valid.as_byte = 0;
  };

    /**
     * Merge valid fields from the right fix into
     *  a "fused" fix on the left (i.e., /this/).
     */

    gps_fix & operator |=( const gps_fix & r )
    {
      if (r.status != STATUS_NONE)
        status = r.status;

#ifdef GPS_FIX_DATE
      if (r.valid.date) {
        dateTime.date  = r.dateTime.date;
        dateTime.month = r.dateTime.month;
        dateTime.year  = r.dateTime.year;
      }
#endif

#ifdef GPS_FIX_TIME
      if (r.valid.time) {
        dateTime.hours = r.dateTime.hours;
        dateTime.minutes = r.dateTime.minutes;
        dateTime.seconds = r.dateTime.seconds;
        dateTime_cs      = r.dateTime_cs;
      }
#endif

#ifdef GPS_FIX_LOCATION
      if (r.valid.location) {
        lat = r.lat;
        lon = r.lon;
      }
#endif

#ifdef GPS_FIX_ALTITUDE
      if (r.valid.altitude)
        alt = r.alt;
#endif

#ifdef GPS_FIX_HEADING
      if (r.valid.heading)
        hdg = r.hdg;
#endif

#ifdef GPS_FIX_SPEED
      if (r.valid.speed)
        spd = r.spd;
#endif

#ifdef GPS_FIX_SATELLITES
      if (r.valid.satellites)
        satellites = r.satellites;
#endif

#ifdef GPS_FIX_HDOP
      if (r.valid.hdop)
        hdop = r.hdop;
#endif

      valid.as_byte |= r.valid.as_byte;

      return *this;
    }

} __attribute__((packed));

#endif