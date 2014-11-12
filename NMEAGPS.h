#ifndef NMEAGPS_h
#define NMEAGPS_h

/**
 * @file NMEAGPS.h
 * @version 2.0
 *
 * @section License
 * Copyright (C) 2014, Thomas Lohmueller
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

// Include core libraries
#include "Cosa/Watchdog.hh"
#include "Cosa/Event.hh"
#include "Cosa/Time.hh"

/**
 * GPS Parser for NEO-6 Modules.
 *
 * @section Limitations
 * Very limited support for NMEA messages.
 * Only NMEA messages of types GGA, GLL, RMC, VTG, ZDA are parsed.
 */

class NMEAGPS : public IOStream::Device, public Event::Handler
{
public:
    /** NMEA message types. */
    enum nmea_msg_t {
        NMEA_UNKNOWN,
        NMEA_GGA,
        NMEA_GLL,
        NMEA_GSA,
        NMEA_GSV,
        NMEA_RMC,
        NMEA_VTG,
        NMEA_ZDA,
    } __attribute__((packed));
    static const nmea_msg_t NMEA_FIRST_MSG = NMEA_GGA;
    static const nmea_msg_t NMEA_LAST_MSG  = NMEA_ZDA;
    
protected:
    IOStream::Device *m_device; // required for transmitting *to* the device

    /**
     * Written to by UART driver as soon as a new char is received.
     * Called inside Irq handler.
     */
    virtual int putchar( char c );

    /**
     * Current parser state
     */
    uint8_t         crc;
    uint8_t         fieldIndex;
    uint8_t         chrCount;  // index of current character in current field
    enum nmea_msg_t nmeaMessage:4;
    uint8_t         decimal:3; // digits after the decimal point

    /**
     * Internal FSM.
     */
    enum rxState_t {
        NMEA_IDLE,
        NMEA_WAIT_FOR_COMMAND,
        NMEA_RECEIVING_DATA,
        NMEA_RECEIVING_CRC1,
        NMEA_RECEIVING_CRC2
    };
    volatile rxState_t rxState;
    static const uint8_t NMEA_LAST_STATE = NMEA_RECEIVING_CRC2;

    bool receiving() const
    {
      return (rxState != NMEA_IDLE) || (m_device && m_device->available());
    }

public:

    /**
     * Constructor
     */
    NMEAGPS( IOStream::Device *device = (IOStream::Device *) NULL )
    {
      m_device = device;
      rxState = NMEA_IDLE;
    };

    //  A structure for holding the 3D position: latitude, longitude and altitude.

    struct position_t {
        int32_t    lat; // degree * 1e7, negative is South
        int32_t    lon; // degree * 1e7, negative is West

        struct alt_t {
          uint16_t m;
          union {
            struct {
              uint8_t  cm:7;
              bool     negative:1;
            } __attribute__((packed));
            uint8_t all;
          } __attribute__((packed));
          
          void init() volatile 
          {
            m   = 0;
            all = 0; // cm = 0, negative = false
          }
          
          int32_t in_cm() const volatile
            { return (negative ? -1L : 1L) * (m*100UL + cm); };
        } alt;

        void init()
        {
          lat = 0;
          lon = 0;
          alt.init();
        };

    } __attribute__((packed));

    //  A structure for holding the velocity: speed and heading.
    
    struct velocity_t {
    
      struct spd_t {
          uint8_t  knots;
          uint16_t milli_knots;

          void init() volatile 
          {
            knots = 0;
            milli_knots = 0;
          }

          uint32_t in_mkn() const volatile     // knots * 1000
            { return ((uint32_t) knots)*1000L + milli_knots; };
          uint32_t in_cm_per_s() const volatile;

          void from_cm_per_s( uint32_t cm_per_s )
          {
            uint32_t nm00000_per_h = (cm_per_s * NM_PER_KM000) / 3600;
            knots = nm00000_per_h / 100000UL;
            milli_knots = (nm00000_per_h - knots * 100000) / 100;
          }

      } spd;

      struct hdg_t {
          union {
            struct {
              uint16_t degrees:9;
              uint8_t  centi_degrees:7;
            } __attribute__((packed));
            uint16_t all;
          } __attribute__((packed));

          void init() volatile { all = 0; };

          uint16_t in_cd() const volatile     // degrees * 100
            { return degrees*100 + centi_degrees; };

      } hdg;

      void init() volatile { spd.init(); hdg.init(); };

    } __attribute__((packed));

    static const float NM_PER_KM = 1.852;
    static const float KM_PER_NM = 1.0 / 1.852;
    static const uint16_t NM_PER_KM000 = 1852;

    enum gps_fix_status_t {
      GPS_FIX_NONE = 0,
      GPS_FIX_STD  = 1,
      GPS_FIX_DGPS = 2,
      GPS_FIX_EST  = 6
    } __attribute__((packed));

    //  A structure for holding a GPS fix: time, position, velocity.

    struct gps_fix_t {
        struct position_t position;
        struct velocity_t velocity;
        struct time_t     dateTime;
        uint8_t           dateTime_cs; // hundredths of a second
        union {
          struct {
            gps_fix_status_t  status    :3;
            uint8_t           satellites:5;
          } __attribute__((packed));
          uint8_t status_satellites;
        } __attribute__((packed));
        uint8_t           hdop;

        //  Flags to indicate which members of this fix are valid.

        union gps_fix_valid_t {
          uint8_t as_byte;
          struct {
            bool dateTime:1;
            bool location:1;
            bool altitude:1;
            bool speed:1;
            bool heading:1;
          } __attribute__((packed));
        } __attribute__((packed))
            valid;

        gps_fix_t() { init(); };

        void init()
        {
          position.init();
          velocity.init();
          status_satellites = 0;
          hdop = 0;
          valid.as_byte = 0;
        };

        //  This operator allows merging valid fields from the right fix into
        //  a "fused" fix on the left (i.e., /this/).

        gps_fix_t & operator |=( const gps_fix_t & r )
        {
          if (r.status != GPS_FIX_NONE)
            status = r.status;
          if (r.hdop != 0)
            hdop = r.hdop;
          if (r.satellites != 0)
            satellites = r.satellites;
          if (r.valid.dateTime)
            dateTime = r.dateTime;
          if (r.valid.location) {
            position.lat = r.position.lat;
            position.lon = r.position.lon;
          }
          if (r.valid.altitude)
            position.alt = r.position.alt;
          if (r.valid.heading)
            velocity.hdg = r.velocity.hdg;
          if (r.valid.speed)
            velocity.spd = r.velocity.spd;
          valid.as_byte |= r.valid.as_byte;
          return *this;
        }

        //  Convenience accessors for fix members.

        int32_t latitudeL() const { return position.lat; };
        float latitude() const { return ((float) position.lat) * 1.0e-7; };

        int32_t longitudeL() const { return position.lon; };
        float longitude() const { return ((float) position.lon) * 1.0e-7; };

        int32_t altitude_cm() const { return position.alt.in_cm(); };
        float altitude() const { return ((float) altitude_cm()) * 0.01; };

        uint32_t speed_mkn() const { return velocity.spd.in_mkn(); };
        uint32_t speed_cm_per_s() const { return velocity.spd.in_cm_per_s(); };
        float speed() const { return ((float)speed_mkn()) * 0.001; };
        float speed_kph() const { return speed() * KM_PER_NM; };

        uint16_t heading_cd() const { return velocity.hdg.in_cd(); };
        float heading() const { return ((float)heading_cd()) * 0.01; };

    };
    
    //  Current fix state accessor.
    //  /fix/ will be constantly changing as characters are received.
    //  For example, fix().longitude() may return nonsense data if
    //  characters for that field are currently being processed in /putchar/.
    //  /is_coherent/ *must* be checked before accessing members of /fix/.
    //  If you need access to the current /fix/ at any time, you must
    //  take a snapshot while it is_coherent, and then use the snapshot
    //  later.

    volatile const struct gps_fix_t & fix() const { return m_fix; };

    //  Determine whether the members of /fix/ are "currently" coherent.
    //  It will return true when a complete sentence and the CRC characters 
    //  are received (or after a CR if no CRC is present).
    //  It will return false after a sentence's command and comma
    //  are received (e.g., "$GPGGA,").
    //  If this instance is connected to UART interrupts, /is_coherent/
    //  could change at any time.

    bool is_coherent() const { return (m_fix.valid.as_byte != 0); }

    //  Notes regarding the volatile /fix/:
    //
    //  The time window for accessing a coherent /fix/ is fairly narrow, 
    //  about 9 character times, or about 10mS on a 9600-baud connection.
    //  There are several ways to safely access /fix/ and its members:
    //
    //  If an NMEAGPS instance is hooked directly to the UART
    //    so that it processes characters in the interrupt, these methods 
    //    will work:
    //
    //  1) void loop()
    //     {
    //       synchronized {
    //         if (gps.is_coherent()) {
    //           // access only valid members and/or
    //           // save a snapshot for later
    //           safe_fix = fix();
    //          }
    //       }
    //       // access only valid members of /safe_fix/ here
    //     }
    //
    //  2) void derived_NMEAGPS::on_event( uint8_t type, uint16_t value )
    //     {
    //       synchronized {
    //         if (is_coherent()) {
    //           // access only valid members and/or
    //           // save a snapshot for later
    //           safe_fix = fix();
    //         }
    //       }
    //       // access only valid members of /safe_fix/ here
    //     }
    //     This is susceptible to event processing delays; other kinds of
    //     events may delay getting to /fix/ while it is still coherent.
    //
    //  Or, if an NMEAGPS instance is fed characters from a non-interrupt
    //  context, the following method will work:
    //
    //  void loop()
    //  {
    //    bool was_ok = gps.is_coherent();
    //    while (uart.available()) {
    //      gps.putchar( uart.getchar() );
    //      if (gps.is_coherent()) {
    //        if (!was_ok) {
    //          //  Got something new!
    //          // access only valid members and/or
    //          // save a snapshot for later
    //          safe_fix = fix();
    //          was_ok = true;
    //        }
    //      } else
    //        was_ok = false;
    //    }
    //    // access only valid members of /safe_fix/ here.
    //  }

    /**
     * Internal GPS parser statistics.
     */
#ifdef NEOGPS_STATS
    volatile struct {
        uint8_t  parser_ok;     // count of successfully parsed packets
        uint8_t  parser_crcerr; // count of CRC errors
    } statistics;
#endif

    //  Request the specified NMEA sentence

    void poll( nmea_msg_t msg ) const;

    // Send a message to the GPS device

    void send( const char *msg ) const; // '$' is optional, and '*' and CS added
    void send_P( const char *msg ) const; // '$' is optional, and '*' and CS added

protected:
    //  Current fix state
    volatile struct gps_fix_t m_fix;

private:
    void rxBegin();
    void rxEnd( bool ok );
    
    bool parseField( char chr );
    bool parseTimeField( char chr );
    bool parseFixMode( char chr );
    bool parseFixStatus( char chr );
    void parseSpeed( char chr );
    void parseHeading( char chr );
    void parseAltitude( char chr );

    // parse lat/lon dddmm.mmmm fields

    void parseDDMM( volatile int32_t & val, char chr )
    {
      if (chrCount == 0) {
        val = 0;
        decimal = 0;
      }
      
      if (chr == '.') {
        decimal = 1;
        uint8_t *valBCD = (uint8_t *) const_cast<int32_t *>( &val );
        uint8_t  deg     = to_binary( valBCD[2] )*10 + to_binary( valBCD[1] );
        val = (deg * 60) + to_binary( valBCD[0] );
        // val now in units of minutes

      } else if (!decimal)
        // val is BCD until *after* decimal point
        val = (val<<4) | (chr - '0');
      else if (decimal++ < 6)
        val = val*10 + (chr - '0');
    }

    void endDDMM( volatile int32_t & val )
    {
      if (val) {
        // If the last chars in ".mmmm" were not received,
        //    force the value into its final state.
        if (!decimal)
          parseDDMM( val, '.' );
        while (decimal++ < 6)
          val *= 10;

        // Value was in minutes x 1000000, convert to degrees x 10000000.
        val += (val*2 + 1)/3; // aka (10*val+3)/6, but without sign truncation
      }
    }

    bool send_header( const char * & msg ) const;
    void send_trailer( uint8_t crc ) const;
};

#endif
