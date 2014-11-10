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
    IOStream::Device *m_device;

    /**
     * Written to by UART driver as soon as a new char is received.
     * Called inside Irq handler.
     */
    virtual int putchar( char c );

    /*
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

          void from_cm_per_s( uint32_t cm_per_s )
          {
            uint32_t nm00000_per_h = (cm_per_s * 1852) / 3600;
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

    enum gps_fix_status_t {
      GPS_FIX_NONE = 0,
      GPS_FIX_STD  = 1,
      GPS_FIX_DGPS = 2,
      GPS_FIX_EST  = 6
    } __attribute__((packed));

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

        union gps_fix_valid_t {
          uint8_t all;
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
          valid.all = 0;
        };

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
          valid.all |= r.valid.all;
          return *this;
        }

        int32_t latitudeL() const { return position.lat; };
        float latitude() const { return ((float) position.lat) * 1.0e-7; };

        int32_t longitudeL() const { return position.lon; };
        float longitude() const { return ((float) position.lon) * 1.0e-7; };

        int32_t altitude_cm() const { return position.alt.in_cm(); };
        float altitude() const { return ((float) altitude_cm()) * 0.01; };

        uint32_t speed_mkn() const { return velocity.spd.in_mkn(); };
        float speed() const { return ((float)speed_mkn()) * 0.001; };

        uint16_t heading_cd() const { return velocity.hdg.in_cd(); };
        float heading() const { return ((float)heading_cd()) * 0.01; };

    };

    volatile const struct gps_fix_t & fix() const { return m_fix; };


    /**
     * Internal GPS parser statistics.
     */
#ifdef NEOGPS_STATS
    volatile struct {
        uint8_t  parser_ok;     // count of successfully parsed packets
        uint8_t  parser_crcerr; // count of CRC errors
    } statistics;
#endif

    void poll( nmea_msg_t msg ) const;

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
