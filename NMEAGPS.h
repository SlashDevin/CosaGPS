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
    
    /**
     * Written to by UART driver as soon as a new char is received.
     * Called inside Irq handler.
     */
    virtual int putchar( char c );

protected:
    IOStream::Device *m_device; // required for transmitting *to* the device

    /**
     * Current parser state
     */
    uint8_t         crc;
    uint8_t         fieldIndex;
    uint8_t         chrCount;  // index of current character in current field
    enum nmea_msg_t nmeaMessage;
    uint8_t         decimal; // digits after the decimal point
    bool            negative;

    /**
     * Internal FSM.
     */
    enum rxState_t {
        NMEA_IDLE,
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

    // A structure for holding the two parts of a floating-point number.

    struct whole_frac {
      int16_t whole;
      int16_t frac;
      void init() { whole = 0; frac = 0; };
      int32_t int32_00() const { return ((int32_t)whole) * 100L + frac; };
      int16_t int16_00() const { return whole * 100 + frac; };
      int32_t int32_000() const { return whole * 1000L + frac; };
    };

    enum gps_fix_status_t {
      GPS_FIX_NONE = 0,
      GPS_FIX_STD  = 1,
      GPS_FIX_DGPS = 2,
      GPS_FIX_EST  = 6
    } __attribute__((packed));

    //  A structure for holding a GPS fix: time, position, velocity.

    struct gps_fix_t {
        int32_t       lat;  // degree * 1e7, negative is South
        int32_t       lon;  // degree * 1e7, negative is West
        whole_frac    alt;  // m*100
        whole_frac    spd;  // knots*1000
        whole_frac    hdg;  // degrees*100
        struct time_t dateTime;
        uint8_t       dateTime_cs; // hundredths of a second
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
          lat = lon = 0;
          alt.init();
          spd.init();
          hdg.init();
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
            lat = r.lat;
            lon = r.lon;
          }
          if (r.valid.altitude)
            alt = r.alt;
          if (r.valid.heading)
            hdg = r.hdg;
          if (r.valid.speed)
            spd = r.spd;
          valid.as_byte |= r.valid.as_byte;
          return *this;
        }

        //  Convenience accessors for fix members.

        int32_t latitudeL() const { return lat; };
        float latitude() const { return ((float) lat) * 1.0e-7; };

        int32_t longitudeL() const { return lon; };
        float longitude() const { return ((float) lon) * 1.0e-7; };

        int32_t altitude_cm() const { return alt.int32_00(); };
        float altitude() const { return ((float) altitude_cm()) * 0.01; };

        uint32_t speed_mkn() const { return spd.int32_000(); };
        float speed() const { return ((float)speed_mkn()) * 0.001; };

        uint16_t heading_cd() const { return hdg.int16_00(); };
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
    bool parseCommand( char chr );
    bool parseTimeField( char chr );
    bool parseFix( char chr );
    void parseFloat( whole_frac & val, char chr, uint8_t max_decimal );

    // parse lat/lon dddmm.mmmm fields

    void parseDDMM( volatile int32_t & val, char chr )
    {
      if (chrCount == 0) {
        val = 0;
        decimal = 0;
      }
      
      if ((chr == '.') || ((chr == ',') && !decimal)) {
        // Now we know how many digits are in degrees, so we
        // can use the appropriate digits and switch from BCD to binary.
        // (The last two digits are always minutes.)
        decimal = 1;
        uint8_t *valBCD = (uint8_t *) const_cast<int32_t *>( &val );
        uint8_t  deg     = to_binary( valBCD[2] )*10 + to_binary( valBCD[1] );
        val = (deg * 60) + to_binary( valBCD[0] );
        // val now in units of minutes
      }
      
      if (chr == ',') {
        if (val) {
          // If the last chars in ".mmmm" were not received,
          //    force the value into its final state.
          while (decimal++ < 6)
            val *= 10;

          // Value was in minutes x 1000000, convert to degrees x 10000000.
          val += (val*2 + 1)/3; // aka (100*val+30)/60, but without sign truncation
        }
      } else if (!decimal)
        // val is BCD until *after* decimal point
        val = (val<<4) | (chr - '0');
      else if (decimal++ < 6)
        val = val*10 + (chr - '0');
    }

    bool send_header( const char * & msg ) const;
    void send_trailer( uint8_t crc ) const;
};

#endif
