#ifndef NMEAGPS_H
#define NMEAGPS_H

/**
 * @file NMEAGPS.h
 * @version 2.1
 *
 * @section License
 * Copyright (C) 2014, SlashDevin
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
#include "Cosa/Time.hh"

/**
 * GPS Parser for NEO-6 Modules.
 *
 * @section Limitations
 * Very limited support for NMEA messages.
 * Only NMEA messages of types GGA, GLL, RMC, VTG, ZDA are parsed.
 */

/**
 * Enable/disable the parsing of specific sentences.
 *
 * Note: Only RMC and ZDA contain date information.  Other
 * sentences contain time information.  Both date and time are 
 * required if you will be doing time_t-to-clock_t operations.
 */

#define NMEAGPS_PARSE_GGA
//#define NMEAGPS_PARSE_GLL
//#define NMEAGPS_PARSE_GSA
//#define NMEAGPS_PARSE_GSV
#define NMEAGPS_PARSE_RMC
//#define NMEAGPS_PARSE_VTG
#define NMEAGPS_PARSE_ZDA

class NMEAGPS
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
     * Current parser state
     */
    uint8_t         crc;
    uint8_t         fieldIndex;
    uint8_t         chrCount;  // index of current character in current field
    uint8_t         decimal; // digits after the decimal point
    bool            negative;

    enum rxState_t {
        NMEA_IDLE,
        NMEA_RECEIVING_DATA,
        NMEA_RECEIVING_CRC1,
        NMEA_RECEIVING_CRC2
    };
    static const uint8_t NMEA_FIRST_STATE = NMEA_IDLE;
    static const uint8_t NMEA_LAST_STATE  = NMEA_RECEIVING_CRC2;

    rxState_t rxState;

public:

    /**
     * Constructor
     */
    NMEAGPS( IOStream::Device *device = (IOStream::Device *) NULL )
    {
      m_device = device;
      rxState = NMEA_IDLE;
    };

    /**
     * Process one character of an NMEA GPS sentence.  The  internal state machine
     * tracks what part of the sentence has been received so far.  As the
     * sentence is received, members of the /fix/ structure are updated.  
     * @return true when new /fix/ data is available and coherent.
     */
    bool decode( char c );

    /**
     * Most recent NMEA sentence type received.  This is not necessarily
     * related to the current /fix/.
     */
    enum nmea_msg_t nmeaMessage;

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
    };

    /**
     * The current fix status or mode of the GPS device.  Unfortunately, the NMEA
     * sentences are a little inconsistent in their use of "status" and "mode".
     * Both fields are mapped onto this enumerated type.  Be aware that
     * different manufacturers interpret them differently.  This can cause 
     * problems in sentences which include both types (e.g., GPGLL).
     */
     
    enum gps_fix_status_t {
      GPS_FIX_NONE = 0,
      GPS_FIX_STD  = 1,
      GPS_FIX_DGPS = 2,
      GPS_FIX_EST  = 6
    } __attribute__((packed));

    /**
     * A structure for holding a GPS fix: time, position, velocity.
     */

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

        /**
         * Merge valid fields from the right fix into
         *  a "fused" fix on the left (i.e., /this/).
         */

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
        float latitudeF() const { return ((float) lat) * 1.0e-7; };
        double latitude() const { return ((double) lat) * 1.0e-7; };

        int32_t longitudeL() const { return lon; };
        float longitudeF() const { return ((float) lon) * 1.0e-7; };
        double longitude() const { return ((double) lon) * 1.0e-7; };

        int32_t altitude_cm() const { return alt.int32_00(); };
        float altitudeF() const { return ((float) altitude_cm()) * 0.01; };
        double altitude() const { return ((double) altitude_cm()) * 0.01; };

        uint32_t speed_mkn() const { return spd.int32_000(); };
        float speed() const { return ((float)speed_mkn()) * 0.001; };

        uint16_t heading_cd() const { return hdg.int16_00(); };
        float heading() const { return ((float)heading_cd()) * 0.01; };

    };
    
    //  Current fix accessor.
    //  /fix/ will be constantly changing as characters are received.
    //  For example, fix().longitude() may return nonsense data if
    //  characters for that field are currently being processed in /decode/.
    //  /is_coherent/ *must* be checked before accessing members of /fix/.
    //  If you need access to the current /fix/ at any time, you must
    //  take a snapshot while it is_coherent, and then use the snapshot
    //  later.

    const struct gps_fix_t & fix() const { return m_fix; };

    //  Determine whether the members of /fix/ are "currently" coherent.
    //  It will return true when a complete sentence and the CRC characters 
    //  have been received (or after a CR if no CRC is present).
    //  It will return false after a sentence's command and comma
    //  have been received (e.g., "$GPGGA,").
    //  If NMEAGPS processes characters in UART interrupts, /is_coherent/
    //  could change at any time.

    bool is_coherent() const { return (m_fix.valid.as_byte != 0); }

    //  Notes regarding the volatile /fix/:
    //
    //  If an NMEAGPS instance is fed characters from a non-interrupt
    //  context, the following method is safe:
    //
    //  void loop()
    //  {
    //    while (uart.available()) {
    //      if (gps.decode( uart.getchar() )) {
    //        // Got something new!
    //        // Access only valid members and/or save a snapshot for later
    //        safe_fix = gps.fix();
    //      }
    //    }
    //    // Access valid members of /safe_fix/ anywhere, any time.
    //  }

    /**
     * Internal GPS parser statistics.
     */
#ifdef NMEAGPS_STATS
    struct {
        uint8_t  parser_ok;     // count of successfully parsed packets
        uint8_t  parser_crcerr; // count of CRC errors
    } statistics;
#endif

    /**
     * Request the specified NMEA sentence.  Not all devices will respond.
     */

    void poll( nmea_msg_t msg ) const;

    /**
     * Send a message to the GPS device.
     * The '$' is optional, and the '*' and CS will be added automatically.
     */

    void send( const char *msg ) const;
    void send_P( const char *msg ) const;

protected:
    //  Current fix
    struct gps_fix_t m_fix;

private:
    void rxBegin();
    void rxEnd( bool ok );
    
    bool parseField( char chr );
    bool parseCommand( char chr );
    bool parseTimeField( char chr );
    bool parseFix( char chr );
    void parseFloat( whole_frac & val, char chr, uint8_t max_decimal );
    void parseDDMM( int32_t & val, char chr );

    bool send_header( const char * & msg ) const;
    void send_trailer( uint8_t crc ) const;
};

#endif
