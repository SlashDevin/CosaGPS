#ifndef NeoGPS_h
#define NeoGPS_h

/**
 * @file NeoGPS.h
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
 * Very limited support for NMEA and UBX messages.
 * Only NMEA messages of types ZDA and GGA are parsed.
 * UBX interface only allows basic configuration and 
 * does not check CRC on Acknowledge messages.
 */

class NeoGPS : public IOStream::Device, public Event::Handler
{
public:

    /**
     * Struct for storing location data from GPS
     */
    struct Location {
        int32_t               lat; // degree * 1e6, negative is South
        int32_t               lon; // degree * 1e6, negative is West
        uint16_t              alt;
    } __attribute__((packed));

    enum gps_fix_status_t {
      GPS_FIX_NONE = 0,
      GPS_FIX_STD  = 1,
      GPS_FIX_DGPS = 2,
      GPS_FIX_EST  = 6
    } __attribute__((packed));

    struct {
        enum gps_fix_status_t stat     :3;
        uint8_t               satCnt   :5;
    } __attribute__((packed));

    volatile uint8_t         hdop;

    volatile struct Location location;
    volatile struct time_t   dateTime;    // BCD
    volatile uint8_t         dateTime_cs; // BCD hundredths of a second

    volatile uint8_t  speed;        // BCD knots
    volatile uint16_t speed_frac;   // BCD knots * 1000
    uint16_t get_speed() const
    {
      uint16_t  val    =
        ((uint16_t)to_binary( speed_frac>>8 ))*100 +
        ((uint16_t)to_binary( speed_frac ));
      return ((uint16_t)to_binary(speed))*1000 + val;
    };
    
    volatile uint16_t heading;      // BCD degrees
    volatile uint8_t  heading_frac; // BCD degrees * 100
    uint16_t get_heading() const
    {
      uint16_t  val    =
        ((uint16_t)to_binary( heading>>8 ))*100 +
        ((uint16_t)to_binary( heading ));
      return val*100 + ((uint16_t)to_binary(heading_frac));
    };

    volatile union {
      uint8_t all;
      struct {
        bool dateTime:1;
        bool location:1;
        bool speed:1;
        bool heading:1;
      } __attribute__((packed));
    } __attribute__((packed))
        valid;

    /**
     * Internal GPS parser statistics.
     */
#ifdef NEOGPS_STATS
    volatile struct {
        uint8_t  parser_ok;     // count of successfully parsed packets
        uint8_t  parser_crcerr; // count of CRC errors
    } statistics;
#endif

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
     * Constructor
     */
    NeoGPS( IOStream::Device *device = (IOStream::Device *) NULL )
    {
      m_device = device;
      valid.all = 0;
      nmeaMessage = NMEA_UNKNOWN;
      rxState = NMEA_IDLE;
    };

    void send( const char *msg ); // '$' is optional, and '*' and CS added
protected:
    IOStream::Device *m_device;

    /**
     * Written to by UART driver as soon as a new char is received.
     * Called inside Irq handler.
     */
    int putchar( char c );

    /**
     * Internal FSM state.
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
              

    /*
     * Current internal parser state
     */
    volatile uint8_t crc;
    uint8_t chrCount;  // index of current character in current field

private:
    struct {
      enum nmea_msg_t nmeaMessage   :7;
      bool            before_decimal:1;
    } __attribute__((packed));
    
    void rxBegin();
    void rxEnd( bool ok );

    bool parseField(char chr);
    uint8_t fieldIndex;     // index of currently receiving field
    bool parseTimeField(char chr);

    // parse lat/lon dddmm.mmmm fields

    void parseDDMM( volatile int32_t & val, char chr )
    {
      if (chrCount == 0) {
        val = 0;
        before_decimal = true;
      }
      
      if (chr == '.') {
        before_decimal = false;
        uint8_t *valBCD = (uint8_t *) const_cast<int32_t *>( &val );
        uint8_t  deg     = to_binary( valBCD[2] )*10 + to_binary( valBCD[1] );
        val = (deg * 60) + to_binary( valBCD[0] );
        // val now in units of minutes

      } else if (before_decimal)
          // val is BCD until *after* decimal point
          val = (val<<4) | (chr - '0');
      else 
          val = val*10 + (chr - '0');
    }

    void endDDMM( volatile int32_t & val )
    {
      // was in minutes x 100000, now in degrees x 1000000
      val = (val+3)/6;
    }

};

#endif
