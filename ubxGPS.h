#ifndef _UBXGPS_H_
#define _UBXGPS_H_

#include <avr/pgmspace.h>

#include "Cosa/Trace.hh"

/*
 * Enable/disable Neo-6 protocols.  Either or both can be enabled
 */
//#define UBLOX_PARSE_NMEA
#define UBLOX_PARSE_UBLOX

/**
 * Enable/disable the parsing of specific NMEA sentences.
 *
 * Configuring out a sentence prevents its fields from being parsed.
 * However, the sentence type will still be recognized by /decode/ and 
 * stored in member /nmeaMessage/.  No valid flags will be true.
 *
 */
#define NMEAGPS_PARSE_PUBX_00
#define NMEAGPS_PARSE_PUBX_04

/**
 * Enable/disable the parsing of specific UBX messages.
 *
 * Configuring out a message prevents its fields from being parsed.
 * However, the message type will still be recognized by /decode/ and 
 * stored in member /nmeaMessage/.  No valid flags will be true.
 */

#define UBLOX_PARSE_STATUS
#define UBLOX_PARSE_TIMEGPS
#define UBLOX_PARSE_TIMEUTC
#define UBLOX_PARSE_POSLLH
#define UBLOX_PARSE_VELNED
//#define UBLOX_PARSE_CFGNAV5
//#define UBLOX_PARSE_MONVER
 
//----------------------------

#ifdef UBLOX_PARSE_NMEA
#define UBLOX_INHERITANCE : public NMEAGPS

#include "NMEAGPS.h"
#ifndef NMEAGPS_DERIVED_TYPES
#error You must "#define NMEAGPS_DERIVED_TYPES" in NMEAGPS.h!
#endif

#else

#define UBLOX_INHERITANCE
#include "GPSfix.h"

#endif

#ifdef  UBLOX_PARSE_UBLOX

#include "ubxmsg.h"

#include "Cosa/Power.hh"

// NOTE: RTC::millis() is used for ACK timing

#endif

class ubloxGPS UBLOX_INHERITANCE
{
    ubloxGPS( const ubloxGPS & );

public:

#ifndef UBLOX_PARSE_NMEA
    enum decode_t { DECODE_CHR_INVALID, DECODE_CHR_OK, DECODE_COMPLETED };
#endif

#ifndef UBLOX_PARSE_UBLOX
    ubloxGPS() {};
#else
    ubloxGPS( IOStream::Device *device )
      :
        storage( (ublox::msg_t *) NULL ),
        reply( (ublox::msg_t *) NULL ),
        reply_expected( false ),
        ack_expected( false ),
        m_device( device )
      {};

    /**
     * Process one character from a ublox device.  The internal state machine
     * tracks what part of the NMEA/UBX packet has been received so far.  As the
     * packet is received, members of the /fix/ structure are updated.  
     * @return true when new /fix/ data is available and coherent.
     */
    decode_t decode( char c );

    /**
     * Received message header.  Payload is only stored if /storage/ is 
     * overridden for that message type.
     */
    ublox::msg_t & rx() { return m_rx_msg; }

#if defined(GPS_FIX_DATE) | defined(GPS_FIX_TIME)
    /**
     * GPS time is offset from UTC by a number of leap seconds.  To convert a GPS
     * time to UTC time, the current number of leap seconds must be known.
     * See http://en.wikipedia.org/wiki/Global_Positioning_System#Leap_seconds
     */
    static uint8_t leap_seconds;

    /**
     * Some receivers report time WRT start of the current week, defined as
     * Sunday 00:00:00.  To save fairly expensive date/time calculations,
     * the UTC start of week is cached
     */
    static void start_of_week( time_t & now )
      {
        now.set_day();
        s_start_of_week =
          (clock_t) now  -  
          (clock_t) ((((now.day-1)*24L + 
                       now.hours)*60L + 
                       now.minutes)*60L +
                       now.seconds);
      }

    static clock_t start_of_week()
    {
      return s_start_of_week;
    }

    /*
     * Convert a GPS time-of-week to UTC.
     * Requires /leap_seconds/ and /start_of_week/.
     */
    static clock_t TOW_to_UTC( uint32_t gps_time_of_week )
      { return (clock_t) (s_start_of_week + gps_time_of_week - leap_seconds); }

    /**
     * Set /fix/ timestamp from a GPS time-of-week in milliseconds.
     * Requires /leap_seconds/ and /start_of_week/.
     **/
    bool from_TOWms( uint32_t time_of_week_ms )
    {
//trace << PSTR("from_TOWms(") << time_of_week_ms << PSTR("), sow = ") << start_of_week() << PSTR(", leap = ") << leap_seconds << endl;
      bool ok = (start_of_week() != 0) && (leap_seconds != 0);
      if (ok) {
        time_of_week_ms /= 10UL;
        clock_t tow_s = time_of_week_ms/100UL;
        m_fix.dateTime = TOW_to_UTC( tow_s ); 
        m_fix.dateTime_cs = ((uint8_t)(time_of_week_ms - tow_s*100UL)) % 100;
        m_fix.valid.time = true;
        m_fix.valid.date = true;
      }
      return ok;
    }
#endif

    /**
     * Send a message (non-blocking).
     *    Although multiple /send_request/s can be issued,
     *      replies will cause multiple dispatches to /on_event/
     */
    bool send_request( const ublox::msg_t & msg )
    {
      write( msg );
      return true;
    };
    bool send_request_P( const ublox::msg_t & msg )
    {
      write_P( msg );
      return true;
    };

    /**
     * Send a message and wait for a reply (blocking).
     *    No event will be generated for the reply.
     *    If /msg/ is a UBX_CFG, this will wait for a UBX_CFG_ACK/NAK
     *      and return true if ACKed.
     *    If /msg/ is a poll, this will wait for the reply.
     *    If /msg/ is neither, this will return true immediately.
     *    If /msg/ is both, this will wait for both the reply and the ACK/NAK.
     *    If /storage_for/ is implemented, those messages will continue
     *      to be saved while waiting for this reply.
     */
    bool send( const ublox::msg_t & msg, ublox::msg_t *reply_msg = (ublox::msg_t *) NULL );
    bool send_P( const ublox::msg_t & msg, ublox::msg_t *reply_msg = (ublox::msg_t *) NULL );

    //  Ask for a specific message (non-blocking).
    //     /on_event/ will receive the header later.
    //  See also /send_request/.
    bool poll_request( const ublox::msg_t & msg )
    {
//trace << '?' << msg.msg_class << '/' << msg.msg_id << ' ';
      ublox::msg_t poll_msg( msg.msg_class, msg.msg_id, 0 );
      return send_request( poll_msg );
    };
    bool poll_request_P( const ublox::msg_t & msg )
    {
      ublox::msg_t poll_msg( (ublox::msg_class_t) pgm_read_byte( &msg.msg_class ),
                      (ublox::msg_id_t) pgm_read_byte( &msg.msg_id ), 0 );
      return send_request( poll_msg );
    };

    //  Ask for a specific message (blocking).
    //    See also /send/.
    bool poll( ublox::msg_t & msg )
    {
      ublox::msg_t poll_msg( msg.msg_class, msg.msg_id, 0 );
      return send( poll_msg, &msg );
    };
    bool poll_P( const ublox::msg_t & msg, ublox::msg_t *reply_msg = (ublox::msg_t *) NULL )
    {
      ublox::msg_t poll_msg( (ublox::msg_class_t) pgm_read_byte( &msg.msg_class ),
                      (ublox::msg_id_t) pgm_read_byte( &msg.msg_id ), 0 );
      return send( poll_msg, reply_msg );
    };

#ifdef UBLOX_PARSE_NMEA
#define UBX_IDLE_VALUE NMEA_IDLE
#define UBX_FIRST_VALUE NMEA_LAST_STATE+1
#else
#define UBX_IDLE_VALUE 0
#define UBX_FIRST_VALUE 1
#endif

protected:

    enum ubxState_t {
        UBX_IDLE  = UBX_IDLE_VALUE,
        UBX_SYNC2 = UBX_FIRST_VALUE,
        UBX_HEAD,
        UBX_RECEIVING_DATA,
        UBX_CRC_A,
        UBX_CRC_B
    };
    static const ubxState_t UBX_FIRST_STATE = UBX_SYNC2;
    static const ubxState_t UBX_LAST_STATE  = UBX_CRC_B;

#ifndef UBLOX_PARSE_NMEA
public:
    const struct gps_fix & fix() const { return m_fix; };
private:
    ubxState_t rxState;
    typedef ubxState_t rxState_t;
    uint8_t         chrCount;  // index of current character in current field
protected:
    struct gps_fix m_fix;
    bool            coherent:1;
#endif

    inline void write
      ( uint8_t c, uint8_t & crc_a, uint8_t & crc_b ) const
    {
        m_device->putchar( c );
        crc_a += c;
        crc_b += crc_a;
    };
    void write( const ublox::msg_t & msg );
    void write_P( const ublox::msg_t & msg );

    void wait_for_idle();
    bool wait_for_ack();
    bool waiting() const
    {
      return (ack_expected && (!ack_received && !nak_received)) ||
             (reply_expected && !reply_received);
    }
    bool receiving() const
    {
      return (rxState != UBX_IDLE) || (m_device && m_device->available());
    }

    //  This method is called from inside /wait_for_idle/.
    //
    //  If a derived class processes incoming chars in the background
    //  (e.g., a derived IOStream::Device::/putchar/ called from the IRQ),
    //  this default /delay/ method is sufficient.
    //
    //  If /this/ instance processes characters in the foreground,
    //  /run/ must be provided to continue decoding the input stream while 
    //  waiting for UBX replies.  For example,
    //            while (uart.available())
    //              decode( uart.getchar() );

    virtual void run() { Power::sleep(); };

    // Derived class should override this if the contents of a
    //   particular message need to be saved.
    // This executes in an interrupt context, so be quick!
    //  NOTE: the ublox::msg_t.length will get stepped on, so you may need to
    //  set it every time if you are using a union for your storage.
    virtual ublox::msg_t *storage_for( const ublox::msg_t & rx_msg )
      { return (ublox::msg_t *)NULL; };

private:
    ublox::msg_t   *storage;   // cached ptr to hold a received msg.

    // Storage for a specific received message.
    //   Used internally by send & poll variants.
    //   Checked and used before /storage_for/ is called.
    ublox::msg_t   *reply;

    struct {
      bool     reply_expected:1;
      bool     reply_received:1;
      bool     ack_expected:1;
      bool     ack_received:1;
      bool     nak_received:1;
      bool     ack_same_as_sent:1;
    } __attribute__((packed));
    struct ublox::msg_hdr_t sent;

    struct rx_msg_t : ublox::msg_t
    {
      uint8_t  crc_a;   // accumulated as packet received
      uint8_t  crc_b;   // accumulated as packet received

      rx_msg_t()
      {
        init();
      }

      void init()
      {
        msg_class = ublox::UBX_UNK;
        msg_id    = ublox::UBX_ID_UNK;
        crc_a = 0;
        crc_b = 0;
      }

    } __attribute__((packed));

    rx_msg_t m_rx_msg;

    void rxBegin();
    bool rxEnd();

    static const uint8_t SYNC_1 = 0xB5;
    static const uint8_t SYNC_2 = 0x62;

    IOStream::Device *m_device;

    static clock_t s_start_of_week;

#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
    bool parseTOW( uint8_t chr )
    {
      if (chrCount == 0) {
        m_fix.valid.date =
        m_fix.valid.time = false;
      }
      ((uint8_t *) &m_fix.dateTime)[ chrCount ] = chr;
      if (chrCount == 3) {
        uint32_t tow = *((uint32_t *) &m_fix.dateTime);
//trace << PSTR("@ ") << tow;
        if (!from_TOWms( tow ))
          m_fix.dateTime = (clock_t) 0L;
//trace << PSTR(".") << m_fix.dateTime_cs;
      }
      return true;
    }
#endif
#endif

#ifdef UBLOX_PARSE_NMEA
public:

    /** ublox proprietary NMEA message types. */
    enum pubx_msg_t {
        PUBX_00 = NMEA_LAST_MSG+1,
        PUBX_04 = PUBX_00+4
    };
    static const nmea_msg_t PUBX_FIRST_MSG = (nmea_msg_t) PUBX_00;
    static const nmea_msg_t PUBX_LAST_MSG  = (nmea_msg_t) PUBX_04;

protected:
    static const char * const ublox_nmea[] __PROGMEM;
    static const uint8_t ublox_nmea_size;
    static const msg_table_t ublox_msg_table __PROGMEM;

    const msg_table_t *msg_table() const { return &ublox_msg_table; };

    decode_t parseCommand( char c );

#endif

protected:
    bool parseField( char chr );
};

#endif
