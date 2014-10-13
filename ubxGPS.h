#ifndef _UBXGPS_H_
#define _UBXGPS_H_

#include "NeoGPS.h"
#include "ubxmsg.h"

#include <avr/pgmspace.h>

class ubloxGPS : public NeoGPS
{
public:

    ubloxGPS( IOStream::Device *device )
      : NeoGPS( device ),
        storage( (ublox::msg_t *) NULL ),
        reply( (ublox::msg_t *) NULL ),
        reply_expected( false ),
        ack_expected( false )
      {};

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

private:
    ubloxGPS(); // NO!
    ubloxGPS( const ubloxGPS & ); // NO!

    inline void write
      ( uint8_t c, uint8_t & crc_a, uint8_t & crc_b ) const
    {
        m_device->putchar( c );
        crc_a += c;
        crc_b += crc_a;
    };
    void write( const ublox::msg_t & msg );
    void write_P( const ublox::msg_t & msg );

    void wait_for_idle() const;
    bool wait_for_ack();
    bool waiting() const
    {
      return (ack_expected && (!ack_received && !nak_received)) ||
             (reply_expected && !reply_received);
    }


    // Derived class should override this if the contents of a
    //   particular message need to be saved.
    // This executes in an interrupt context, so be quick!
    //  NOTE: the ublox::msg_t.length will get stepped on, so you may need to
    //  set it every time if you are using a union for your storage.
    virtual ublox::msg_t *storage_for( volatile const ublox::msg_t & rx_msg )
      { return (ublox::msg_t *)NULL; };

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

      void init() volatile
      {
        msg_class = ublox::UBX_UNK;
        msg_id    = ublox::UBX_ID_UNK;
        crc_a = 0;
        crc_b = 0;
      }

    } __attribute__((packed));
    volatile rx_msg_t m_rx_msg;
    volatile rx_msg_t & rx() { return m_rx_msg; }

    void rxBegin();
    void rxEnd();

    static const uint8_t SYNC_1 = 0xB5;
    static const uint8_t SYNC_2 = 0x62;

protected:
    enum ubxState_t {
        UBX_IDLE  = NMEA_IDLE,
        UBX_SYNC2 = NMEA_LAST_STATE+1,
        UBX_HEAD,
        UBX_RECEIVING_DATA,
        UBX_CRC_A,
        UBX_CRC_B
    }  __attribute__((packed));

    int putchar( char c );
};

#endif
