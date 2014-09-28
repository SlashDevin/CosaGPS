#ifndef _UBXGPS_H_
#define _UBXGPS_H_

#include "NeoGPS.h"

class ubloxGPS : public NeoGPS
{
public:
    static const uint8_t SYNC_1 = 0xB5;
    static const uint8_t SYNC_2 = 0x62;

    enum ubxState_t {
        UBX_IDLE = NMEA_LAST_STATE+1,
        UBX_SYNC2,
        UBX_HEAD,
        UBX_RECEIVING_DATA,
        UBX_CRC_A,
        UBX_CRC_B
    }  __attribute__((packed));

    enum msg_class_t
      { UBX_NAV  = 0x01,  // Navigation results
        UBX_RXM  = 0x02,  // Receiver Manager messages
        UBX_INF  = 0x04,  // Informational messages
        UBX_ACK  = 0x05,  // ACK/NAK replies to CFG messages
        UBX_CFG  = 0x06,  // Configuration input messages
        UBX_MON  = 0x0A,  // Monitoring messages
        UBX_AID  = 0x0B,  // Assist Now aiding messages
        UBX_TIM  = 0x0D,  // Timing messages
        UBX_NMEA = 0xF0,  // NMEA Standard messages
        UBX_UNK  = 0xFF
      }  __attribute__((packed));

    enum msg_id_t
      {
        UBX_ACK_NAK  = 0x00, // Reply to CFG messages
        UBX_ACK_ACK  = 0x01, // Reply to CFG messages
        UBX_CFG_MSG  = 0x01, // Configure which messages to send
        UBX_CFG_RATE = 0x08, // Configure message rate
        UBX_CFG_NAV5 = 0x24, // Configure navigation engine settings
        UBX_ID_UNK   = 0xFF
      }  __attribute__((packed));

    struct msg_hdr_t {
        enum msg_class_t msg_class:8;
        enum msg_id_t    msg_id:8;
    }  __attribute__((packed));

    struct msg_t : msg_hdr_t {
        uint16_t length;  // should be sizeof(this)-sizeof(msg+hdr_t)
#define UBX_LEN (sizeof(*this)-sizeof(msg_t))

        msg_t() { length = 0; };
        msg_t( enum msg_class_t m, enum msg_id_t i, uint16_t l )
        {
            msg_class = m;
            msg_id    = i;
            length    = l;
        }

        bool send( IOStream::Device *tx ) const;
        bool send_P( IOStream::Device *tx ) const;

        inline void send( IOStream::Device *tx, uint8_t c, uint8_t crc_a, uint8_t crc_b ) const
        {
            tx->putchar( c );
            crc_a += c;
            crc_b += crc_a;
        }
    } __attribute__((packed));

    enum time_ref_t {
      UBX_TIME_REF_UTC=0,
      UBX_TIME_REF_GPS=1
    } __attribute__((packed));

    struct cfg_rate_t : msg_t {
        uint16_t        GPS_meas_rate;
        uint16_t        nav_rate;
        enum time_ref_t time_ref:16;

        cfg_rate_t( uint16_t gr, uint16_t nr, enum time_ref_t tr )
          : msg_t( UBX_CFG, UBX_CFG_RATE, UBX_LEN )
        {
          GPS_meas_rate = gr;
          nav_rate      = nr;
          time_ref      = tr;
        }
    }  __attribute__((packed));

    enum ubx_nmea_msg_t {
        UBX_GPGGA = 0x00,
        UBX_GPGLL = 0x01,
        UBX_GPGSA = 0x02,
        UBX_GPGSV = 0x03,
        UBX_GPRMC = 0x04,
        UBX_GPVTG = 0x05,
        UBX_GPZDA = 0x08
    } __attribute__((packed));

    struct cfg_msg_t : msg_t {
        msg_class_t  cfg_msg_class; // usually UBX_NMEA
        uint8_t      cfg_msg;
        uint8_t      rate;

        cfg_msg_t( enum nmea_msg_t m, uint8_t r )
          : msg_t( UBX_CFG, UBX_CFG_MSG, UBX_LEN )
        {
          cfg_msg_class = UBX_NMEA;
          cfg_msg       = m;
          rate          = r;
        };
    } __attribute__((packed));

    enum dyn_model_t {
        UBX_DYN_MODEL_PORTABLE   = 0,
        UBX_DYN_MODEL_STATIONARY = 2,
        UBX_DYN_MODEL_PEDESTRIAN = 3,
        UBX_DYN_MODEL_AUTOMOTIVE = 4,
        UBX_DYN_MODEL_SEA        = 5,
        UBX_DYN_MODEL_AIR_1G     = 6,
        UBX_DYN_MODEL_AIR_2G     = 7,
        UBX_DYN_MODEL_AIR_4G     = 8
    } __attribute__((packed));

    enum position_fix_t {
        UBX_POS_FIX_2D_ONLY = 1,
        UBX_POS_FIX_3D_ONLY = 2,
        UBX_POS_FIX_AUTO    = 3
    } __attribute__((packed));

    struct cfg_nav5_t : msg_t {
        struct parameter_mask_t {
            bool dyn_model            :1;
            bool min_elev             :1;
            bool fix                  :1;
            bool dr_limit             :1;
            bool pos_mask             :1;
            bool time_mask            :1;
            bool static_hold_thr      :1;
            bool dgps_timeout         :1;
            int  _unused_             :8;
        } __attribute__((packed));

        union {
          struct parameter_mask_t apply;
          uint16_t                apply_word;
        };
                
        enum dyn_model_t       dyn_model;
        enum position_fix_t    fix_mode;
        int32_t                fixed_alt;          // m MSL x0.01
        uint32_t               fixed_alt_variance; // m^2 x0.0001
        int8_t                 min_elev;           // deg
        uint8_t                dr_limit;           // s
        uint16_t               pos_dop_mask;       // x0.1
        uint16_t               time_dop_mask;      // x0.1
        uint16_t               pos_acc_mask;       // m
        uint16_t               time_acc_mask;      // m
        uint8_t                static_hold_thr;    // cm/s
        uint8_t                dgps_timeout;       // s
        uint32_t always_zero_1;
        uint32_t always_zero_2;
        uint32_t always_zero_3;

        cfg_nav5_t() : msg_t( UBX_CFG, UBX_CFG_NAV5, UBX_LEN )
          { apply_word = 0xFF00; }

    }  __attribute__((packed));

     /**
      * Configures NMEA message intervals.
      * Uses UBX configuration messages and only works on Ublox modules.
      */
    bool enableNMEA( enum nmea_msg_t msgType, uint8_t rate );

    /**
     * Sends a UBX message and waits for ACK if it is a configuration message.
     */
    bool send( msg_t & msg );
    bool send_P( const msg_t & msg );

    /**
     * Identifies the device used to send UBX messages.
     * @param[in] tx
     */
    void sendingDevice(IOStream::Device *tx) {
        _tx = tx;
    }

    ubloxGPS()
      : _tx( (IOStream::Device *)NULL ),
        acked( false )
      {};

private:
    IOStream::Device *_tx;

    bool wait_for_ack();
    volatile bool acked;
    struct msg_hdr_t ubx_ack;

    struct rx_msg_t : msg_t
    {
        uint8_t  crc_a;   // accumulated as packet received
        uint8_t  crc_b;   // accumulated as packet received
        void init()
        {
          crc_a = 0;
          crc_b = 0;
        }
    };
    struct rx_msg_t rx_msg;
    void rxBegin();
    void rxEnd();
    int putchar( char c );
};

#endif
