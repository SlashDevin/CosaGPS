#ifndef UBXMSG_H
#define UBXMSG_H

#include "NeoGPS.h"
class ubloxGPS;

namespace ublox {

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
        UBX_ACK_NAK    = 0x00, // Reply to CFG messages
        UBX_ACK_ACK    = 0x01, // Reply to CFG messages
        UBX_CFG_MSG    = 0x01, // Configure which messages to send
        UBX_CFG_RATE   = 0x08, // Configure message rate
        UBX_CFG_NAV5   = 0x24, // Configure navigation engine settings
        UBX_MON_VER    = 0x04, // Monitor Receiver/Software version
        UBX_NAV_POSLLH = 0x02, // Current Position
        UBX_NAV_VELNED = 0x12, // Current Velocity
        UBX_ID_UNK   = 0xFF
      }  __attribute__((packed));

      struct msg_hdr_t {
          msg_class_t msg_class;
          msg_id_t    msg_id;
          bool same_kind( const msg_hdr_t & msg ) const volatile
            { return (msg_class == msg.msg_class) && (msg_id == msg.msg_id); }
      }  __attribute__((packed));

      struct msg_t : msg_hdr_t {
          uint16_t length;  // should be sizeof(this)-sizeof(msg+hdr_t)
#define UBX_CTOR_LEN (sizeof(*this)-sizeof(msg_t))

          msg_t()
          {
            length    = 0;
          };
          msg_t( enum msg_class_t m, enum msg_id_t i, uint16_t l )
          {
              msg_class = m;
              msg_id    = i;
              length    = l;
          }
      } __attribute__((packed));

    /**
      * Configure message intervals.
      */

    enum ubx_nmea_msg_t { // msg_id's for UBX_NMEA msg_class
        UBX_GPGGA = 0x00,
        UBX_GPGLL = 0x01,
        UBX_GPGSA = 0x02,
        UBX_GPGSV = 0x03,
        UBX_GPRMC = 0x04,
        UBX_GPVTG = 0x05,
        UBX_GPZDA = 0x08
    } __attribute__((packed));

    struct cfg_msg_t : msg_t {
        msg_class_t  cfg_msg_class;
        msg_id_t     cfg_msg;
        uint8_t      rate;

        cfg_msg_t( msg_class_t m, msg_id_t i, uint8_t r )
          : msg_t( UBX_CFG, UBX_CFG_MSG, UBX_CTOR_LEN )
        {
          cfg_msg_class = m;
          cfg_msg       = i;
          rate          = r;
        };
    } __attribute__((packed));

    extern bool configNMEA( ubloxGPS &gps, NeoGPS::nmea_msg_t msgType, uint8_t rate );
    

    // Configure navigation rate
    enum time_ref_t {
      UBX_TIME_REF_UTC=0,
      UBX_TIME_REF_GPS=1
    } __attribute__((packed));

    struct cfg_rate_t : msg_t {
        uint16_t        GPS_meas_rate;
        uint16_t        nav_rate;
        enum time_ref_t time_ref:16;

        cfg_rate_t( uint16_t gr, uint16_t nr, enum time_ref_t tr )
          : msg_t( UBX_CFG, UBX_CFG_RATE, UBX_CTOR_LEN )
        {
          GPS_meas_rate = gr;
          nav_rate      = nr;
          time_ref      = tr;
        }
    }  __attribute__((packed));

    //  Navigation Engine Expert Settings
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
        } __attribute__((packed));
                
        enum dyn_model_t       dyn_model:8;
        enum position_fix_t    fix_mode:8;
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

        cfg_nav5_t() : msg_t( UBX_CFG, UBX_CFG_NAV5, UBX_CTOR_LEN )
          {
            apply_word = 0xFF00;
            always_zero_1 =
            always_zero_2 =
            always_zero_3 = 0;
          }

    }  __attribute__((packed));

    // Geodetic Position Solution
    struct nav_posllh_t : msg_t {
        uint32_t time_of_week; // mS
        int32_t  lon; // deg * 1e7
        int32_t  lat; // deg * 1e7
        int32_t  height_above_ellipsoid; // mm
        int32_t  height_MSL; // mm
        uint32_t horiz_acc; // mm
        uint32_t vert_acc; // mm

        nav_posllh_t() : msg_t( UBX_NAV, UBX_NAV_POSLLH, UBX_CTOR_LEN ) {};
    }  __attribute__((packed));

    // Velocity Solution in North/East/Down
    struct nav_velned_t : msg_t {
        uint32_t time_of_week; // mS
        int32_t  vel_north;    // cm/s
        int32_t  vel_east;     // cm/s
        int32_t  vel_down;     // cm/s
        uint32_t speed_3D;     // cm/s
        uint32_t speed_2D;     // cm/s
        int32_t  heading;      // degrees * 1e5
        uint32_t speed_acc;    // cm/s
        uint32_t heading_acc;  // degrees * 1e5
        
        nav_velned_t() : msg_t( UBX_NAV, UBX_NAV_VELNED, UBX_CTOR_LEN ) {};
    }  __attribute__((packed));
    
};

#endif