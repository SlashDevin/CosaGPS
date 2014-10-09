#ifndef UBXMSG_H
#define UBXMSG_H

#include "ubxGPS.h"

namespace ublox {

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

    struct cfg_msg_t : ubloxGPS::msg_t {
        ubloxGPS::msg_class_t  cfg_msg_class;
        ubloxGPS::msg_id_t     cfg_msg;
        uint8_t      rate;

        cfg_msg_t( ubloxGPS::msg_class_t m, ubloxGPS::msg_id_t i, uint8_t r )
          : msg_t( ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_MSG, UBX_CTOR_LEN )
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

    struct cfg_rate_t : ubloxGPS::msg_t {
        uint16_t        GPS_meas_rate;
        uint16_t        nav_rate;
        enum time_ref_t time_ref:16;

        cfg_rate_t( uint16_t gr, uint16_t nr, enum time_ref_t tr )
          : msg_t( ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_RATE, UBX_CTOR_LEN )
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

    struct cfg_nav5_t : ubloxGPS::msg_t {
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

        cfg_nav5_t() : msg_t( ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_NAV5, UBX_CTOR_LEN )
          {
            apply_word = 0xFF00;
            always_zero_1 =
            always_zero_2 =
            always_zero_3 = 0;
          }

    }  __attribute__((packed));
};

#endif