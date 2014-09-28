#include "ubxGPS.h"

#include "Cosa/RTC.hh"

void ubloxGPS::rxBegin()
{
  rx_msg.init();
  chrCount = 0;
}

void ubloxGPS::rxEnd()
{
    if (rx_msg.msg_class != UBX_UNK) {

#ifdef NEOGPS_STATS
        statistics.parser_ok++;
#endif
        if ((rx_msg.msg_class == UBX_ACK) &&
            (rx_msg.msg_id    == UBX_ACK_ACK) &&
            (rx_msg.length    == 2)) {
          acked = true;
        }
//      else
//        Event::push( Event::RECEIVE_COMPLETED_TYPE, this, 0 );
    }
}


int ubloxGPS::putchar(char c)
{
    uint8_t chr = c;

    switch ((ubxState_t) rxState) {

      case UBX_IDLE:
        if (chr == SYNC_1)
          rxState = (rxState_t) UBX_SYNC2;
        else
          // Delegate
          return NeoGPS::putchar( c );
        break;

      case UBX_SYNC2:
          if (chr == SYNC_2) {
            rxBegin();
            rxState = (rxState_t) UBX_HEAD;
          } else {
            rxState = (rxState_t) UBX_IDLE;
          }
          break;
          
      case UBX_HEAD:
          rx_msg.crc_a += chr;
          rx_msg.crc_b += rx_msg.crc_a;

          switch (chrCount++) {
            case 0:
              rx_msg.msg_class = (msg_class_t) chr;
              break;
            case 1:
              rx_msg.msg_id = (msg_id_t) chr;
              break;
            case 2:
              rx_msg.length = chr << 8;
              break;
            case 3:
              rx_msg.length += chr;
              chrCount = 0;
              rxState = (rxState_t) UBX_RECEIVING_DATA;
              break;
          }
          break;
          
        case UBX_RECEIVING_DATA:
          rx_msg.crc_a += chr;
          rx_msg.crc_b += rx_msg.crc_a;

          // Except for acks, the payload is not saved

          if (rx_msg.msg_class == UBX_ACK) {
            if (chrCount == 0)
              ubx_ack.msg_class = (msg_class_t) chr;
            else if (chrCount == 1)
              ubx_ack.msg_id = (msg_id_t) chr;
          }
              
          if (++chrCount >= rx_msg.length) {
            // payload size received
            rxState = (rxState_t) UBX_CRC_A;
          }
          break;

      case UBX_CRC_A:
          if (chr != rx_msg.crc_a) {
            rx_msg.msg_class = UBX_UNK;
#ifdef NEOGPS_STATS
            statistics.parser_crcerr++;
#endif
          }
          rxState = (rxState_t) UBX_CRC_B;
          break;

      case UBX_CRC_B:
          if (chr != rx_msg.crc_b) {
            rx_msg.msg_class = UBX_UNK;
#ifdef NEOGPS_STATS
            statistics.parser_crcerr++;
#endif
          } else
            rxEnd();
          rxState = (rxState_t) UBX_IDLE;
          break;
    }

    return c;
}

static const uint8_t cfg_msg_data[] __PROGMEM =
  { ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_MSG,
    sizeof(ubloxGPS::cfg_msg_t), 0,
    ubloxGPS::UBX_NMEA, NeoGPS::NMEA_VTG, 0 };
  
static const ubloxGPS::cfg_msg_t *cfg_msg_P =
  (const ubloxGPS::cfg_msg_t *) &cfg_msg_data[0];

bool ubloxGPS::enableNMEA( enum nmea_msg_t msgType, uint8_t rate )
{
  static const ubx_nmea_msg_t ubx[] = {
        UBX_GPGGA,
        UBX_GPGLL,
        UBX_GPGSA,
        UBX_GPGSV,
        UBX_GPRMC,
        UBX_GPVTG,
        UBX_GPZDA,
    };

  if ((unsigned int)msgType >= membersof(ubx))
    return false;


  send_P( *cfg_msg_P );

  cfg_msg_t cfg_msg( msgType, rate );

  return send( cfg_msg );
}


bool ubloxGPS::msg_t::send( IOStream::Device *tx ) const
{
    tx->putchar( SYNC_1 );
    tx->putchar( SYNC_2 );

    uint8_t crc_a = 0;
    uint8_t crc_b = 0;
    send( tx, msg_class, crc_a, crc_b );
    send( tx, msg_id, crc_a, crc_b );
    
    uint16_t l = length;
    send( tx, l, crc_a, crc_b );
    send( tx, l >> 8, crc_a, crc_b );

    uint8_t *payload = (uint8_t *)this;
    payload = &payload[ sizeof(msg_t) ];
    while (l--)
      send( tx, *payload++, crc_a, crc_b );

    tx->putchar( crc_a );
    tx->putchar( crc_b );
    tx->flush();

    return true;
}


bool ubloxGPS::msg_t::send_P( IOStream::Device *tx ) const
{
    tx->putchar( SYNC_1 );
    tx->putchar( SYNC_2 );

    uint32_t dword = pgm_read_dword( this );
    uint8_t crc_a = 0;
    uint8_t crc_b = 0;

    send( tx, (uint8_t) dword, crc_a, crc_b ); // msg_class
    dword >>= 8;
    send( tx, (uint8_t) dword, crc_a, crc_b ); // msg_id
    dword >>= 8;
    
    uint16_t l = dword;
    send( tx, l, crc_a, crc_b );
    send( tx, l >> 8, crc_a, crc_b );

    uint8_t *payload = (uint8_t *)this;
    payload = &payload[ sizeof(msg_t) ];
    while (l > 0) {
      if (l >= sizeof(dword)) {
        uint8_t chunk = sizeof(dword);
        l -= sizeof(dword);
        dword = pgm_read_dword( payload );
        while (chunk--) {
          send( tx, (uint8_t) dword, crc_a, crc_b );
          dword >>= 8;
        }
        payload = &payload[ sizeof(dword) ];

      } else {
        send( tx, pgm_read_byte( payload++ ), crc_a, crc_b );
        l--;
      }
    }

    tx->putchar( crc_a );
    tx->putchar( crc_b );
    tx->flush();

    return true;
}


bool ubloxGPS::wait_for_ack()
{
    acked = false;
    
    uint16_t sent = RTC::millis();

    do {
      if (acked)
        break;
      Watchdog::delay( 16 );
    } while (((uint16_t) RTC::millis()) - sent < 500);
        
    return acked;
}

/**
 * send( msg_t & msg )
 * Sends UBX command and waits for the ack.
 */
bool ubloxGPS::send( msg_t & msg )
{
    // make sure we don't interfere with other data
    for (uint8_t waits=0; waits++ < 3;) {

      if ((rxState != NMEA_IDLE) || 
          (rxState != (enum rxState_t) UBX_IDLE)) {
        // Not sure this is really required.  According
        //    to "u-blox 6 Receiver Description, Including 
        //    Protocol Specification", data sent from the
        //    device is managed at a frame level:
        //    out-of-date or overflow *frames* are
        //    discarded.  Need to look into this...
        Watchdog::delay(16);

      } else {
        for (uint8_t tries = 0; tries++ < 3;) {
          if (msg.send( _tx ) &&
              ((msg.msg_class != UBX_CFG) ||
               (wait_for_ack() &&
                (ubx_ack.msg_class == msg.msg_class) &&
                (ubx_ack.msg_id    == msg.msg_id))))
            return true;
        }
        break;
      }
    }

    return false;
}

bool ubloxGPS::send_P( const msg_t & msg )
{
    // make sure we don't interfere with other data
    for (uint8_t waits=0; waits++ < 3;) {

      if ((rxState != NMEA_IDLE) || 
          (rxState != (enum rxState_t) UBX_IDLE)) {
        Watchdog::delay(16);

      } else {
        for (uint8_t tries = 0; tries++ < 3;) {
          msg.send_P( _tx );
          if ((msg.msg_class != UBX_CFG) ||
              (wait_for_ack() &&
               (ubx_ack.msg_class == msg.msg_class) &&
               (ubx_ack.msg_id    == msg.msg_id)))
            return true;
        }
        break;
      }
    }

    return false;
}

const ubloxGPS::msg_hdr_t test __PROGMEM =
  { ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_RATE };

const uint8_t test2_data[] __PROGMEM =
  { ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_RATE,
    sizeof(ubloxGPS::cfg_rate_t), 0,
    0xE8, 0x03, 0x01, 0x00, ubloxGPS::UBX_TIME_REF_GPS, 0  };

const ubloxGPS::msg_t *test2 = (const ubloxGPS::msg_t *) &test2_data[0];


#if 0
// example configuration for stationary application

    cfg_nav5_t setNav = {
        UBX_CFG, UBX_CFG_NAV5, sizeof(cfg_nav5_t), 0,0, 
        { true, true, true, true, true, true, true, true, 0xFF },
        UBX_DYN_MODEL_STATIONARY,
        UBX_POS_FIX_AUTO,
        0,                  // fixed alt
        10000,              // fixed alt variance
        5,                  // min elev
        0,                  // Max Time to perform Dead Reckoning
        250,                // PDOP mask x0.1
        250,                // HDOP mask x0.1
        100,                // Pos Accuracy mask
        300,                // Time Accuracy mask
        0,                  // Static hold threshold (cm/s)
        0,                  // DGPS timeout (v7 FW or later)
        0, 0, 0 // -- always zero --
      };

    if (! send( setNav ))
        return false;
    
    // 1Hz Data Rate
    cfg_rate_t setDataRate =
      { UBX_CFG, UBX_CFG_RATE, sizeof(cfg_rate_t), 0,0,
        1000, 1, UBX_TIME_REF_GPS };

    if (! send( setDataRate ))
        return false;

    // Disable unused message types (GLL, GSA, GSV, RMC, VTG)
    for (enum nmea_msg_t i=NMEA_GLL; i <= NMEA_VTG; i++) {
        if (! enableNMEA( i, 0 ))
            return false;
    }
    
    // make sure we receive date/time messages
    if (! enableNMEA( NMEA_GPZDA, 2))
        return false;

    // after the time has a fix (good ZDAs received),
    //    we'll enable the location messages (e.g., GPRMC) and
    //    disable GPZDA.
    
    return true;

#endif
