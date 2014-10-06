#include "ubxGPS.h"

#include "Cosa/RTC.hh"

void ubloxGPS::rxBegin()
{
  m_rx_msg.init();
  chrCount = 0;
}

void ubloxGPS::rxEnd()
{
    if (m_rx_msg.msg_class != UBX_UNK) {

#ifdef NEOGPS_STATS
        statistics.parser_ok++;
#endif
      if (m_rx_msg.save_in_reply) {
        m_rx_msg.save_in_reply  = false;
        m_rx_msg.reply_received = true;
        m_rx_msg.reply          = (msg_t *) NULL;
      }
      Event::push( Event::RECEIVE_COMPLETED_TYPE, this, UBX_MSG );
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
          c = NeoGPS::putchar( c );
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
          m_rx_msg.crc_a += chr;
          m_rx_msg.crc_b += m_rx_msg.crc_a;

          switch (chrCount++) {
            case 0:
              m_rx_msg.msg_class = (msg_class_t) chr;
              break;
            case 1:
              m_rx_msg.msg_id = (msg_id_t) chr;
              break;
            case 2:
              m_rx_msg.length = chr;
              break;
            case 3:
              m_rx_msg.length += chr << 8;
              chrCount = 0;
              rxState = (rxState_t) UBX_RECEIVING_DATA;
              m_rx_msg.start_save();
              break;
          }
          break;
          
        case UBX_RECEIVING_DATA:
          m_rx_msg.crc_a += chr;
          m_rx_msg.crc_b += m_rx_msg.crc_a;

          if (m_rx_msg.save_in_reply &&
              (chrCount < m_rx_msg.reply->length))
            ((uint8_t *)m_rx_msg.reply)[ chrCount+sizeof(msg_hdr_t) ] = chr;

          if (++chrCount >= m_rx_msg.length) {
            // payload size received
            rxState = (rxState_t) UBX_CRC_A;
          }
          break;

      case UBX_CRC_A:
          if (chr != m_rx_msg.crc_a) {
            m_rx_msg.msg_class = UBX_UNK;
#ifdef NEOGPS_STATS
            statistics.parser_crcerr++;
#endif
          }
          rxState = (rxState_t) UBX_CRC_B;
          break;

      case UBX_CRC_B:
          if (chr != m_rx_msg.crc_b) {
            m_rx_msg.msg_class = UBX_UNK;
#ifdef NEOGPS_STATS
            statistics.parser_crcerr++;
#endif
          } else {
            rxEnd();
          }
          rxState = (rxState_t) UBX_IDLE;
          break;

      default:
          // Delegate
          c = NeoGPS::putchar( c );
          break;
    }

    return c;
}

bool ubloxGPS::enableNMEA( enum nmea_msg_t msgType, uint8_t rate )
{
  static const ubx_nmea_msg_t ubx[] __PROGMEM = {
        UBX_GPGGA,
        UBX_GPGLL,
        UBX_GPGSA,
        UBX_GPGSV,
        UBX_GPRMC,
        UBX_GPVTG,
        UBX_GPZDA,
    };

  uint8_t msg_index = (uint8_t) msgType - (uint8_t) NMEA_FIRST_MSG;

  if (msg_index >= membersof(ubx))
    return false;

  nmea_msg_t msg = (nmea_msg_t) pgm_read_byte( &ubx[msg_index] );
  cfg_msg_t cfg_msg( msg, rate );

  return send( cfg_msg );
}


void ubloxGPS::wait_for_idle() const
{
  m_device->flush();
  for (uint8_t waits=0; (!m_rx_msg.reply_received) && waits++ < 8;) {
    if (receiving()) {
      Watchdog::delay(16);
    }
  }
}


bool ubloxGPS::wait_for_reply()
{
    uint16_t sent = RTC::millis();

    do {
      if (m_rx_msg.reply_received) {
        return true;
      }
      Watchdog::delay( 16 );
    } while (((uint16_t) RTC::millis()) - sent < 100);
        
    return false;
}

void ubloxGPS::write( const msg_t & msg ) const
{
  m_device->putchar( SYNC_1 );
  m_device->putchar( SYNC_2 );

  uint8_t  crc_a = 0;
  uint8_t  crc_b = 0;
  uint8_t *ptr   = (uint8_t *) &msg;
  uint16_t l     = msg.length + sizeof(msg_t);
  while (l--)
    write( *ptr++, crc_a, crc_b );

  m_device->putchar( crc_a );
  m_device->putchar( crc_b );
}

void ubloxGPS::write_P( const msg_t & msg ) const
{
  m_device->putchar( SYNC_1 );
  m_device->putchar( SYNC_2 );

  uint8_t  crc_a = 0;
  uint8_t  crc_b = 0;
  uint8_t *ptr   = (uint8_t *) &msg;
  uint16_t l     = msg.length + sizeof(msg_t);
  uint32_t dword;

  while (l > 0) {
    if (l >= sizeof(dword)) {
      l -= sizeof(dword);
      dword = pgm_read_dword( ptr );
      for (uint8_t i=sizeof(dword); i--;) {
        write( (uint8_t) dword, crc_a, crc_b );
        dword >>= 8;
      }
      ptr += sizeof(dword);

    } else {
      write( pgm_read_byte( ptr++ ), crc_a, crc_b );
      l--;
    }
  }

  m_device->putchar( crc_a );
  m_device->putchar( crc_b );
}

/**
 * send( msg_t & msg )
 * Sends UBX command and optionally waits for the ack.
 */

bool ubloxGPS::send( const msg_t & msg, msg_t *reply_msg )
{
  cfg_ack_t local_reply;

  if (reply_msg)
    reply( reply_msg );
  else {
    if (msg.msg_class != UBX_CFG) {
      local_reply.msg_class = msg.msg_class;
      local_reply.msg_id    = msg.msg_id;
      local_reply.length    = 0;
    }
    reply( &local_reply );
  }

  write( msg );

  // When the input buffer is emptied, we'll check 
  // to see if the reply was received.
  wait_for_idle();

  if (wait_for_reply())
    return (msg.msg_class != UBX_CFG) || (local_reply.msg_id == UBX_ACK_ACK);

  reply( (msg_t *) NULL );

  return false;
}


bool ubloxGPS::send_P( const msg_t & msg, msg_t *reply_msg )
{
    return false;
}


#if 0
static const uint8_t cfg_msg_data[] __PROGMEM =
  { ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_MSG,
    sizeof(ubloxGPS::cfg_msg_t), 0,
    ubloxGPS::UBX_NMEA, NeoGPS::NMEA_VTG, 0 };
  
static const ubloxGPS::cfg_msg_t *cfg_msg_P =
  (const ubloxGPS::cfg_msg_t *) &cfg_msg_data[0];

    send_P( *cfg_msg_P );

const ubloxGPS::msg_hdr_t test __PROGMEM =
  { ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_RATE };

const uint8_t test2_data[] __PROGMEM =
  { ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_RATE,
    sizeof(ubloxGPS::cfg_rate_t), 0,
    0xE8, 0x03, 0x01, 0x00, ubloxGPS::UBX_TIME_REF_GPS, 0  };

const ubloxGPS::msg_t *test2 = (const ubloxGPS::msg_t *) &test2_data[0];


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
