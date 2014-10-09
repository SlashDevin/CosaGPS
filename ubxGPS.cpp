#include "ubxGPS.h"

#include "Cosa/RTC.hh"

void ubloxGPS::rxBegin()
{
  rx().init();
  storage = (msg_t *) NULL;
  chrCount = 0;
}

void ubloxGPS::rxEnd()
{
  if (rx().msg_class == UBX_ACK) {

    if (ack_expected && ack_same_as_sent) {
      if (rx().msg_id == UBX_ACK_ACK)
        ack_received = true;
      else if (rx().msg_id == UBX_ACK_NAK)
        nak_received = true;
      ack_expected = false;
    }

  } else if (rx().msg_class != UBX_UNK) {

#ifdef NEOGPS_STATS
        statistics.parser_ok++;
#endif
    bool event = true;
    if (storage) {
      if (reply_expected && (storage == reply)) {
        reply_expected = false;
        reply_received = true;
        reply = (msg_t *) NULL;
        event = false;
      } else {
        storage->msg_class = rx().msg_class;
        storage->msg_id    = rx().msg_id;
        storage->length    = rx().length;
      }
      storage = (msg_t *) NULL;
    }
    if (event) {
      uint16_t val = ((uint16_t)rx().msg_class) +
                     (((uint16_t)rx().msg_id)<<8);
      Event::push( Event::RECEIVE_COMPLETED_TYPE, this, val );
    }
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
          rx().crc_a += chr;
          rx().crc_b += rx().crc_a;

          switch (chrCount++) {
            case 0:
              rx().msg_class = (msg_class_t) chr;
              break;
            case 1:
              rx().msg_id = (msg_id_t) chr;
              break;
            case 2:
              rx().length = chr;
              break;
            case 3:
              rx().length += chr << 8;
              chrCount = 0;
              rxState = (rxState_t) UBX_RECEIVING_DATA;
              if (rx().msg_class == UBX_ACK) {
                if (ack_expected)
                  ack_same_as_sent = true; // so far...
              } else if (reply_expected && rx().same_kind( *reply ))
                storage = reply;
              else
                storage = storage_for( rx() );
              break;
          }
          break;

        case UBX_RECEIVING_DATA:
          rx().crc_a += chr;
          rx().crc_b += rx().crc_a;

          if (storage && (chrCount < storage->length))
            ((uint8_t *)storage)[ sizeof(msg_t)+chrCount ] = chr;

          if (ack_same_as_sent) {
            if (((chrCount == 0) && (sent.msg_class != (msg_class_t)chr)) ||
                ((chrCount == 1) && (sent.msg_id    != (msg_id_t)chr)))
              ack_same_as_sent = false;
          }

          if (++chrCount >= rx().length) {
            // payload size received
            rxState = (rxState_t) UBX_CRC_A;
          }
          break;

      case UBX_CRC_A:
          if (chr != rx().crc_a) {
            rx().msg_class = UBX_UNK;
#ifdef NEOGPS_STATS
            statistics.parser_crcerr++;
#endif
          }
          rxState = (rxState_t) UBX_CRC_B;
          break;

      case UBX_CRC_B:
          if (chr != rx().crc_b) {
            rx().msg_class = UBX_UNK;
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



void ubloxGPS::wait_for_idle() const
{
  // Wait for the input buffer to be emptied
  for (uint8_t waits=0; waits < 8; waits++) {
    if (!receiving() || !waiting())
      break;
    Watchdog::delay(16);
  }
}


bool ubloxGPS::wait_for_ack()
{
  m_device->flush();

  uint16_t sent = 0;
  uint16_t idle_time = 0;

  do {
    if (receiving()) {
      wait_for_idle();
      sent = RTC::millis();
    } else if (!waiting()) {
      return true;
    } else {
      // Idle, accumulate time
      uint16_t now = RTC::millis();
      if (sent != 0)
        idle_time += now-sent;
      sent = now;
      Watchdog::delay( 16 );
    }
  } while (idle_time < 100);

  return false;
}

void ubloxGPS::write( const msg_t & msg )
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

  sent.msg_class = msg.msg_class;
  sent.msg_id    = msg.msg_id;
}

void ubloxGPS::write_P( const msg_t & msg )
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

  sent.msg_class = msg.msg_class;
  sent.msg_id    = msg.msg_id;
}

/**
 * send( msg_t & msg )
 * Sends UBX command and optionally waits for the ack.
 */

//#include "Cosa/trace.hh"

bool ubloxGPS::send( const msg_t & msg, msg_t *reply_msg )
{
//trace << PSTR("::send - ");
  bool ok = true;

  write( msg );

  if (msg.msg_class == UBX_CFG) {
    ack_received = false;
    nak_received = false;
    ack_same_as_sent = false;
    ack_expected = true;
  }

  if (reply_msg) {
    reply = reply_msg;
    reply_received = false;
    reply_expected = true;
  }

  if (waiting()) {
    ok = wait_for_ack();
    if (ok) {
//if (ack_received) trace << PSTR("got an ACK\n");
//else if (nak_received) trace << PSTR("got a NAK!\n");
//else trace << PSTR("ok!\n");
    }
//else trace << PSTR("wait_for_ack failed!\n");
  }

  return ok;
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
#endif
