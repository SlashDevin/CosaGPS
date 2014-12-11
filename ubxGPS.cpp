#include "ubxGPS.h"

#include "Cosa/RTC.hh"
#include "Cosa/Watchdog.hh"
#include "Cosa/Trace.hh"

#ifdef UBLOX_PARSE_UBLOX
using namespace ublox;

uint8_t ubloxGPS::leap_seconds = 0;
clock_t ubloxGPS::start_of_week = 0;

void ubloxGPS::rxBegin()
{
  m_rx_msg.init();
  storage = (msg_t *) NULL;
  chrCount = 0;
#ifdef UBLOX_PARSE_NMEA
  nmeaMessage = NMEA_UNKNOWN;
#endif
}

bool ubloxGPS::rxEnd()
{
  bool visible_msg = false;

  if (rx().msg_class == UBX_ACK) {

    if (ack_expected && ack_same_as_sent) {
      if (rx().msg_id == UBX_ACK_ACK)
        ack_received = true;
      else if (rx().msg_id == UBX_ACK_NAK)
        nak_received = true;
      ack_expected = false;
    }

  } else if (rx().msg_class != UBX_UNK) {

#ifdef NMEAGPS_STATS
        statistics.parser_ok++;
#endif
    visible_msg = true;
    if (storage) {
      if (reply_expected && (storage == reply)) {
        reply_expected = false;
        reply_received = true;
        reply = (msg_t *) NULL;
        visible_msg = false;
      } else {
        storage->msg_class = rx().msg_class;
        storage->msg_id    = rx().msg_id;
        if (storage->length > rx().length)
          storage->length    = rx().length;
      }
      storage = (msg_t *) NULL;
    }
  }
  return visible_msg;
}

ubloxGPS::decode_t ubloxGPS::decode( char c )
{
    decode_t res = DECODE_CHR_OK;
    uint8_t chr = c;

    switch ((ubxState_t) rxState) {

      case UBX_IDLE:
        if (chr == SYNC_1)
          rxState = (rxState_t) UBX_SYNC2;
#ifdef UBLOX_PARSE_NMEA
        else {
          // Delegate
          res = NMEAGPS::decode( c );
          if (rxState != NMEA_IDLE)
            m_rx_msg.init();
        }
#endif
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
//trace << hex << chr;
          m_rx_msg.crc_a += chr;
          m_rx_msg.crc_b += m_rx_msg.crc_a;

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
//trace << hex << chr;
          m_rx_msg.crc_a += chr;
          m_rx_msg.crc_b += m_rx_msg.crc_a;

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
          if (chr != m_rx_msg.crc_a) {
            rx().msg_class = UBX_UNK;
#ifdef NMEAGPS_STATS
            statistics.parser_crcerr++;
#endif
          }
          rxState = (rxState_t) UBX_CRC_B;
          break;

      case UBX_CRC_B:
          if (chr != m_rx_msg.crc_b) {
            rx().msg_class = UBX_UNK;
#ifdef NMEAGPS_STATS
            statistics.parser_crcerr++;
#endif
          } else if (rxEnd())
            res = ubloxGPS::DECODE_COMPLETED;
          rxState = (rxState_t) UBX_IDLE;
          break;

      default:
#ifdef UBLOX_PARSE_NMEA
          // Delegate
          res = NMEAGPS::decode( c );
#endif
          break;
    }

    return res;
}


void ubloxGPS::wait_for_idle()
{
  // Wait for the input buffer to be emptied
  for (uint8_t waits=0; waits < 8; waits++) {
    run();
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
//trace << PSTR("::write ") << msg.msg_class << PSTR("/") << msg.msg_id << endl;
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
//trace << PSTR("::send - ") << hex << msg.msg_class << PSTR(" ") << hex << msg.msg_id << PSTR(" ");
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

#endif

#ifdef UBLOX_PARSE_NMEA
static const char pubx[] __PROGMEM =  "PUBX";
const char * const ubloxGPS::ublox_nmea[] __PROGMEM = { pubx };
const uint8_t ubloxGPS::ublox_nmea_size = membersof(ublox_nmea);

const NMEAGPS::msg_table_t ubloxGPS::ublox_msg_table __PROGMEM =
  {
    ubloxGPS::PUBX_FIRST_MSG,
    &NMEAGPS::nmea_msg_table,
    ubloxGPS::ublox_nmea_size,
    ubloxGPS::ublox_nmea
  };

//---------------------------------------------

bool ubloxGPS::parseField(char chr)
{
    bool ok = true;

    switch (nmeaMessage) {

      case PUBX_00:
        switch (fieldIndex) {
            case 1:
//trace << chr;
              // The first field is actually a message subtype
              if (chrCount == 0)
                ok = (chr == '0');
              else if (chrCount == 1)
                nmeaMessage = (nmea_msg_t) (nmeaMessage + chr - '0');
              break;
#ifdef NMEAGPS_PARSE_PUBX_00
            CASE_TIME(2);
            CASE_LOC(3);
            CASE_ALT(7);
            case 8:
              switch (chrCount) {
                case 0:
                  if (chr == 'N')
                    m_fix.status = gps_fix::STATUS_NONE;
                  else if (chr == 'T')
                    m_fix.status = gps_fix::STATUS_TIME_ONLY;
                  else if (chr == 'R')
                    m_fix.status = gps_fix::STATUS_EST;
                  else if (chr == 'G')
                    m_fix.status = gps_fix::STATUS_STD;
                  else if (chr == 'D')
                    m_fix.status = gps_fix::STATUS_DGPS;
                  else ok = false;
                  break;
                case 1:
                  if (((chr == 'T') && (m_fix.status == gps_fix::STATUS_TIME_ONLY)) ||
                      ((chr == 'K') && (m_fix.status == gps_fix::STATUS_EST)) ||
                      (((chr == '2') || (chr == '3')) &&
                       ((m_fix.status == gps_fix::STATUS_STD) || (m_fix.status == gps_fix::STATUS_DGPS))) ||
                      ((chr == 'F') && (m_fix.status == gps_fix::STATUS_NONE)))
                    ;
                  else if ((chr == 'R') && (m_fix.status == gps_fix::STATUS_DGPS))
                    m_fix.status = gps_fix::STATUS_EST;
                  else
                    ok = false;
                  break;
              }
              break;
            CASE_SPEED(11); // kph!
            CASE_HEADING(12);
            CASE_HDOP(15);
            CASE_SAT(18);
#endif
        }
        break;

      case PUBX_04:
#ifdef NMEAGPS_PARSE_PUBX_04
        switch (fieldIndex) {
            CASE_TIME(2);
            CASE_DATE(3);
#if defined(GPS_FIX_DATE) | defined(GPS_FIX_TIME)
            case 4:
              if (chrCount == 0) {
                bool someDT = false;
#if defined(GPS_FIX_DATE)
                someDT |= m_fix.valid.date;
#endif
#if defined(GPS_FIX_TIME)
                someDT |= m_fix.valid.time;
#endif
                if (someDT && (m_fix.status == gps_fix::STATUS_NONE))
                  m_fix.status = gps_fix::STATUS_TIME_ONLY;
              }
              break;
#endif
        }
#endif
        break;

      default: return NMEAGPS::parseField(chr);
    }
    return ok;
}
#endif

#if 0
static const uint8_t cfg_msg_data[] __PROGMEM =
  { ubloxGPS::UBX_CFG, ubloxGPS::UBX_CFG_MSG,
    sizeof(ubloxGPS::cfg_msg_t), 0,
    ubloxGPS::UBX_NMEA, NMEAGPS::NMEA_VTG, 0 };

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
