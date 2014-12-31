#include "ubxGPS.h"

#include "Cosa/RTC.hh"
#include "Cosa/Trace.hh"

using namespace ublox;

//----------------------------------

void ubloxGPS::rxBegin()
{
  m_rx_msg.init();
  storage = (msg_t *) NULL;
  chrCount = 0;
}

bool ubloxGPS::rxEnd()
{
  coherent = true;

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
//if (!visible_msg) trace << PSTR("XXX");

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
static char toHexDigit( uint8_t val )
{
  val &= 0x0F;
  return (val >= 10) ? ((val - 10) + 'A') : (val + '0');
}


ubloxGPS::decode_t ubloxGPS::decode( char c )
{
    decode_t res = DECODE_CHR_OK;
    uint8_t chr = c;

//trace << '-' << rxState;
    switch ((ubxState_t) rxState) {

      case UBX_IDLE:
//if ((c != '\r') && (c != '\n')) trace << toHexDigit(c >> 4) << toHexDigit(c);
        if (chr == SYNC_1)
          rxState = (rxState_t) UBX_SYNC2;
        else
          res = DECODE_CHR_INVALID;
        break;


      case UBX_SYNC2:
//if ((c != '\r') && (c != '\n')) trace << '+' << toHexDigit(c >> 4) << toHexDigit(c);
        if (chr == SYNC_2) {
          rxBegin();
          rxState = (rxState_t) UBX_HEAD;
        } else {
          rxState = (rxState_t) UBX_IDLE;
        }
        break;

      case UBX_HEAD:
//if ((c != '\r') && (c != '\n')) trace << '&' << toHexDigit(c >> 4) << toHexDigit(c);
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
              
              m_fix.valid.init();
              coherent = false;
              
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

parseField( chr );

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
          } else if (rxEnd()) {
//trace << '!';
            res = ubloxGPS::DECODE_COMPLETED;
          }
          rxState = (rxState_t) UBX_IDLE;
          break;

      default:
          res = DECODE_CHR_INVALID;
          break;
    }

    if (res == DECODE_CHR_INVALID) {
//if ((c != '\r') && (c != '\n')) trace << 'x' << toHexDigit(c >> 4) << toHexDigit(c);
      if (rx().msg_class != UBX_UNK)
        m_rx_msg.init();

      // Delegate
      res = NMEAGPS::decode( c );
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
      run();
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
/*
    if (ok) {
      if (ack_received) {
        trace << PSTR("ACK!\n");
      } else if (nak_received) {
        trace << PSTR("NAK!\n");
      } else {
        trace << PSTR("ok!\n");
      }
    } else
      trace << PSTR("wait_for_ack failed!\n");
*/
  }

  return ok;
}


bool ubloxGPS::send_P( const msg_t & msg, msg_t *reply_msg )
{
    return false;
}

//---------------------------------------------

bool ubloxGPS::parseField( char c )
{
    uint8_t chr = c;
    bool ok = true;

    switch (rx().msg_class) {

      case UBX_NAV:
//if (chrCount == 0) trace << PSTR( " NAV ");
        switch (rx().msg_id) {

          case UBX_NAV_STATUS:
//if (chrCount == 0) trace << PSTR( "stat ");
#ifdef UBLOX_PARSE_STATUS
            switch (chrCount) {
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
              case 0: case 1: case 2: case 3:
                ok = parseTOW( chr );
                break;
#endif
              case 4:
                ok = parseFix( chr );
                break;
              case 5:
                {
                  ublox::nav_status_t::flags_t flags =
                    *((ublox::nav_status_t::flags_t *) &chr);
                  m_fix.status =
                    ublox::nav_status_t::to_status
                      ( (ublox::nav_status_t::status_t) m_fix.status, flags );
//trace << m_fix.status << ' ';
                }
                break;
            }
#endif
            break;

          case UBX_NAV_POSLLH:
//if (chrCount == 0) trace << PSTR( "velned ");
#ifdef UBLOX_PARSE_POSLLH
            switch (chrCount) {

#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
              case 0: case 1: case 2: case 3:
                ok = parseTOW( chr );
                break;
#endif

#ifdef GPS_FIX_LOCATION
              case 4: case 5: case 6: case 7:
                 ((uint8_t *)&m_fix.lon) [ chrCount-4 ] = chr;
                 break;
              case 8: case 9: case 10: case 11:
                 ((uint8_t *)&m_fix.lat) [ chrCount-8 ] = chr;
                 if (chrCount == 11)
                   m_fix.valid.location = true;
                 break;
#endif

#ifdef GPS_FIX_ALTITUDE
              case 16: case 17: case 18: case 19:
                ((uint8_t *)&m_fix.alt) [ chrCount-16 ] = chr;
                if (chrCount == 19) {
                  gps_fix::whole_frac *altp = &m_fix.alt;
                  int32_t height_MSLmm = *((int32_t *)altp);
//trace << PSTR(" alt = ") << height_MSLmm;
                  m_fix.alt.whole = height_MSLmm / 1000UL;
                  m_fix.alt.frac  = ((uint16_t)(height_MSLmm - (m_fix.alt.whole * 1000UL)))/10;
//trace << PSTR(" = ") << m_fix.alt.whole << PSTR(".");
//if (m_fix.alt.frac < 10) trace << '0';
//trace << m_fix.alt.frac;
                  m_fix.valid.altitude = true;
                }
                break;
#endif
            }
#endif
            break;

          case UBX_NAV_VELNED:
//if (chrCount == 0) trace << PSTR( "velned ");
#ifdef UBLOX_PARSE_VELNED
            switch (chrCount) {

#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
              case 0: case 1: case 2: case 3:
                ok = parseTOW( chr );
                break;
#endif

#ifdef GPS_FIX_SPEED
              case 20: case 21: case 22: case 23:
                ((uint8_t *)&m_fix.spd) [ chrCount-20 ] = chr;
                if (chrCount == 23) {
                  gps_fix::whole_frac *spdp = &m_fix.spd;
                  uint32_t ui = (*((uint32_t *)spdp) * 9UL);
                  m_fix.spd.whole = ui/250UL; // kph = cm/s * 3600/100000
                  m_fix.spd.frac = ui - (m_fix.spd.whole * 1000UL);
                  m_fix.valid.speed = true;
//trace << PSTR("spd = ") << m_fix.speed_mkn();
                }
                break;
#endif

#ifdef GPS_FIX_HEADING
              case 24: case 25: case 26: case 27:
                ((uint8_t *)&m_fix.hdg) [ chrCount-24 ] = chr;
                if (chrCount == 27) {
                  gps_fix::whole_frac *hdgp = &m_fix.hdg;
                  uint32_t ui = (*((uint32_t *)hdgp) * 36UL);
                  m_fix.hdg.whole = ui / 100000UL;
                  m_fix.hdg.frac  = (ui - (m_fix.hdg.whole * 100000UL))/1000UL;
                  m_fix.valid.heading = true;
//trace << PSTR(" hdg = ") << m_fix.heading_cd();
                }
                break;
#endif
            }
#endif
            break;

          case UBX_NAV_TIMEGPS:
//if (chrCount == 0) trace << PSTR( "timegps ");
#ifdef UBLOX_PARSE_TIMEGPS
            switch (chrCount) {

#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
              case 0: case 1: case 2: case 3:
                ok = parseTOW( chr );
                break;
              case 10:
                GPSTime::leap_seconds = (int8_t) chr;
                break;
              case 11:
                {
                  ublox::nav_timegps_t::valid_t &v = *((ublox::nav_timegps_t::valid_t *) &chr);
                  if (!v.leap_seconds)
                    GPSTime::leap_seconds = 0; // oops!
//else trace << PSTR("leap ") << GPSTime::leap_seconds << ' ';
                  if (GPSTime::leap_seconds != 0) {
                    if (!v.time_of_week) {
                      m_fix.valid.date =
                      m_fix.valid.time = false;
                    } else if ((GPSTime::start_of_week() == 0) &&
                               m_fix.valid.date && m_fix.valid.time) {
                      GPSTime::start_of_week( m_fix.dateTime );
//trace << m_fix.dateTime << '.' << m_fix.dateTime_cs;
                    }
                  }
                }
                break;
#endif
            }
#endif
            break;

          case UBX_NAV_TIMEUTC:
//if (chrCount == 0) trace << PSTR( " timeUTC ");
#ifdef UBLOX_PARSE_TIMEUTC
#if defined(GPS_FIX_TIME) | defined(GPS_FIX_DATE)
            switch (chrCount) {

#if defined(GPS_FIX_DATE)
              case 12: m_fix.dateTime.year   = chr; break;
              case 13: m_fix.dateTime.year   =
                        ((((uint16_t)chr) << 8) + m_fix.dateTime.year) % 100;
                break;
              case 14: m_fix.dateTime.month  = chr; break;
              case 15: m_fix.dateTime.date   = chr; break;
#endif
#if defined(GPS_FIX_TIME)
              case 16: m_fix.dateTime.hours   = chr; break;
              case 17: m_fix.dateTime.minutes = chr; break;
              case 18: m_fix.dateTime.seconds = chr; break;
#endif
              case 19:
                {
                  ublox::nav_timeutc_t::valid_t &v = *((ublox::nav_timeutc_t::valid_t *) &chr);
#if defined(GPS_FIX_DATE)
                  m_fix.valid.date = (v.UTC & v.time_of_week);
#endif
#if defined(GPS_FIX_TIME)
                  m_fix.valid.time = (v.UTC & v.time_of_week);
#endif
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
                  if (m_fix.valid.date &&
                      (GPSTime::start_of_week() == 0) &&
                      (GPSTime::leap_seconds != 0))
                    GPSTime::start_of_week( m_fix.dateTime );
#endif
//trace << m_fix.dateTime << PSTR(".") << m_fix.dateTime_cs;
//trace << ' ' << v.UTC << ' ' << v.time_of_week << ' ' << GPSTime::start_of_week();
                }
                break;
            }
#endif
#endif
            break;

          default:
            break;
        }
        break;
      case UBX_RXM:
      case UBX_INF:
      case UBX_ACK:
      case UBX_CFG:
        switch (rx().msg_id) {
          case UBX_CFG_MSG:
#ifdef UBLOX_PARSE_CFGMSG
#endif
            break;
          case UBX_CFG_RATE:
#ifdef UBLOX_PARSE_CFGRATE
#endif
            break;
          case UBX_CFG_NAV5:
#ifdef UBLOX_PARSE_CFGNAV5
#endif
            break;
          default:
            break;
        }
        break;
      case UBX_MON:
        switch (rx().msg_id) {
          case UBX_MON_VER:
#ifdef UBLOX_PARSE_MONVER
#endif
            break;
          default:
            break;
        }
        break;
      case UBX_AID:
      case UBX_TIM:
      case UBX_NMEA:
        break;
      default:
        break;
    }

    return ok;
}


bool ubloxGPS::parseFix( uint8_t c )
{
  static const gps_fix::status_t ubx[] __PROGMEM =
    {
      gps_fix::STATUS_NONE,
      gps_fix::STATUS_EST,   // dead reckoning only
      gps_fix::STATUS_STD,   // 2D
      gps_fix::STATUS_STD,   // 3D
      gps_fix::STATUS_STD,   // GPS + dead reckoning
      gps_fix::STATUS_TIME_ONLY
    };

  if (c >= membersof(ubx))
    return false;

  m_fix.status = (gps_fix::status_t) pgm_read_byte( &ubx[c] );
  m_fix.valid.status = true;

  return true;
}

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
