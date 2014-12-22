#include "ubxNMEA.h"

#include "Cosa/Trace.hh"

static char toHexDigit( uint8_t val )
{
  val &= 0x0F;
  return (val >= 10) ? ((val - 10) + 'A') : (val + '0');
}

static const char pubx[] __PROGMEM =  "PUBX";
const char * const ubloxNMEA::ublox_nmea[] __PROGMEM = { pubx };
const uint8_t ubloxNMEA::ublox_nmea_size = membersof(ublox_nmea);

const NMEAGPS::msg_table_t ubloxNMEA::ublox_msg_table __PROGMEM =
  {
    ubloxNMEA::PUBX_FIRST_MSG,
    &NMEAGPS::nmea_msg_table,
    ubloxNMEA::ublox_nmea_size,
    ubloxNMEA::ublox_nmea
  };

//---------------------------------------------

bool ubloxNMEA::parseField(char chr)
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

    default:
      //  
      return NMEAGPS::parseField(chr);
  }

  return ok;
}