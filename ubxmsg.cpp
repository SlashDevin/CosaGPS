#include "ubxmsg.h"

using namespace ublox;

bool ublox::configNMEA( ubloxGPS &gps, NeoGPS::nmea_msg_t msgType, uint8_t rate )
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

  uint8_t msg_index = (uint8_t) msgType - (uint8_t) NeoGPS::NMEA_FIRST_MSG;

  if (msg_index >= membersof(ubx))
    return false;

  ubloxGPS::msg_id_t msg_id = (ubloxGPS::msg_id_t) pgm_read_byte( &ubx[msg_index] );

  return gps.send( cfg_msg_t( ubloxGPS::UBX_NMEA, msg_id, rate ) );
}
