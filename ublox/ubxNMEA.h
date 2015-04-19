#ifndef _UBXNMEA_H_
#define _UBXNMEA_H_

/**
 * @file UBXNMEA.h
 * @version 2.1
 *
 * @section License
 * Copyright (C) 2014, SlashDevin
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#include "NMEAGPS.h"

/**
 * Enable/disable the parsing of specific NMEA sentences.
 *
 * Configuring out a sentence prevents its fields from being parsed.
 * However, the sentence type will still be recognized by /decode/ and 
 * stored in member /nmeaMessage/.  No valid flags would be available.
 *
 */
#define NMEAGPS_PARSE_PUBX_00
#define NMEAGPS_PARSE_PUBX_04

// Ublox proprietary messages do not have a message type.  These
// messages start with "$PUBX," which ends with the manufacturer ID.  The
// message type is actually specified by the first numeric field.  In order
// to parse these messages, /parse_mfr_ID/ must be overridden to set the
// /nmeaMessage/ to PUBX_00 during /parseCommand/.  When the first numeric
// field is completed by /parseField/, it may change /nmeamessage/ to one 
// of the other PUBX message types.

#if (defined(NMEAGPS_PARSE_PUBX_00) | defined(NMEAGPS_PARSE_PUBX_00))  \
        &                \
    !defined(NMEAGPS_PARSE_MFR_ID)
#error NMEAGPS_PARSE_MFR_ID must be defined in NMEAGPS.h in order to parse PUBX messages!
#endif

#ifndef NMEAGPS_DERIVED_TYPES
#error You must "#define NMEAGPS_DERIVED_TYPES" in NMEAGPS.h!
#endif

/**
 * NMEA 0183 Parser for ublox Neo-6 GPS Modules.
 *
 * @section Limitations
 * Very limited support for ublox proprietary NMEA messages.
 * Only NMEA messages of types PUBX,00 and PUBX,04 are parsed.
 */

class ubloxNMEA : public NMEAGPS
{
    ubloxNMEA( const ubloxNMEA & );

public:

    ubloxNMEA() {};

    /** ublox proprietary NMEA message types. */
    enum pubx_msg_t {
        PUBX_00 = NMEA_LAST_MSG+1,
        PUBX_04 = PUBX_00+4
    };
    static const nmea_msg_t PUBX_FIRST_MSG = (nmea_msg_t) PUBX_00;
    static const nmea_msg_t PUBX_LAST_MSG  = (nmea_msg_t) PUBX_04;

protected:
    bool parseMfrID( char chr )
      { bool ok;
        switch (chrCount) {
          case  1: ok = (chr == 'U'); break;
          case  2: ok = (chr == 'B'); break;
          default: if (chr == 'X') {
                     ok = true;
                     nmeaMessage = (nmea_msg_t) PUBX_00;
                   } else
                     ok = false;
                   break;
        }
        return ok;
      };

    bool parseField( char chr );
    bool parseFix( char chr );
} NEOGPS_PACKED;

#endif
