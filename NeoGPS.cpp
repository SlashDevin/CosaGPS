/**
 * @file NeoGPS.cpp
 * @version 2.0
 *
 * @section License
 * Copyright (C) 2014, Thomas Lohmueller
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

#include "NeoGPS.h"

#include "Cosa/IOStream/Driver/UART.hh"

#ifndef CR
#define CR (13)
#endif
#ifndef LF
#define LF (10)
#endif

/**
 * parseHEX(char a)
 * Parses a single character as HEX and returns byte value.
 */
inline static uint8_t parseHEX(char a) {
    a |= 0x20; // make it lowercase
    if (('a' <= a) && (a <= 'f'))
        return a - 'a' + 10;
    else
        return a - '0';
}

static void end_frac( volatile uint8_t & val )
{
  if (val == 0xFF)
    val = 0;      // no digits
  else if ((val & 0xF0) == 0xF0)
    val <<= 4;    // only one significant digit
  else if ((val & 0x0F) == 0x0F)
    val &= ~0x0F; // last digit 0
}

static void end_frac( volatile uint16_t & val )
{
  const uint16_t mask = 0xF000;

  // shift significant digits to the top (over any BCD 0xF's),
  //    shifting zeros in the least significant digits
  while ((val & mask) == mask)
    val <<= 4;
}

static void zero_append( volatile uint16_t & val )
{
  uint16_t mask = 0x000F;
  while (mask && ((val & mask) == mask)) {
    val &= ~mask;
    mask <<= 4;
  }
}

void NeoGPS::rxBegin()
{
    crc = 0;
    nmeaMessage = NMEA_UNKNOWN;
    rxState = NMEA_WAIT_FOR_COMMAND;
    chrCount = 0;
}


void NeoGPS::rxEnd( bool ok )
{
  rxState = NMEA_IDLE; // just in case Event::Handler cares about rxState

  valid.all = 0;

  if (ok) {
    // mark specific received data as valid   
    switch (nmeaMessage) {
      case NMEA_RMC:
        if (stat == GPS_FIX_NONE)
          ok = false;
        else {
          end_frac( speed_frac );
          zero_append( heading );
          valid.location = true;
          valid.dateTime = true;
          valid.speed    = true; 
          valid.heading  = true;
        }
        break;

      case NMEA_GGA:
        if (stat == GPS_FIX_NONE)
          ok = false;
        else {
          end_frac( alt_frac );
          valid.altitude = true;
          valid.location = true;
          valid.dateTime = true;
        }
        break;

      case NMEA_GLL:
        if (stat == GPS_FIX_NONE)
          ok = false;
        else {
          valid.location = true;
          valid.dateTime = true;
        }
        break;

      case NMEA_VTG:
        if (stat == GPS_FIX_NONE)
          ok = false;
        else {
          end_frac( speed_frac );
          zero_append( heading );
          valid.speed    = true; 
          valid.heading  = true;
        }
        break;

      case NMEA_ZDA:
        if (dateTime.date == 0)
          ok = false;
        else
          valid.dateTime = true;
        break;
      default:
        break;
    }
  }

#ifdef NEOGPS_STATS
    statistics.parser_ok++;
#endif

  if (ok)
    Event::push( Event::RECEIVE_COMPLETED_TYPE, this, nmeaMessage );
}


int NeoGPS::putchar( char c )
{
    if (c == '$') {  // Always restarts
      rxBegin();

    } else {
      switch (rxState) {
        case NMEA_IDLE:
            break;

            // Wait until full command name is received
        case NMEA_WAIT_FOR_COMMAND:
            crc ^= c;      // part of CRC calculation
            
            if (c == ',') {   // comma marks end of command name

                rxState = NMEA_RECEIVING_DATA;
                fieldIndex = 0;
                chrCount = 0;
                
            } else if ((c < 'A') || ('Z' < c)) {
                // abort command parsing on invalid character
                rxState = NMEA_IDLE;

            } else {
              // A nested FSM to handle the command name
              switch (chrCount++) {
                case 0:
                  if (c != 'G')
                    rxState = NMEA_IDLE;
                  break;
                case 1:
                  if (c != 'P')
                    rxState = NMEA_IDLE;
                  break;
                case 2:
                  if      (c == 'G') ; // ok but several choices
                  else if (c == 'Z') nmeaMessage = NMEA_ZDA;
                  else if (c == 'R') nmeaMessage = NMEA_RMC;
                  else if (c == 'V') nmeaMessage = NMEA_VTG;
                  else
                    rxState = NMEA_IDLE;
                  break;
                case 3:
                  if (((c == 'D') && (nmeaMessage == NMEA_ZDA)) ||
                      ((c == 'M') && (nmeaMessage == NMEA_RMC)) ||
                      ((c == 'T') && (nmeaMessage == NMEA_VTG))
                     )
                    ; // ok so far...
                  else if (nmeaMessage == NMEA_UNKNOWN) {
                    if (c == 'G')
                      nmeaMessage = NMEA_GGA;
                    else if (c == 'S')
                      nmeaMessage = NMEA_GSA;
                    else if (c == 'L')
                      nmeaMessage = NMEA_GLL;
                    else
                      rxState = NMEA_IDLE;
                  } else
                    rxState = NMEA_IDLE;
                  break;
                case 4:
                  if (((c == 'A') &&
                       ((nmeaMessage == NMEA_GGA) ||
                        (nmeaMessage == NMEA_GSA) ||
                        (nmeaMessage == NMEA_ZDA)))             ||
                      ((c == 'C') && (nmeaMessage == NMEA_RMC)) ||
                      ((c == 'L') && (nmeaMessage == NMEA_GLL)) ||
                      ((c == 'G') && (nmeaMessage == NMEA_VTG)))
                    ; // ok so far, comma better be next...
                  else if ((c == 'V') && (nmeaMessage == NMEA_GSA))
                    nmeaMessage = NMEA_GSV; // not GSA after all
                  else
                    rxState = NMEA_IDLE;
                  break;
                case 5:
                  // Too many letters?
                  rxState = NMEA_IDLE;
                  break;
              }
            }
            break;
            
            
            // Wait until complete line is received
        case NMEA_RECEIVING_DATA:
            if (c == '*') {   // Line finished, CRC follows
                rxState = NMEA_RECEIVING_CRC1;

            } else if ((c == CR) || (c == LF)) { // Line finished, no CRC
                rxEnd( true );

            } else if ((c < ' ') || ('~' < c)) { // Invalid char
                rxEnd( false );

            } else {            // normal data character
                crc ^= c;
                if (c == ',') { // a comma marks the next field
                    fieldIndex++;
                    chrCount = 0;
                } else {
                    parseField(c);
                    chrCount++;
                }
            }
            break;
            
            
            // Receiving first CRC character
        case NMEA_RECEIVING_CRC1:
            if (crc>>4 != parseHEX(c)) { // mismatch, count as CRC error
#ifdef NEOGPS_STATS
                statistics.parser_crcerr++;
#endif
                rxEnd( false );
            } else  // valid first CRC nibble
                rxState = NMEA_RECEIVING_CRC2;
            break;
            
            
            // Receiving second CRC character, parse line if CRC matches
        case NMEA_RECEIVING_CRC2:
            if ((crc & 0x0F) != parseHEX(c)) {// CRCs do not match
#ifdef NEOGPS_STATS
                statistics.parser_crcerr++;
#endif
                rxEnd( false );
            } else  // valid second CRC nibble
                rxEnd( true );
            break;
      }
    }

    return c;
}

bool NeoGPS::parseField(char chr)
{
    bool ok = true;

    switch (nmeaMessage) {

        case NMEA_GGA:
            switch (fieldIndex) {
                case 0:                 // Time  HHMMSS.ss
                    ok = parseTimeField(chr);
                    break;
                case 1:                 // latitude
                    parseDDMM( location.lat, chr );
                    break;
                case 2:                 // N/S indicator
                    endDDMM( location.lat );
                    if (chr == 'S')
                      location.lat = -location.lat;
                    break;
                case 3:                 // longitude
                    parseDDMM( location.lon, chr );
                    break;
                case 4:                 // E/W indicator
                    endDDMM( location.lon );
                    if (chr == 'W')
                      location.lon = -location.lon;
                    break;
                case 5:                 // position fix status
                    stat   = (enum gps_fix_status_t) (chr - '0');
                    satCnt = 0;
                    hdop   = 0;
                    break;
                case 6:                 // number of satellites
                    satCnt = satCnt*10 + (chr - '0');
                    break;
                case 7:                 // HDOP
                    if (chr != '.')
                      hdop = hdop*10 + (chr - '0');
                    break;
                case 8:                 // Altitude
                    parseAltitude( chr );
                    break;
                default:
                    ok = false;
                    break;
            }
            break;

        case NMEA_GLL:
            switch (fieldIndex) {
                case 0:                 // latitude
                    parseDDMM( location.lat, chr );
                    break;
                case 1:                 // N/S indicator
                    endDDMM( location.lat );
                    if (chr == 'S')
                      location.lat = -location.lat;
                    break;
                case 2:                 // longitude
                    parseDDMM( location.lon, chr );
                    break;
                case 3:                 // E/W indicator
                    endDDMM( location.lon );
                    if (chr == 'W')
                      location.lon = -location.lon;
                    break;
                case 4:                 // Time  HHMMSS.ss
                    ok = parseTimeField(chr);
                    break;
                case 5:
                    ok = parseFixStatus( chr );
                    break;
                case 6:                // fix mode
                    ok = parseFixMode( chr );
                    break;
            }
            break;

        case NMEA_GSA:
        case NMEA_GSV:
            break;
                  
        case NMEA_RMC:
            switch (fieldIndex) {
                case 0:                  // Time  HHMMSS.ss
                    ok = parseTimeField(chr);
                    break;
                case 1:
                    ok = parseFixStatus( chr );
                    break;
                case 2:                   // latitude
                    parseDDMM( location.lat, chr );
                    break;
                case 3:                  // N/S indicator
                    endDDMM( location.lat );
                    if (chr == 'S')
                      location.lat = -location.lat;
                    break;
                case 4:                  // longitude
                    parseDDMM( location.lon, chr );
                    break;
                case 5:                  // E/W indicator
                    endDDMM( location.lon );
                    if (chr == 'W')
                      location.lon = -location.lon;
                    break;
                case 6:                  // speed
                    parseSpeed( chr );
                    break;
                case 7:                  // heading
                    parseHeading( chr );
                    break;
                case 8:                 // DDMMYY
                    switch (chrCount) {
                      case 0: dateTime.date   = (chr - '0')<<4; break;
                      case 1: dateTime.date  |= (chr - '0');    break;
                      case 2: dateTime.month  = (chr - '0')<<4; break;
                      case 3: dateTime.month |= (chr - '0');    break;
                      case 4: dateTime.year   = (chr - '0')<<4; break;
                      case 5: dateTime.year  |= (chr - '0');    break;
                      default: ok = false;                      break;
                    }
                    break;
                case 9:                // fix mode
                    ok = parseFixMode( chr );
                    break;
                default:
                    ok = false;
                    break;
            }
            break;

        case NMEA_VTG:
            switch (fieldIndex) {
                case 0:
                    parseHeading( chr );
                    break;
                case 1:
                    ok = (chr == 'T');
                    break;
                case 3:
                    // don't care about magnetic heading
                    break;
                case 4:
                    ok = (chr == 'M');
                    break;
                case 5:
                    parseSpeed( chr );
                    break;
                case 6:
                    ok = (chr == 'N');
                    break;
                case 7:
                    // don't care about speed in kph
                    break;
                case 8:
                    ok = (chr == 'K');
                    break;
                case 9:                // fix mode
                    ok = parseFixMode( chr );
                    break;
            }
            break;

        case NMEA_ZDA:
            switch (fieldIndex) {
                case 0:                         // Time  HHMMSS.ss
                    ok = parseTimeField(chr);
                    dateTime.date  = 0;
                    dateTime.month = 0;
                    dateTime.year  = 0;
                    break;
                case 1:                         // Date
                    dateTime.date  = (dateTime.date <<4) | (chr - '0');
                    break;
                case 2:                         // Month
                    dateTime.month = (dateTime.month<<4) | (chr - '0');
                    break;
                case 3:                         // Year
                    dateTime.year  = (dateTime.year <<4) | (chr - '0');
                    break;
                default:
                    ok = false;
                    break;
            }
            break;

        default:
            ok = false;
            break;
    }

    return ok;
}


inline bool NeoGPS::parseTimeField(char chr)
{
  switch (chrCount) {
      case 0: dateTime.hours    = (chr - '0')<<4; return true;
      case 1: dateTime.hours   |= (chr - '0');    return true;
      case 2: dateTime.minutes  = (chr - '0')<<4; return true;
      case 3: dateTime.minutes |= (chr - '0');    return true;
      case 4: dateTime.seconds  = (chr - '0')<<4; return true;
      case 5: dateTime.seconds |= (chr - '0');    return true;
      case 6: if (chr == '.')                     return true;
      case 7: dateTime_cs       = (chr - '0')<<4; return true;
      case 8: dateTime_cs      |= (chr - '0');    return true;
  }

  return false;
}

bool NeoGPS::parseFixMode( char chr )
{
  bool ok = true;

  if (chr == 'A')
    stat = GPS_FIX_STD;
  else if (chr == 'N')
    stat = GPS_FIX_NONE;
  else if (chr == 'D')
    stat = GPS_FIX_DGPS;
  else if (chr == 'E')
    stat = GPS_FIX_EST;
  else
    ok = false;

  return ok;
}

bool NeoGPS::parseFixStatus( char chr )
{
  bool ok = true;

  if (chr == 'A')
    stat = GPS_FIX_STD;
  else if (chr == 'V')
    stat = GPS_FIX_NONE;
  else
    ok = false;

  return ok;
}

void NeoGPS::parseSpeed( char chr )
{
  if (chrCount == 0) {
    speed        = 0;
    speed_frac   = 0xFFFF;
    before_decimal = true;
  }
  if (chr == '.')
    before_decimal = false;
  else if (before_decimal) {
    if (chrCount < 2)
      speed = (speed<<4) | (chr - '0');
    else // too many chars!
      speed = 0x99;
  } else if ((speed_frac & 0xF000) == 0xF000)
    // only keep the first 4 digits after the decimal
    speed_frac = (speed_frac<<4) | (chr - '0');
}

void NeoGPS::parseHeading( char chr )
{
  if (chrCount == 0) {
    heading      = 0;
    before_decimal = true;
  }
  if (chr == '.') {
    before_decimal = false;
    heading = (heading<<4) | 0x0F;
  } else if (before_decimal) {
    if (chrCount < 3)
      heading = (heading<<4) | (chr - '0');
    else // too many chars!
      heading = 0;
  } else if ((((uint8_t)heading) & 0x0F) == 0x0F) {
    // only keep the first digit after the decimal
    heading &= 0xFFF0;
    heading |= (chr - '0');
  }
}

void NeoGPS::parseAltitude( char chr )
{
  if (chrCount == 0) {
    alt      = 0;
    alt_frac = 0xFF;
    before_decimal = true;
  }
  if (chr == '.')
    before_decimal = false;
  else if (before_decimal) {
    if (chrCount < 4)
      alt = (alt<<4) | (chr - '0');
    else // too many chars!
      alt = 0x9999;
  } else if ((alt_frac & 0xF0) == 0xF0)
    // only keep the first two digits after the decimal
    alt_frac = (alt_frac<<4) | (chr - '0');
}


void NeoGPS::poll( nmea_msg_t msg ) const
{
  static const char pm0[] __PROGMEM = "EIGPQ,GGA";
  static const char pm1[] __PROGMEM = "EIGPQ,GLL";
  static const char pm2[] __PROGMEM = "EIGPQ,GSA";
  static const char pm3[] __PROGMEM = "EIGPQ,GSV";
  static const char pm4[] __PROGMEM = "EIGPQ,RMC";
  static const char pm5[] __PROGMEM = "EIGPQ,VTG";
  static const char pm6[] __PROGMEM = "EIGPQ,ZDA";
  static const char * const poll_msgs[] __PROGMEM = { pm0, pm1, pm2, pm3, pm4, pm5, pm6 };

  if ((NMEA_FIRST_MSG <= msg) && (msg <= NMEA_LAST_MSG))
    send_P( (const char *) pgm_read_word(&poll_msgs[msg-NMEA_FIRST_MSG]) );
}


static char toHexDigit( uint8_t val )
{
  val &= 0x0F;
  return (val >= 10) ? ((val - 10) + 'A') : (val + '0');
}


bool NeoGPS::send_header( const char * & msg ) const
{
  if (msg && *msg) {
    m_device->putchar('$');
    if (*msg == '$')
      msg++;
    return true;
  }
  return false;
}


void NeoGPS::send_trailer( uint8_t crc ) const
{
  m_device->putchar('*');

  char hexDigit = toHexDigit( crc>>4 );
  m_device->putchar( hexDigit );

  hexDigit = toHexDigit( crc );
  m_device->putchar( hexDigit );

  m_device->putchar( CR );
  m_device->putchar( LF );
}


void NeoGPS::send( const char *msg ) const
{
  if (send_header( msg )) {
    uint8_t crc = 0;
    while (*msg) {
      crc ^= *msg;
      m_device->putchar( *msg++ );
    }

    send_trailer( crc );
  }
}

void NeoGPS::send_P( const char *msg ) const
{
  if (send_header( msg )) {
    uint8_t crc = 0;
    for(;;) {
      uint8_t chr = pgm_read_byte(msg);
      if (!chr)
        break;
      crc ^= chr;
      m_device->putchar( chr );
      msg++;
    }

    send_trailer( crc );
  }
}
