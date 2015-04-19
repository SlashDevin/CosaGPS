#ifndef STREAMERS_H
#define STREAMERS_H

#include <Cosa/IOStream.hh>

class gps_fix;

/**
 * Print valid fix data to the given stream with the format
 *   "status,dateTime,lat,lon,heading,speed,altitude,satellites,
 *       hdop,vdop,pdop,lat_err,lon_err,alt_err"
 * The "header" above contains the actual compile-time configuration.
 * A comma-separated field will be empty if the data is NOT valid.
 * @param[in] outs output stream.
 * @param[in] fix gps_fix instance.
 * @return iostream.
 */
extern IOStream & operator <<( IOStream &outs, const gps_fix &fix );

class NMEAGPS;

extern uint32_t seconds;

extern void trace_header();
extern void trace_all( const NMEAGPS &gps, const gps_fix &fix );

#endif