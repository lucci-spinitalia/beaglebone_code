#ifndef _GPS_GENERATE_H
#define GPS_GENERATE_H

#include "nmea/nmea.h"

/* Global variable */
extern nmeaINFO gps_generate_info;

/* Prototype */
double Double2GpsCoord(double coordinate);
double GpsCoord2Double(double gps_coord);
void gps_generate_init(int signal, int fix, double lat, double lon, double speed, double elv, double direction, int sat_inuse, int sat_inview, nmeaINFO *info_scr);
int gps_generate(float linear_velocity_km_h, float direction, long sample_rate_us, char *gps_message, nmeaINFO *info_scr);
#endif
