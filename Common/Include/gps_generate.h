#ifndef _GPS_GENERATE_H
#define GPS_GENERATE_H

#include "nmea/nmea.h"

/* Global variable */
extern nmeaINFO gps_generate_info;

/* Prototype */
void gps_generate_init(int signal, int fix, double lat, double lon, double speed, double elv, double direction, int sat_inuse, int sat_inview);
int gps_generate(float linear_velocity_km_h, float yaw_rate_rps, long sample_rate_us, char *gps_message);

#endif
