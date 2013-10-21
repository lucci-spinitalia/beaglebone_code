#include "gps_generate.h"
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/select.h>
#include <unistd.h>
#include <math.h>
#include <sys/stat.h>
#include <fcntl.h>

#define DEVICE_NAME "gps_debug"

nmeaINFO gps_generate_info;
int gps_file;
  
void gps_generate_init(int signal, int fix, double lat, double lon, double speed, double elv, double direction, int sat_inuse, int sat_inview)
{
  nmea_zero_INFO(&gps_generate_info);

  gps_generate_info.sig = signal;
  gps_generate_info.fix = fix;
  gps_generate_info.lat = lat;
  gps_generate_info.lon = lon;
  gps_generate_info.speed = speed;
  gps_generate_info.elv = elv;
  gps_generate_info.direction = direction;

  gps_generate_info.satinfo.inuse = sat_inuse;
  gps_generate_info.satinfo.inview = sat_inview;
}

int gps_generate(float linear_velocity_km_h, float yaw_rate_rps, long sample_rate_us, char *gps_message)
{
  char buff[2048];
  int gen_sz;
  static float actual_position = 0;
  static float direction = 0;
  
  gen_sz = nmea_generate(&buff[0], 2048, &gps_generate_info, GPGGA | GPGSA | GPGSV | GPRMC | GPVTG);

  buff[gen_sz] = 0;
  sprintf(gps_message, "%s", &buff[0]);

  gps_generate_info.speed = linear_velocity_km_h;
  gps_generate_info.utc.sec += 1;
	
  if(gps_generate_info.utc.sec == 60)
  {
    gps_generate_info.utc.sec = 0;
    gps_generate_info.utc.min++;
	  
    if(gps_generate_info.utc.min == 60)
    {
      gps_generate_info.utc.min = 0;
      gps_generate_info.utc.hour++;
    }
  }
  printf("Velocity: %f Yaw: %f\n", linear_velocity_km_h, yaw_rate_rps);
  actual_position += (linear_velocity_km_h * sample_rate_us) / 1000000;
  direction += (yaw_rate_rps * sample_rate_us) / 1000000;
  
  // 1/60 di latitudine sono circa 1.8 km
  gps_generate_info.lon += (actual_position * sin(direction) / (1.83 * 3600));
  gps_generate_info.lat += (actual_position * cos(direction) / (1.83 * 3600));

  return 1;
}

/*int main()
{
  gps_generate_init(3, 3, 5000.0, 3600.0, 0.0, 10.86, 0.0, 2, 2);*/
  
  /*
  gps_generate_info.satinfo.sat[0].id = 1;
  gps_generate_info.satinfo.sat[0].in_use = 1;
  gps_generate_info.satinfo.sat[0].elv = 50;
  gps_generate_info.satinfo.sat[0].azimuth = 0;
  gps_generate_info.satinfo.sat[0].sig = 99;
  */
  
/*  int select_result = -1; // value returned frome select()
  struct timeval select_timeout;
  select_timeout.tv_sec = 1;
  select_timeout.tv_usec = 0;
  
  while(1)
  {
	fflush(stdout);

    select_result = select(1, NULL, NULL, NULL, &select_timeout);

    if(select_result == -1 && errno == EAGAIN)
    {
      perror("select");
      continue;
    }

    if(select_result == -1)
    {
      perror("main:");

      return 1;
    }

	gps_generate(1, 45.0);
	
	select_timeout.tv_sec = 1;
    select_timeout.tv_usec = 0;
  }

  return 0;
}*/
