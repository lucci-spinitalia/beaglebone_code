#include "nmea/nmea.h"
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/select.h>
#include <unistd.h>
#include <math.h>

nmeaINFO info;
  
void gps_generate_init(int signal, int fix, double lat, double lon, double speed, double elv, double direction, int sat_inuse, int sat_inview)
{
  nmea_zero_INFO(&info);

  info.sig = signal;
  info.fix = fix;
  info.lat = lat;
  info.lon = lon;
  info.speed = speed;
  info.elv = elv;
  info.direction = direction;

  info.satinfo.inuse = sat_inuse;
  info.satinfo.inview = sat_inview;
}

int gps_generate(double linear_velocity_km_h, double direction)
{
  char buff[2048];
  int gen_sz;
  
  gen_sz = nmea_generate(&buff[0], 2048, &info, GPGGA | GPGSA | GPGSV | GPRMC | GPVTG);

  buff[gen_sz] = 0;
  printf("%s", &buff[0]);

  info.speed = linear_velocity_km_h;
  info.utc.sec += 1;
	
  if(info.utc.sec == 60)
  {
    info.utc.sec = 0;
    info.utc.min++;
	  
    if(info.utc.min == 60)
    {
      info.utc.min = 0;
  	  info.utc.hour++;
    }
  }
  
  // 1/60 di latitudine sono circa 1.8 km
  info.lon += (linear_velocity_km_h * sin(direction * 3.14 / 180) / (1.83 * 3600));
  info.lat += (linear_velocity_km_h * cos(direction * 3.14 / 180) / (1.83 * 3600));

  return 1;
}

/*int main()
{
  gps_generate_init(3, 3, 5000.0, 3600.0, 0.0, 10.86, 0.0, 2, 2);*/
  
  /*
  info.satinfo.sat[0].id = 1;
  info.satinfo.sat[0].in_use = 1;
  info.satinfo.sat[0].elv = 50;
  info.satinfo.sat[0].azimuth = 0;
  info.satinfo.sat[0].sig = 99;
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
