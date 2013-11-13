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
double latitude_start = 0;
double longitude_start = 0;

double Double2GpsCoord(double coordinate)
{
  double degree;
  double minute;
  
  degree = trunc(coordinate) * 100;
  minute = ((coordinate - trunc(coordinate)) * 60);

  return (degree + minute); 
}

double GpsCoord2Double(double gps_coord)
{
  double degree;
  double minute;
  
  degree = trunc(gps_coord / 100);
  minute = ((gps_coord - degree * 100) / 60);

  return (degree + minute); 
}

void gps_generate_init(int signal, int fix, double lat, double lon, double speed, double elv, double direction, int sat_inuse, int sat_inview, nmeaINFO *info_scr)
{
  int i;
  nmeaINFO *nmea_info;
  
  if(info_scr == NULL)
    nmea_info = &gps_generate_info;
  else
    nmea_info = info_scr;
  
  nmea_zero_INFO(nmea_info);

  nmea_info->sig = signal;
  nmea_info->fix = fix;
  nmea_info->lat = lat;
  nmea_info->lon = lon;
  nmea_info->speed = speed;
  nmea_info->elv = elv;
  nmea_info->direction = direction;

  //nmea_info->satinfo.inuse = sat_inuse;
  nmea_info->satinfo.inview = sat_inview;
  
  if(sat_inuse > sat_inview)
    sat_inuse = sat_inview;
  
  for(i = 0; i < sat_inuse; i++)
  {
    nmea_info->satinfo.sat[i].id = i + 1;
    nmea_info->satinfo.sat[i].in_use = 1;
  }
  
  latitude_start = GpsCoord2Double(lat);
  longitude_start = GpsCoord2Double(lon);
}

int gps_generate(float linear_velocity_km_h, float yaw_rate_rps, long sample_rate_us, char *gps_message, nmeaINFO *info_scr)
{
  char buff[2048];
  int gen_sz;
  double delta_position = 0;
  static double x, y;
  //static double direction = 0;
  nmeaINFO *nmea_info;
  
  if(info_scr == NULL)
    nmea_info = &gps_generate_info;
  else
    nmea_info = info_scr;
  
  gen_sz = nmea_generate(&buff[0], 2048, nmea_info, GPGGA | GPGSA | GPGSV | GPRMC | GPVTG);

  buff[gen_sz] = 0;
  sprintf(gps_message, "%s", &buff[0]);

  nmea_info->speed = linear_velocity_km_h;
  nmea_info->utc.sec += 1;

  if(nmea_info->utc.sec == 60)
  {
    nmea_info->utc.sec = 0;
    nmea_info->utc.min++;

    if(nmea_info->utc.min == 60)
    {
      nmea_info->utc.min = 0;
      nmea_info->utc.hour++;
    }
  }
  //printf("Velocity: %f Yaw: %f\n", linear_velocity_km_h, yaw_rate_rps);
  // 1/60 di latitudine sono circa 1.8 km
  delta_position = ((linear_velocity_km_h / 3.6) * sample_rate_us) / 1000000; // [m]
  x += delta_position * sin(nmea_info->direction * 3.14 / 180);
  y += delta_position * cos(nmea_info->direction  * 3.14 / 180);

  //direction += (yaw_rate_rps * sample_rate_us) / 1000000;
  // Add a scale factor to yaw_rate_rps due the friction and the weigth
  nmea_info->direction += ((yaw_rate_rps * sample_rate_us) / 1000000) * 180 / 3.14;  // degree
  
  if(nmea_info->direction < 0)
    nmea_info->direction += 360;
  
  while(nmea_info->direction > 360)
    nmea_info->direction -= 360;
  /*
   For the y coordinate, 
   we can use the north-south distance between two lines of latitude:

       y = R*(b2-b1)*pi/180

    Here, I have converted the latitude difference, (b2-b1), from degrees 
    to radians, by multiplying it by (pi radians)/(180 degrees). The 
    product of the angle in radians and the radius is the arc length in 
    the same units as the radius.

    For the x coordinate, we can use the distance along a line of latitude 
    from one line of longitude to the other:

      x = R*(a2-a1)*(pi/180)*cos(b1)

    Here we have an additional factor, the cosine of the latitude along 
    which we are measuring. The line of latitude is a circle with a 
    smaller radius than that of the equator; it is reduced by the factor 
    cos(b1).
   
    Solving 
    for lon2,lat2 in terms of x,y:

    lat2 = lat1 + y*180/(pi*R)
    lon2 = lon1 + x*180/(pi*R*cos(lat1))

    where R is the radius of the earth, R = 6367 km
   */
  nmea_info->lon = Double2GpsCoord(longitude_start + x * 180 / (3.14 * 6367000) * cos(GpsCoord2Double(nmea_info->lat) * 3.14 / 180));
  nmea_info->lat = Double2GpsCoord(latitude_start + y * 180 / (3.14 * 6367000));
  
  //printf("Generate lon: %f lat: %f Direction: %f\n",
  //       longitude_start + x * 180 / (3.14 * 6367000) * cos(GpsCoord2Double(nmea_info->lat) * 3.14 / 180),
  //       latitude_start + y * 180 / (3.14 * 6367000), nmea_info->direction);
  
  //nmea_info->direction = (direction * 180) / 3.14;  //degree
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
