#include <nmea/nmea.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/select.h>

#ifdef NMEA_WIN
#   include <windows.h>
#else
#   include <unistd.h>
#endif

int main()
{
  nmeaINFO info;
  char buff[2048];
  int gen_sz;
  int it;

  nmea_zero_INFO(&info);

  info.sig = 3;
  info.fix = 3;
  info.lat = 5000.0;
  info.lon = 3600.0;
  info.speed = 2.14 * NMEA_TUS_MS;
  info.elv = 10.86;

  info.satinfo.inuse = 1;
  info.satinfo.inview = 1;

  /*
  info.satinfo.sat[0].id = 1;
  info.satinfo.sat[0].in_use = 1;
  info.satinfo.sat[0].elv = 50;
  info.satinfo.sat[0].azimuth = 0;
  info.satinfo.sat[0].sig = 99;
  */
  
  int select_result = -1; // value returned frome select()
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

    gen_sz = nmea_generate(&buff[0], 2048, &info, GPGGA | GPGSA | GPGSV | GPRMC | GPVTG);

    buff[gen_sz] = 0;
    printf("%s", &buff[0]);

/*#ifdef NMEA_WIN
        Sleep(500);
#else
        usleep(500000);
#endif*/

    info.speed += .1;
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
	
	select_timeout.tv_sec = 1;
    select_timeout.tv_usec = 0;
  }

  return 0;
}
