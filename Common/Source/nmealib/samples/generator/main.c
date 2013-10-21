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
  nmeaGENERATOR *gen;
  nmeaINFO info;
  char buff[2048];
  int gen_sz;
  int it;

  nmea_zero_INFO(&info);
  
  if(0 == (gen = nmea_create_generator(NMEA_GEN_STATIC/*NMEA_GEN_ROTATE*/, &info)))
    return -1;

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
      
    gen_sz = nmea_generate_from(&buff[0], 2048, &info, gen, GPGGA | GPGSA | GPGSV | GPRMC | GPVTG);

    buff[gen_sz] = 0;
    printf("%s\n", &buff[0]);

	select_timeout.tv_sec = 1;
    select_timeout.tv_usec = 0;
	/*
#ifdef NMEA_WIN
        Sleep(500);
#else
        usleep(500000);        
#endif*/
  }
  nmea_gen_destroy(gen);

  return 0;
}
