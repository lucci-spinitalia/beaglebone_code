#include <nmea/nmea.h>
#include <string.h>
#include <stdio.h>

#include <time.h> 
#include <sys/ioctl.h> 
#include <fcntl.h> 
#include <errno.h>

#include <stdlib.h>
#include <unistd.h>
#include "rs232.h"

/* Macro */
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

int main(int argc, char *argv[])
{
  int it = 0;
  int done = 0;
    
  nmeaINFO info;
  nmeaPARSER parser;
    
  int select_result = -1; // value returned frome select()
  struct timeval select_timeout;
  int nfds = 0;
  fd_set rset;
    
  char rs232_device;
  int bytes_read;
  char rs232_buffer[1024];
  char *token;
  char nmea_message[256];

  rs232_device = com_open("/dev/ttyO2", 9600, 'N', 8, 1);
  
  if(rs232_device < 0)
    perror("com_open");
    
  // Select UART2_TX and set it as output
  printf("Setting tx. . .\n");
  sprintf(rs232_buffer, "echo 11 > /sys/kernel/debug/omap_mux/spi0_d0");
  if(system(rs232_buffer) < 0)
    perror("setting tx");
  
  // Select UART1_RX and set it as input pulled up
  printf("Setting rx. . .\n");
  sprintf(rs232_buffer, "echo 39 > /sys/kernel/debug/omap_mux/spi0_sclk");
  if(system(rs232_buffer) < 0)
    perror("setting rx");
    
  const char *buff[] = {
      "$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n",
      "$GPGGA,111609.14,5001.27,N,3613.06,E,3,08,0.0,10.2,M,0.0,M,0.0,0000*70\r\n",
      "$GPGSV,2,1,08,01,05,005,80,02,05,050,80,03,05,095,80,04,05,140,80*7f\r\n",
      "$GPGSV,2,2,08,05,05,185,80,06,05,230,80,07,05,275,80,08,05,320,80*71\r\n",
      "$GPGSA,A,3,01,02,03,04,05,06,07,08,00,00,00,00,0.0,0.0,0.0*3a\r\n",
      "$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n",
      "$GPVTG,217.5,T,208.8,M,000.00,N,000.01,K*4C\r\n"
  };
 

  select_timeout.tv_sec = 1;
  select_timeout.tv_usec = 0;
    
  nmea_zero_INFO(&info);
  nmea_parser_init(&parser);

  while(!done)
  { 
    fflush(stdout);
      
    FD_ZERO(&rset);

    if(rs232_device > 0)
    {
      FD_SET(rs232_device, &rset);
	  nfds = max(nfds, rs232_device);
    }
    
    select_result = select(nfds + 1, &rset, NULL, NULL, NULL);

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
      
    if(rs232_device > 0)
    {
      if(FD_ISSET(rs232_device, &rset))
      {
        //bytes_read = read(rs232_device, rs232_buffer, sizeof(rs232_buffer));
        bytes_read = rs232_read(rs232_device);

        if(bytes_read > 0)
        {
          if(rs232_check_last_char('\n'))
          {
            bytes_read = rs232_unload_rx(rs232_buffer);
        
            if(bytes_read > 0)
            {
              rs232_buffer[bytes_read] = 0;

              token = strtok(rs232_buffer, "\n");
              while(token != NULL)
              {
                sprintf(nmea_message, "%s\n", token);
                nmea_parse(&parser, nmea_message, (int)strlen(nmea_message), &info);
  
                if(it > 0)
                {
                  printf("\033[14A");
                }
                else
                  it++;
      
                printf("Time: %i/%i/%i %i:%i:%i.%i\n", info.utc.day, info.utc.mon + 1, info.utc.year + 1900, info.utc.hour, info.utc.min, info.utc.sec, info.utc.hsec);
                printf("Signal: %i\n", info.fix);
                printf("Position Diluition of Precision: %f\n", info.PDOP);
                printf("Horizontal Diluition of Precision: %f\n", info.HDOP);
                printf("Vertical Diluition of Precisione: %f\n", info.VDOP);
                printf("Latitude: %f\n", info.lat);
                printf("Longitude: %f\n", info.lon);
                printf("Elevation: %f m\n", info.elv);
                printf("Speed: %f km/h\n", info.speed);
                printf("Direction: %f degrees\n", info.direction);
                printf("Magnetic variation degrees: %f\n", info.declination); 
    
                printf("\nSatellite: \tin view: %i\n\t\tin use: %i\n", info.satinfo.inview, info.satinfo.inuse);
      
                token = strtok(NULL, "\n");
              }
            }
          }
        }	  
      }
    }
      
    //int     smask;      /**< Mask specifying types of packages from which data have been obtained */

    //nmeaTIME utc;       /**< UTC of position */ 

    //int     sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
    //int     fix;        /**< Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */

    //double  PDOP;       /**< Position Dilution Of Precision */
    //double  HDOP;       /**< Horizontal Dilution Of Precision */
    //double  VDOP;       /**< Vertical Dilution Of Precision */

    //double  lat;        /**< Latitude in NDEG - +/-[degree][min].[sec/60] */
    //double  lon;        /**< Longitude in NDEG - +/-[degree][min].[sec/60] */
    //double  elv;        /**< Antenna altitude above/below mean sea level (geoid) in meters */
    //double  speed;      /**< Speed over the ground in kilometers/hour */
    //double  direction;  /**< Track angle in degrees True */
    //double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */

    //nmeaSATINFO satinfo; /**< Satellites information */
      

      
    /*it++;
      
    if(it > 6)
	done = 1;
      
    select_timeout.tv_sec = 1;
    select_timeout.tv_usec = 0;*/
  }
 
    
  nmea_parser_destroy(&parser);

  return 0;
}
