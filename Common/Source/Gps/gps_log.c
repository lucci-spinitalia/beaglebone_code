#include <sys/types.h> 
#include <sys/ioctl.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h>

#include <sys/time.h> 
#include <time.h> 
#include <errno.h>

#include <locale.h>

#include <string.h>

#include <signal.h>

#include <termios.h>

#include "rs232.h"

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int gpio_generic_set_value(char *path, int value);
void gps_compare_log(double latitude, double longitude, double latitude_odometry, double longitude_odometry,
                     double velocity, double pdop, double hdop, double vdop, int sat_inview, int sat_used, double direction, 
                     double direction_bussola, long time_us);

void gps_text_log(char *string);

/* Gps */
int gps_device_airmar = -1;
int gps_device_custom = -1;

unsigned char show_gps_state_flag = 0;

void signal_handler(int signum)
{
  // Garbage collection
  printf("Terminating program...\n");
  
  close(gps_device_airmar);
  close(gps_device_custom);
  
  exit(signum);
}

int main(int argc, char **argv) 
{
  int argc_count;
  if(argc > 1)
  {
    for(argc_count = 1; argc_count < argc; argc_count++)
    {
      if(strcmp(argv[argc_count], "--show-gps") == 0)
        show_gps_state_flag = 1;
      else
      {
        printf("\t--show-gps\tprint pgs state and current gps coord\n");
        exit(0);
      }
    }
  }
  
  // Gps rs232 device
  char gps_device_buffer[RS232_BUFFER_SIZE];
   
  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr; // structure for select()
  
  printf("Initializing. . .\n");

  /* Peripheral initialization */
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Select UART2_TX and set it as output
  gpio_generic_set_value("/sys/kernel/debug/omap_mux/spi0_d0", 11);
  
  // Select UART1_RX and set it as input pulled up
  gpio_generic_set_value("/sys/kernel/debug/omap_mux/spi0_sclk", 39);
  
  gps_device_airmar = com_open("/dev/ttyO2", 4800, 'N', 8, 1);
  
  if(gps_device_airmar < 0)
    perror("com_open");
  else
    printf("AirMar Device\t[opened]\n");
  
  printf("Run main program. . .\n");
 
  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
      
    if(gps_device_airmar > 0)
    {
      FD_SET(gps_device_airmar, &rd);
      nfds = max(nfds, gps_device_airmar);
    }

    select_result = select(nfds + 1, &rd, &wr, NULL, NULL);

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

    /* Manage gps */
    if(gps_device_airmar > 0)
    {
      if(FD_ISSET(gps_device_airmar, &rd))
      {
        bytes_read = rs232_read(gps_device_airmar);
        if((bytes_read > 0) || ((bytes_read < 0) && rs232_buffer_rx_full))
        {
          bytes_read = rs232_unload_rx_filtered(gps_device_buffer, '\n');

          if(bytes_read > 0)
          {
            gps_device_buffer[bytes_read] = '\0';
            
            if(show_gps_state_flag)
              printf("%s", gps_device_buffer);
            gps_text_log(gps_device_buffer);
          }
        }
      }
    }
  }  // end while(!= done)

  return 0;
}

void gps_text_log(char *string)
{
  FILE *file = NULL;

  // Init Log File
  file = fopen("gps_log.txt", "a");
  
  if(!file)
  {
    perror("logfile fopen:");
    return;
  }
  
  fprintf(file, "%s", string);

  fclose(file);
}

int gpio_generic_set_value(char *path, int value)
{
  FILE *file = NULL;

  file = fopen(path, "a");
      
  if(file == NULL)
    return -1;
    
  fprintf(file, "%i", value);

  fclose(file);
  return 1;
}
