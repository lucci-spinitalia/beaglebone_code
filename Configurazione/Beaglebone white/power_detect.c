#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define SYS_5V 38
volatile int STOP = 0;

int main()
{
  int status = 0;
  struct timeval select_timeout;

  int fd, res;
  char buf[255];
  
  // set gpio as input pulled down
  printf("Setting gpio as input\n");
  sprintf(buf, "echo 27 > /sys/kernel/debug/omap_mux/gpmc_ad6");
  if(system(buf) < 0)
    perror("setting gpio as input");
  
  // export gpio
  printf("Export gpio\n");
  sprintf(buf, "echo %i > /sys/class/gpio/export", SYS_5V);
  if(system(buf) < 0)
    perror("export gpio");
	

  select_timeout.tv_sec = 0;
  select_timeout.tv_usec = 500000;
	
  while(STOP == 0)
  {
    status = select(1, NULL, NULL, NULL, &select_timeout);

	sprintf(buf, "/sys/class/gpio/gpio%i/value", SYS_5V);
	fd = open(buf, O_RDWR | O_NOCTTY);
	
	res = read(fd, buf, 255);
	buf[res] = 0;

    if(res < 0)
      perror("read");

    if(buf[0] == '0')
    {
      printf("Turning off the computer. . . \n");
      system("shutdown -P now");

      STOP = 1;
    }
  
	close(fd);
	
	select_timeout.tv_sec = 1;
    select_timeout.tv_usec = 0;
  }

  return 0;
}


