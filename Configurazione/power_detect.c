#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

volatile int STOP = 0;

int main()
{
  fd_set rset;
  int status = 0;

  int rs232_device, res, power_status;
  char buf[255];

  rs232_device = open("/dev/input/event0", O_RDWR);

  if(rs232_device < 0)
    return 0;

  while(STOP == 0)
  {
    FD_ZERO(&rset);

    if(rs232_device > 0)
      FD_SET(rs232_device, &rset);

    status = select(rs232_device + 1, &rset, NULL, NULL, NULL);

    if(status > 0)
    {
      if(FD_ISSET(rs232_device, &rset))
      { 
        res = read(rs232_device, buf, 255);

        if(res < 0)
          perror("read");

        power_status = buf[12];

        if(power_status)
        {
          printf("Turning off the computer. . . \n");
          system("shutdown -P now");

          STOP = 1;
        }
      }
    }
  }

  return 0;
}


