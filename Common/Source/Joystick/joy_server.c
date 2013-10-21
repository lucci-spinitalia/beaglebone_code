#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#include "joystick.h"

int send_command()
{
  char buff[2048];


  return 1;
}

int main()
{
  int bytes_read;
  
  struct wwvi_js_event joy_event;
  
  int select_result = -1; // value returned frome select()
  struct timeval select_timeout;
  select_timeout.tv_sec = 0;
  select_timeout.tv_usec = 100000;
  fd_set rd, wr, er; // structure for select()
  
  //keyboard = fopen(stdin, O_RDWR);
  
  /*if(keyboard < 0)
  {
    perror("open keyboard");
    return 1;
  }*/
  
  while(1)
  {
 /*   fflush(stdout);
 
    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    
    if(keyboard > 0)
      FD_SET(keyboard, &rd);
    
    select_result = select(stdin + 1, &rd, NULL, NULL, NULL);

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

    if(keyboard > 0)
    {
      if(FD_ISSET(stdin, &rd))
      {
        bytes_read = read(stdin, keyboard_buffer, sizeof(keyboard_buffer));
	
	if(bytes_read > 0)
	  printf("Read from keyboard: %c", keyboard_buffer[0]);
      }
    }
	
    select_timeout.tv_sec = 0;
    select_timeout.tv_usec = 100000;*/
    bytes_read = getc(stdin);
    
    switch(bytes_read)
    {
      case 'a':
	break;
	
      case 's':
	break;
	
      case 'd':
	break;
	
      case 'w':
	break;
	
      case 'j':
	break,
	
      case 'k':
	break;
	
      case 'u':
	break;
	
      case 'i':
	break;
	
    }
    printf("%c", bytes_read);
  }

  return 0;
}
