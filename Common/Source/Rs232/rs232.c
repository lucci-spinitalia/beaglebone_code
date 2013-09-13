#include <sys/types.h>
#include <linux/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>

#include "rs232.h"

#define MODEMDEVICE "/dev/ttyO2"
#define _POSIX_SOURCE 1
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;

/* Prototype */
int com_open(char *, __u32, char, int, int);

void flush_device_input(int *);
void flush_device_output(int *);

int rs232_load_tx(unsigned char *data, unsigned int data_length);
int rs232_unload_rx(unsigned char *data);
int rs232_write(int rs232_device);
int rs232_read(int rs232_device);
int rs232_buffer_tx_get_space(void);
int rs232_buffer_rx_get_space(void);

char rs232_buffer_tx[RS232_BUFFER_SIZE];
unsigned int rs232_buffer_tx_ptr_wr;
unsigned int rs232_buffer_tx_ptr_rd;
unsigned char rs232_buffer_tx_empty;
unsigned char rs232_buffer_tx_full;
unsigned char rs232_buffer_tx_overrun;
unsigned int rs232_buffer_tx_data_count;  

char rs232_buffer_rx[RS232_BUFFER_SIZE];
unsigned int rs232_buffer_rx_ptr_wr;
unsigned int rs232_buffer_rx_ptr_rd;
unsigned char rs232_buffer_rx_empty;
unsigned char rs232_buffer_rx_full;
unsigned char rs232_buffer_rx_overrun;
unsigned int rs232_buffer_rx_data_count; 

int com_open(char *device_name, __u32 rate, char parity,
             int data_bits, int stop_bits)
{
  int fd;
  int local_rate = 0;
  int local_databits = 0;
  int local_stopbits = 0;
  int local_parity = 0;
  char upper_parity;
  struct termios oldtio, newtio;

  // Init circular buffer
  rs232_buffer_tx_ptr_wr = 0;
  rs232_buffer_tx_ptr_rd = 0;
  rs232_buffer_tx_empty = 1;
  rs232_buffer_tx_full = 0;
  rs232_buffer_tx_overrun = 0;
  rs232_buffer_tx_data_count = 0;

  rs232_buffer_rx_ptr_wr = 0;
  rs232_buffer_rx_ptr_rd = 0;
  rs232_buffer_rx_empty = 1;
  rs232_buffer_rx_full = 0;
  rs232_buffer_rx_overrun = 0;
  rs232_buffer_rx_data_count = 0;
  
  fd = open(device_name, O_RDWR | O_NOCTTY);

  if(fd < 0)
    return fd;

  // Check fo valid values
  upper_parity = toupper(parity);

  if(((data_bits == 5) || (data_bits == 6) || (data_bits == 7) || (data_bits == 8)) &&
     ((stop_bits == 2) || (stop_bits == 1)) &&
     ((upper_parity == 'N') || (upper_parity == 'O') || (upper_parity == 'E')) &&
     ((rate == 50) || (rate == 75) || (rate == 110) || (rate == 134) || (rate == 150) ||
      (rate == 200) || (rate == 300) || (rate == 600) || (rate == 1200) || (rate == 2400) || 
      (rate == 4800) || (rate == 9600) || (rate == 19200) || (rate == 38400) ||
      (rate == 57600) || (rate == 115200)))
  {
    switch(rate)
    {
      case 50: local_rate = B50; break;
      case 75: local_rate = B75; break;
      case 110: local_rate = B110; break;
      case 134: local_rate = B134; break;
      case 150: local_rate = B150; break;
      case 200: local_rate = B200; break;
      case 300: local_rate = B300; break;
      case 600: local_rate = B600; break;
      case 1200: local_rate = B1200; break;
      case 1800: local_rate = B1800; break;
      case 2400: local_rate = B2400; break;
      case 4800: local_rate = B4800; break;
      case 9600: local_rate = B9600; break;
      case 19200: local_rate = B19200; break;
      case 38400: local_rate = B38400; break;
      case 57600: local_rate = B57600; break;
      case 115200: local_rate = B115200; break;
    }

    switch(data_bits)
    {
      case 5: local_databits = CS5; break;
      case 6: local_databits = CS6; break;
      case 7: local_databits = CS7; break;
      case 8: local_databits = CS8; break;
    }

    if(stop_bits == 2)
      local_stopbits = CSTOPB;
    else
      local_stopbits = 0;

    switch(upper_parity)
    {
      case 'E': local_parity = PARENB; break;
      case 'O': local_parity |= PARODD; break;
    } 


    tcgetattr(fd, &oldtio); //save current port settings
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = local_rate | local_databits | local_stopbits | local_parity | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; //inter-character timer unused
    newtio.c_cc[VMIN] = 1; //blocking read until 5 chars received

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    return fd;
  }
  else
  {
    close(fd);
    return -1;
  }
}

void flush_device_input(int *device)
{
  tcflush(*device, TCIFLUSH);
}

void flush_device_output(int *device)
{
  tcflush(*device, TCOFLUSH);
}


int rs232_buffer_tx_get_space(void)
{
  return (RS232_BUFFER_SIZE - rs232_buffer_tx_data_count);
}

int rs232_buffer_rx_get_space(void)
{
  return (RS232_BUFFER_SIZE - rs232_buffer_rx_data_count);
}

int rs232_load_tx(unsigned char *data, unsigned int data_length)
{
  int i;

  if(rs232_buffer_tx_full)
  {
    rs232_buffer_tx_overrun = 1;
    return -1;
  }

  if(rs232_buffer_tx_get_space() < data_length)
  {
    rs232_buffer_tx_full = 1;
    rs232_buffer_tx_overrun = 1;
    return -1;
  }

  for(i = 0; i < data_length; i++)
  {
    rs232_buffer_tx[rs232_buffer_tx_ptr_wr] = data[i];

    rs232_buffer_tx_data_count ++;
    rs232_buffer_tx_ptr_wr ++;

    if(rs232_buffer_tx_ptr_wr == RS232_BUFFER_SIZE)
      rs232_buffer_tx_ptr_wr = 0;
  }

  rs232_buffer_tx_empty = 0;

  if(rs232_buffer_tx_data_count == RS232_BUFFER_SIZE)
    rs232_buffer_tx_full = 1;

  return 1;
}

int rs232_unload_rx(unsigned char *data)
{
  int length_to_write = 0;

  if(rs232_buffer_rx_empty)
    return 0;

  rs232_buffer_rx_full = 0;
 
  if(rs232_buffer_rx_ptr_rd < rs232_buffer_rx_ptr_wr)
    length_to_write = (rs232_buffer_rx_ptr_wr - rs232_buffer_rx_ptr_rd);
  else
    length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_rd);

  memcpy(data, &rs232_buffer_rx[rs232_buffer_rx_ptr_rd], length_to_write);
	
  rs232_buffer_rx_data_count -= length_to_write;

  if(rs232_buffer_rx_data_count == 0)
    rs232_buffer_rx_empty = 1;

  rs232_buffer_rx_ptr_rd += length_to_write;

  if(rs232_buffer_rx_ptr_rd == RS232_BUFFER_SIZE)
    rs232_buffer_rx_ptr_rd = 0;

//  printf("\nempty: %i, data_count: %i\n", rs232_buffer_rx_empty,rs232_buffer_rx_data_count);
//  printf("full: %i, rd pointer: %i\n", rs232_buffer_rx_full, rs232_buffer_rx_ptr_rd);
//  printf("\n");

  return length_to_write;
}

int rs232_read(int rs232_device)
{
  int bytes_read = -1;

  if(rs232_buffer_rx_full)
  {
    rs232_buffer_rx_overrun = 1;
    return -1;
  }

  if(rs232_device > 0)
  {
    // I want to read as many bytes as possible, but I have to manage the
    // circular buffer. So I can read only the data before restart the
    // buffer.
    bytes_read = read(rs232_device, &rs232_buffer_rx[rs232_buffer_rx_ptr_wr], (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr));

    if(bytes_read > 0)
    {
      rs232_buffer_rx_empty = 0;
      rs232_buffer_rx_data_count += bytes_read;
 
      if(rs232_buffer_rx_data_count == RS232_BUFFER_SIZE)
        rs232_buffer_rx_full = 1;

      rs232_buffer_rx_ptr_wr += bytes_read;

      if(rs232_buffer_rx_ptr_wr == RS232_BUFFER_SIZE)
        rs232_buffer_rx_ptr_wr = 0;
    }
  }

  return bytes_read;
}

int rs232_write(int rs232_device)
{
  int bytes_sent = 0;
  int length_to_write = 0;

  if(rs232_device > 0)
  {
    if(rs232_buffer_tx_empty)
      return 0;

    // I want to send as many bytes as possible, but I have to manage the
    // circular buffer. So I can send only the data before restart the
    // buffer.
    if(rs232_buffer_tx_ptr_rd < rs232_buffer_tx_ptr_wr)
      length_to_write = (rs232_buffer_tx_ptr_wr - rs232_buffer_tx_ptr_rd);
    else
      length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_tx_ptr_rd);

    if(length_to_write > RS232_MAX_TX_LENGTH)
      length_to_write = RS232_MAX_TX_LENGTH;

    bytes_sent = write(rs232_device, &rs232_buffer_tx[rs232_buffer_tx_ptr_rd], length_to_write);

    if(bytes_sent > 0)
    {
      //printf("bytes_sent: %i\n", bytes_sent);
      /*for(i = 0; i < bytes_sent; i++)
        printf("[%x]", rs232_buffer_tx[rs232_buffer_tx_ptr_rd + i]);
      printf("\n");*/ 
      rs232_buffer_tx_full = 0;
      rs232_buffer_tx_data_count -= bytes_sent;
      rs232_buffer_tx_ptr_rd += bytes_sent;

      if(rs232_buffer_tx_ptr_rd == RS232_BUFFER_SIZE)
        rs232_buffer_tx_ptr_rd = 0;
      else if(rs232_buffer_tx_ptr_rd > RS232_BUFFER_SIZE)
        printf("Circular buffer critical error\n");
    }
  }

  if(rs232_buffer_tx_data_count == 0)
    rs232_buffer_tx_empty = 1;

  return bytes_sent;
}

int rs232_search_in_buffer(char *keyword)
{
  if(strstr(rs232_buffer_rx, keyword) != NULL)
    return 1;
  else
    return 0;
}

int rs232_check_last_char(char keyword)
{
  if(rs232_buffer_rx[rs232_buffer_rx_ptr_wr - 1] == keyword)
    return 1;
  else
    return 0;
}
/******************* Example without using circular buffer***********************/
/*int main()
{
  fd_set rset;
  int status = 0;

  int rs232_device, res;
  char buf[255];

  rs232_device = com_open(MODEMDEVICE, 19200, 'N', 8, 1);

  while(STOP == FALSE)
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
        buf[res] = 0;
        printf(":%s:%d\n", buf, res);
        write(rs232_device, buf, res);
  
        if(buf[0] == 'z')
          STOP = TRUE;
      }
    }
  }

//  tcsetattr(rs232_device, TCSANOW, &oldtio); 

  return 0;
}
*/

/******************* Example using circular buffer ***********************/
/*int main()
{
  fd_set rset, wset;
  int status = 0;

  int rs232_device, res;
  char buf[255];
  int bytes_read;

  rs232_device = com_open(MODEMDEVICE, 19200, 'N', 8, 1);

  while(STOP == FALSE)
  {
    FD_ZERO(&rset);
	FD_ZERO(&wset);

    if(rs232_device > 0)
	{
      FD_SET(rs232_device, &rset);
	  
	  if(rs232_buffer_tx_empty == 0)
	    FD_SET(rs232_device, &wset);
	}
    status = select(rs232_device + 1, &rset, &wset, NULL, NULL);

    if(status > 0)
    {
      if(FD_ISSET(rs232_device, &rset))
      { 
        res = rs232_read(rs232_device);
		
	if(res < 0)
	  perror("rs232_read");
      }
	  
	  if(FD_ISSET(rs232_device, &wset))
      { 
        res = rs232_write(rs232_device);
		
	if(res < 0)
	  perror("rs232_read");
      }
    }
	
    if(rs232_buffer_rx_empty == 0)
    {
      bytes_read = rs232_unload_rx(buf);
      buf[bytes_read] = 0;
  
      printf(":%s:%d\n", buf, res);
      rs232_load_tx(buf, bytes_read);
	  
      if(buf[0] == 'z')
        STOP = TRUE;
    }
  }

//  tcsetattr(rs232_device, TCSANOW, &oldtio); 

  return 0;
}*/
