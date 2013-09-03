#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>
#include <linux/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "arm_udp.h"

#define ARM_BUFFER_SIZE 1024

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */

/* Global variable */
char arm_buffer_tx[ARM_BUFFER_SIZE];
unsigned int arm_buffer_tx_ptr_wr;
unsigned int arm_buffer_tx_ptr_rd;
unsigned char arm_buffer_tx_empty;
unsigned char arm_buffer_tx_full;
unsigned char arm_buffer_tx_overrun;
unsigned int arm_buffer_tx_data_count;

int arm_open(int *socket, struct sockaddr_in *address, char *ip_address, int port)
{
  // Init circular buffer
  arm_buffer_tx_ptr_wr = 0;
  arm_buffer_tx_ptr_rd = 0;
  arm_buffer_tx_empty = 1;
  arm_buffer_tx_full = 0;
  arm_buffer_tx_overrun = 0;
  arm_buffer_tx_data_count = 0;
	
  if(init_client(socket, address, ip_address, port) == -1)
	return 0;
  else
    return 1;
}

int arm_buffer_tx_get_space(void)
{
  return (ARM_BUFFER_SIZE - arm_buffer_tx_data_count);
}

int arm_load_tx(char *data, int data_length)
{
  int i;

  if(arm_buffer_tx_full)
  {
    arm_buffer_tx_overrun = 1;
    return -1;
  }

 if(arm_buffer_tx_get_space() < data_length)
  {
    arm_buffer_tx_full = 1;
    arm_buffer_tx_overrun = 1;
    return -1;
  }

  for(i = 0; i < data_length; i++)
  {
    arm_buffer_tx[arm_buffer_tx_ptr_wr] = data[i];

    arm_buffer_tx_data_count++;
    arm_buffer_tx_ptr_wr++;

    if(arm_buffer_tx_ptr_wr == ARM_BUFFER_SIZE)
      arm_buffer_tx_ptr_wr = 0;
  }

  arm_buffer_tx_empty = 0;

  if(arm_buffer_tx_data_count == ARM_BUFFER_SIZE)
    arm_buffer_tx_full = 1;

  return 1;
}

int arm_send(int device, struct sockaddr_in *dest_address)
{
  int bytes_sent = 0;
  int length_to_write = 0;

  if(device > 0)
  {
    if(arm_buffer_tx_empty)
      return 0;

    // I want to send as many bytes as possible, but I have to manage the
    // circular buffer. So I can send only the data before restart the
    // buffer.
    if(arm_buffer_tx_ptr_rd < arm_buffer_tx_ptr_wr)
      length_to_write = (arm_buffer_tx_ptr_wr - arm_buffer_tx_ptr_rd);
    else
      length_to_write = (ARM_BUFFER_SIZE - arm_buffer_tx_ptr_rd);

	bytes_sent = sendto(device, &arm_buffer_tx[arm_buffer_tx_ptr_rd], length_to_write, 0, (struct sockaddr *)dest_address, sizeof(*dest_address));
    //bytes_sent = write(rs232_device, &rs232_buffer_tx[rs232_buffer_tx_ptr_rd], length_to_write);

    if(bytes_sent > 0)
    {
      arm_buffer_tx_full = 0;
      arm_buffer_tx_data_count -= bytes_sent;
      arm_buffer_tx_ptr_rd += bytes_sent;

      if(arm_buffer_tx_ptr_rd == ARM_BUFFER_SIZE)
        arm_buffer_tx_ptr_rd = 0;
      else if(arm_buffer_tx_ptr_rd > ARM_BUFFER_SIZE)
        printf("Circular buffer critical error\n");
    }
  }

  if(arm_buffer_tx_data_count == 0)
    arm_buffer_tx_empty = 1;

  return bytes_sent;
}
/*
int segway_read(int socket, union segway_union *segway_status)
{
  int bytes_read;
  __u8 udfb_data[(SEGWAY_PARAM*4) + 3];

  if(socket < 0)
    return -1;

  bytes_read = recvfrom(socket, udfb_data, sizeof(udfb_data), 0, NULL, NULL);

  if(bytes_read > 0)
  {
    //int i = 0;
    segway_config_update(udfb_data, segway_status);

    //for(i = 0; i < bytes_read; i++)
    //printf("[%x]", udfb_data[i]);

    //printf("\n");
  }
  else
    return -1;

  return bytes_read;
}*/
