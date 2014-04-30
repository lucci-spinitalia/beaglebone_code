#include <sys/socket.h>
#include <linux/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "arm_udp.h"

#define ARM_BUFFER_SIZE 1024

/* Global variable */
struct arm_frame arm_buffer_tx[ARM_BUFFER_SIZE];
unsigned int arm_buffer_tx_ptr_wr;
unsigned int arm_buffer_tx_ptr_rd;
unsigned char arm_buffer_tx_empty;
unsigned char arm_buffer_tx_full;
unsigned char arm_buffer_tx_overrun;
unsigned int arm_buffer_tx_data_count;

struct arm_frame arm_buffer_rx[ARM_BUFFER_SIZE];
unsigned int arm_buffer_rx_ptr_wr;
unsigned int arm_buffer_rx_ptr_rd;
unsigned char arm_buffer_rx_empty;
unsigned char arm_buffer_rx_full;
unsigned char arm_buffer_rx_overrun;
unsigned int arm_buffer_rx_data_count;

volatile unsigned char arm_net_down = 1;

int arm_open_server(int *socket, struct sockaddr_in *address, int src_port)
{
  // Init circular buffer
  arm_buffer_tx_ptr_wr = 0;
  arm_buffer_tx_ptr_rd = 0;
  arm_buffer_tx_empty = 1;
  arm_buffer_tx_full = 0;
  arm_buffer_tx_overrun = 0;
  arm_buffer_tx_data_count = 0;

  arm_buffer_rx_ptr_wr = 0;
  arm_buffer_rx_ptr_rd = 0;
  arm_buffer_rx_empty = 1;
  arm_buffer_rx_full = 0;
  arm_buffer_rx_overrun = 0;
  arm_buffer_rx_data_count = 0;
  
  if(init_server(socket, address, src_port) == -1)
	  return 0;
  else
  {
    arm_net_down = 0;
    return 1;
  }
}

int arm_open(int *socket, struct sockaddr_in *address, char *ip_address, int dest_port)
{
  // Init circular buffer
  arm_buffer_tx_ptr_wr = 0;
  arm_buffer_tx_ptr_rd = 0;
  arm_buffer_tx_empty = 1;
  arm_buffer_tx_full = 0;
  arm_buffer_tx_overrun = 0;
  arm_buffer_tx_data_count = 0;

  arm_buffer_rx_ptr_wr = 0;
  arm_buffer_rx_ptr_rd = 0;
  arm_buffer_rx_empty = 1;
  arm_buffer_rx_full = 0;
  arm_buffer_rx_overrun = 0;
  arm_buffer_rx_data_count = 0;
  
  if(init_client(socket, address, ip_address, dest_port) == -1)
	  return 0;
  else
  {
    arm_net_down = 0;
    return 1;
  }
}

int arm_buffer_tx_get_space(void)
{
  return (ARM_BUFFER_SIZE - arm_buffer_tx_data_count);
}

int arm_buffer_rx_get_space(void)
{
  return (ARM_BUFFER_SIZE - arm_buffer_rx_data_count);
}

int arm_load_tx(struct arm_frame data)
{
  if(arm_buffer_tx_full)
  {
    arm_buffer_tx_overrun = 1;
    return -1;
  }

  if(arm_net_down)
    return 0;

  memcpy(&arm_buffer_tx[arm_buffer_tx_ptr_wr], &data, sizeof(struct arm_frame));
  arm_crc_compute_byte_buffer_crc((__u8 *)&arm_buffer_tx[arm_buffer_tx_ptr_wr], sizeof(arm_buffer_tx[arm_buffer_tx_ptr_wr].arm_command));

  arm_buffer_tx_data_count++;
  arm_buffer_tx_ptr_wr++;

  if(arm_buffer_tx_ptr_wr == ARM_BUFFER_SIZE)
    arm_buffer_tx_ptr_wr = 0;

  arm_buffer_tx_empty = 0;

  if(arm_buffer_tx_data_count == ARM_BUFFER_SIZE)
    arm_buffer_tx_full = 1;

  return 1;
}

int arm_unload_rx(struct arm_frame *data)
{
  if(arm_buffer_rx_empty)
    return 0;

  arm_buffer_rx_full = 0;
 
  memcpy(data, arm_buffer_rx[arm_buffer_rx_ptr_rd].arm_command, sizeof(struct arm_frame));

  arm_buffer_rx_data_count--;
  
  if(arm_buffer_rx_data_count == 0)
    arm_buffer_rx_empty = 1;

  arm_buffer_rx_ptr_rd++;

  if(arm_buffer_rx_ptr_rd == ARM_BUFFER_SIZE)
    arm_buffer_rx_ptr_rd = 0;

  return 1;
}

int arm_send(int device, struct sockaddr_in *dest_address)
{
  int bytes_sent = 0;

  if(device > 0)
  {
    if(arm_buffer_tx_empty)
      return 0;

    bytes_sent = sendto(device, &arm_buffer_tx[arm_buffer_tx_ptr_rd], sizeof(struct arm_frame), 0, (struct sockaddr *)dest_address, sizeof(*dest_address));
    //bytes_sent = write(rs232_device, &rs232_buffer_tx[rs232_buffer_tx_ptr_rd], length_to_write);

    if(bytes_sent > 0)
    {
      arm_net_down = 0;
  
      arm_buffer_tx_full = 0;
      arm_buffer_tx_data_count--;
      arm_buffer_tx_ptr_rd ++;

      if(arm_buffer_tx_ptr_rd == ARM_BUFFER_SIZE)
        arm_buffer_tx_ptr_rd = 0;
      else if(arm_buffer_tx_ptr_rd > ARM_BUFFER_SIZE)
        printf("Circular buffer critical error\n");
    }
    else
      arm_net_down = 1;
  }

  if(arm_buffer_tx_data_count == 0)
    arm_buffer_tx_empty = 1;

  return bytes_sent;
}

int arm_read(int device, struct sockaddr_in *arm_client_address_dest)
{
  int bytes_read = -1;
  socklen_t arm_client_address_dest_len = sizeof(*arm_client_address_dest);

  if(arm_buffer_rx_full)
  {
    arm_buffer_rx_overrun = 1;
    return -1;
  }

  if(device > 0)
  {
    bytes_read = recvfrom(device, arm_buffer_rx[arm_buffer_rx_ptr_wr].arm_command, sizeof(struct arm_frame), 0, (struct sockaddr *)arm_client_address_dest, &arm_client_address_dest_len);

    if(bytes_read > 0)
    {
      arm_buffer_rx_empty = 0;
      arm_buffer_rx_data_count ++;
 
      if(arm_buffer_rx_data_count == ARM_BUFFER_SIZE)
        arm_buffer_rx_full = 1;

      arm_buffer_rx_ptr_wr ++;

      if(arm_buffer_rx_ptr_wr == ARM_BUFFER_SIZE)
        arm_buffer_rx_ptr_wr = 0;
    }
  }

  return bytes_read;
}

__u16 arm_compute_crc_table_value(__u16 the_byte)
{
  __u16 j;
  __u16 k;
  __u16 table_value;

  k = the_byte;

  table_value = 0;

  for(j = 0; j < 8; j++)
  {
    if(((table_value ^ k) & 0x0001) == 0x0001)
      table_value = (table_value >> 1) ^ CRC_ADJUSTMENT;
    else
      table_value >>= 1;

    k >>= 1;
  }

  return (table_value);
}

__u16 arm_crc_calculate_crc_16(__u16 old_crc, __u8 new_byte)
{
  __u16 temp;
  __u16 new_crc;

  temp = old_crc ^ new_byte;

  new_crc = (old_crc >> 8) ^ crc_table[temp & 0x00FF];

  return (new_crc);
}

void arm_crc_initialize(void)
{
  __u16 byte;

  for(byte = 0; byte < CRC_TABLE_SIZE; byte++)
    crc_table[byte] = arm_compute_crc_table_value(byte);
}

void arm_crc_compute_byte_buffer_crc(__u8 *byte_buffer, __u32 bytes_in_buffer)
{
  __u32 count;
  __u32 crc_index = bytes_in_buffer - 2;
  __u16 new_crc = INITIAL_CRC;

  for(count = 0; count < crc_index; count++)
  {
    new_crc = arm_crc_calculate_crc_16(new_crc, byte_buffer[count]);
  }

  byte_buffer[crc_index] = (__u8)((new_crc & 0xFF00) >> 8);
  byte_buffer[crc_index+1] = (__u8)(new_crc & 0x00FF);
}

unsigned char arm_crc_byte_buffer_crc_is_valid(__u8 *byte_buffer, __u32 bytes_in_buffer)
{
  __u32 count;
  __u32 crc_index = bytes_in_buffer -2;
  __u16 new_crc = INITIAL_CRC;
  __u16 received_crc = INITIAL_CRC;
  unsigned char success;

  for(count = 0; count < crc_index; count++)
  {
    new_crc = arm_crc_calculate_crc_16(new_crc, byte_buffer[count]);
  }

  received_crc = ((byte_buffer[crc_index] << 8) & 0xFF00);
  received_crc |= (byte_buffer[crc_index + 1] & 0x00FF);

  //printf("new_crc: %x\treceived_crc: %x\n", new_crc, received_crc);
  if(received_crc == new_crc)
    success = 1;
  else
  {
    printf("Crc i %i but should be %i\n", received_crc, new_crc);
    success = 0;
  }

  return success;
}
