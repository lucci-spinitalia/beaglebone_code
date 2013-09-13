#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>
#include <linux/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>

#include "socket_udp.h"
#include "pc_interface_udp.h"

#define PC_INTERFACE_BUFFER_SIZE 1024

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))


/* Global variable */
struct pc_interface_udp_frame pc_interface_buffer_tx[PC_INTERFACE_BUFFER_SIZE];
unsigned int pc_interface_buffer_tx_ptr_wr;
unsigned int pc_interface_buffer_tx_ptr_rd;
unsigned char pc_interface_buffer_tx_empty;
unsigned char pc_interface_buffer_tx_full;
unsigned char pc_interface_buffer_tx_overrun;
unsigned int pc_interface_buffer_tx_data_count;

struct pc_interface_udp_frame pc_interface_buffer_rx[PC_INTERFACE_BUFFER_SIZE];
unsigned int pc_interface_buffer_rx_ptr_wr;
unsigned int pc_interface_buffer_rx_ptr_rd;
unsigned char pc_interface_buffer_rx_empty;
unsigned char pc_interface_buffer_rx_full;
unsigned char pc_interface_buffer_rx_overrun;
unsigned int pc_interface_buffer_rx_data_count;

int pc_interface_connect(int *socket, struct sockaddr_in *address, int src_port, char *ip_address, int dest_port)
{
  // Init circular buffer
  pc_interface_buffer_tx_ptr_wr = 0;
  pc_interface_buffer_tx_ptr_rd = 0;
  pc_interface_buffer_tx_empty = 1;
  pc_interface_buffer_tx_full = 0;
  pc_interface_buffer_tx_overrun = 0;
  pc_interface_buffer_tx_data_count = 0;
  
  pc_interface_buffer_rx_ptr_wr = 0;
  pc_interface_buffer_rx_ptr_rd = 0;
  pc_interface_buffer_rx_empty = 1;
  pc_interface_buffer_rx_full = 0;
  pc_interface_buffer_rx_overrun = 0;
  pc_interface_buffer_rx_data_count = 0;

  if(init_client2(socket, address, src_port, ip_address, dest_port) == -1)
	return 0;
  else
    return 1;
}

int pc_interface_buffer_tx_get_space(void)
{
  return (PC_INTERFACE_BUFFER_SIZE - pc_interface_buffer_tx_data_count);
}

int pc_interface_buffer_rx_get_space(void)
{
  return (PC_INTERFACE_BUFFER_SIZE - pc_interface_buffer_rx_data_count);
}

int pc_interface_load_tx(struct pc_interface_udp_frame data)
{
  if(pc_interface_buffer_tx_full)
  {
    pc_interface_buffer_tx_overrun = 1;
    return -1;
  }

  memcpy(&pc_interface_buffer_tx[pc_interface_buffer_tx_ptr_wr], &data, sizeof(struct pc_interface_udp_frame));
  
  pc_interface_buffer_tx_data_count++;
  pc_interface_buffer_tx_ptr_wr++;

  if(pc_interface_buffer_tx_ptr_wr == PC_INTERFACE_BUFFER_SIZE)
    pc_interface_buffer_tx_ptr_wr = 0;

  pc_interface_buffer_tx_empty = 0;

  if(pc_interface_buffer_tx_data_count == PC_INTERFACE_BUFFER_SIZE)
    pc_interface_buffer_tx_full = 1;

  //printf("Tx ptr: %i, Data count: %i, Empty: %i, Full: %i\n", segway_buffer_tx_ptr_wr, segway_buffer_tx_data_count, segway_buffer_tx_empty, segway_buffer_tx_full);
  return 1;
}

int pc_interface_unload_rx(struct pc_interface_udp_frame *data)
{
  if(pc_interface_buffer_rx_empty)
    return 0;


  pc_interface_buffer_rx_full = 0;
 
  memcpy(data, &pc_interface_buffer_rx[pc_interface_buffer_rx_ptr_rd], sizeof(struct pc_interface_udp_frame));

  pc_interface_buffer_rx_data_count--;
  
  if(pc_interface_buffer_rx_data_count == 0)
    pc_interface_buffer_rx_empty = 1;

  pc_interface_buffer_rx_ptr_rd ++;

  if(pc_interface_buffer_rx_ptr_rd == PC_INTERFACE_BUFFER_SIZE)
    pc_interface_buffer_rx_ptr_rd = 0;

//  printf("\nempty: %i, data_count: %i\n", rs232_buffer_rx_empty,rs232_buffer_rx_data_count);
//  printf("full: %i, rd pointer: %i\n", rs232_buffer_rx_full, rs232_buffer_rx_ptr_rd);
//  printf("\n");

  return 1;
}

int pc_interface_send(int device, struct sockaddr_in *dest_address)
{
  int bytes_sent = 0;

  if(device > 0)
  {
    if(pc_interface_buffer_tx_empty)
      return 0;

    bytes_sent = sendto(device, &pc_interface_buffer_tx[pc_interface_buffer_tx_ptr_rd], sizeof(pc_interface_buffer_tx[pc_interface_buffer_tx_ptr_rd]), 0, (struct sockaddr *)dest_address, sizeof(*dest_address));

    if(bytes_sent > 0)
    {
      pc_interface_buffer_tx_full = 0;
      pc_interface_buffer_tx_data_count--;
      pc_interface_buffer_tx_ptr_rd++;

      if(pc_interface_buffer_tx_ptr_rd == PC_INTERFACE_BUFFER_SIZE)
        pc_interface_buffer_tx_ptr_rd = 0;
      else if(pc_interface_buffer_tx_ptr_rd > PC_INTERFACE_BUFFER_SIZE)
        printf("Circular buffer critical error\n");
    }
  }

  if(pc_interface_buffer_tx_data_count == 0)
    pc_interface_buffer_tx_empty = 1;

  return bytes_sent;
}

int pc_interface_read(int device)
{
  int bytes_read = -1;

  if(pc_interface_buffer_rx_full)
  {
    pc_interface_buffer_rx_overrun = 1;
    return -1;
  }

  if(device > 0)
  {
    // I want to read as many bytes as possible, but I have to manage the
    // circular buffer. So I can read only the data before restart the
    // buffer.
    bytes_read = recvfrom(device, &pc_interface_buffer_rx[pc_interface_buffer_rx_ptr_wr], sizeof(struct pc_interface_udp_frame), 0, NULL, NULL);

    if(bytes_read > 0)
    {
      pc_interface_buffer_rx_empty = 0;
      pc_interface_buffer_rx_data_count ++;
 
      if(pc_interface_buffer_rx_data_count == PC_INTERFACE_BUFFER_SIZE)
        pc_interface_buffer_rx_full = 1;

      pc_interface_buffer_rx_ptr_wr ++;

      if(pc_interface_buffer_rx_ptr_wr == PC_INTERFACE_BUFFER_SIZE)
        pc_interface_buffer_rx_ptr_wr = 0;
    }
  }

  return bytes_read;
}

/******************* Example using circular buffer ***********************/
/*int main()
{
  int udp_client = -1;
  struct pc_interface_udp_frame pc_interface_buffer_temp;
  struct sockaddr_in client_address;

  int i;
  int done = 0;  // for the while in main loop
  int bytes_read;  // to check how many bytes has been read
  int bytes_sent;

  int select_result = -1;  // value returned frome select()
  int nfds = 0;  // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  
  if(pc_interface_connect(&udp_client, &client_address, "127.0.0.1", 32000) == 0)
    perror("error connection");
  else
    printf("Init UDP client\t[OK]\n");
 
  printf("Run main program. . .\n");
  
  pc_interface_buffer_temp.param.id = 0x11;
  pc_interface_buffer_temp.param.length = (0x07 << 3) | (0x06);
  pc_interface_buffer_temp.param.data[0] = 0x00;
  pc_interface_buffer_temp.param.data[1] = 0x01;
  pc_interface_buffer_temp.param.data[2] = 0x02;
  pc_interface_buffer_temp.param.data[3] = 0x03;
  pc_interface_buffer_temp.param.data[4] = 0x04;
  pc_interface_buffer_temp.param.data[5] = 0x05;
  pc_interface_buffer_temp.param.data[6] = 0x06;

  pc_interface_load_tx(pc_interface_buffer_temp);
  
  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    if(udp_client > 0)
    {
      FD_SET(udp_client, &rd);
      nfds = max(nfds, udp_client);  

      if(pc_interface_buffer_tx_empty == 0)
      {
        FD_SET(udp_client, &wr);
        nfds = max(nfds, udp_client);
      }
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
	
    if(udp_client > 0)
    {
      if(FD_ISSET(udp_client, &rd))
      {
        bytes_read = pc_interface_read(udp_client);
		
        if(bytes_read < 0)
          perror("Read from rs232_device");
		else
		{
		  pc_interface_unload_rx(&pc_interface_buffer_temp);
		  printf("Message Received\n");
		  printf("Header: [%x]\nLength: [%x]\nId: [%x]\n", pc_interface_buffer_temp.param.header, 
		         pc_interface_buffer_temp.param.length >> 3,
				 ((__u32)((pc_interface_buffer_temp.param.length & 0x07) <<8) | pc_interface_buffer_temp.param.id));
				 
		  printf("Data:");
		  for(i = 0; i < pc_interface_buffer_temp.param.length >> 3; i++)
            printf("[%x]", pc_interface_buffer_temp.param.data[i]);
			
	      printf("\n");
		  
        }
      } //if(FD_ISSET(rs232_device, &rd))

      if(FD_ISSET(udp_client, &wr))
      {
        printf("Send Message. . .\n");
        bytes_sent = pc_interface_send(udp_client, &client_address);

        if(bytes_sent <= 0)
          printf("Error on pc_interface_send");

      }  //if(FD_ISSET(udp_client, &wr))
    }
  }  // end while(!= done)
  
  return 0; 
}*/