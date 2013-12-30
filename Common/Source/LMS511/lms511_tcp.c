#include <sys/socket.h>
#include <sys/select.h>
#include <linux/types.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>

#include "lms511_tcp.h"

#define LMS511_ADDRESS "192.168.1.104"
#define LMS511_PORT 2111
#define LMS511_BUFFER_SIZE 8192

#define LMS511_STARTLINE 0x02
#define LMS511_ENDLINE   0x03

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */

/* Global variable */
unsigned char lms511_buffer_tx[LMS511_BUFFER_SIZE];
unsigned int lms511_buffer_tx_ptr_wr;
unsigned int lms511_buffer_tx_ptr_rd;
unsigned char lms511_buffer_tx_empty;
unsigned char lms511_buffer_tx_full;
unsigned char lms511_buffer_tx_overrun;
unsigned int lms511_buffer_tx_data_count;

unsigned char lms511_buffer_rx[LMS511_BUFFER_SIZE];
unsigned int lms511_buffer_rx_ptr_wr;
unsigned int lms511_buffer_rx_ptr_rd;
unsigned char lms511_buffer_rx_empty;
unsigned char lms511_buffer_rx_full;
unsigned char lms511_buffer_rx_overrun;
unsigned int lms511_buffer_rx_data_count;

unsigned int lms511_count; // var for loop only
struct LMS511_INFO lms511_info;

float convert_to_float(__u32 value)
{
  return (*((float *)&value));
}

int lms511_open(int *socket, struct sockaddr_in *address, char *ip_address, int dest_port)
{
  // Init circular buffer
  lms511_buffer_tx_ptr_wr = 0;
  lms511_buffer_tx_ptr_rd = 0;
  lms511_buffer_tx_empty = 1;
  lms511_buffer_tx_full = 0;
  lms511_buffer_tx_overrun = 0;
  lms511_buffer_tx_data_count = 0;

  lms511_buffer_rx_ptr_wr = 0;
  lms511_buffer_rx_ptr_rd = 0;
  lms511_buffer_rx_empty = 1;
  lms511_buffer_rx_full = 0;
  lms511_buffer_rx_overrun = 0;
  lms511_buffer_rx_data_count = 0;
  
  lms511_info.state = LMS511_UNDEFINED;
  lms511_info.scaling_factor = 1;
  lms511_info.data.spot_number = 0;
  lms511_info.data.spot = NULL;
  
  if(init_tcp_client(socket, ip_address, dest_port) == -1)
	return -1;
  else
    return 1;
}

int lms511_buffer_tx_get_space(void)
{
  return (LMS511_BUFFER_SIZE - lms511_buffer_tx_data_count);
}

int lms511_buffer_rx_get_space(void)
{
  return (LMS511_BUFFER_SIZE - lms511_buffer_rx_data_count);
}

int lms511_load_tx(char *data)
{
  int i;
  
  if(lms511_buffer_tx_full)
  {
    lms511_buffer_tx_overrun = 1;
    return -1;
  }

  for(i = 0; i < strlen(data); i++)
  {
    lms511_buffer_tx[lms511_buffer_tx_ptr_wr] = data[i];

    lms511_buffer_tx_data_count++;
    lms511_buffer_tx_ptr_wr++;

    if(lms511_buffer_tx_ptr_wr == LMS511_BUFFER_SIZE)
      lms511_buffer_tx_ptr_wr = 0;
  }
  
  lms511_buffer_tx_empty = 0;

  if(lms511_buffer_tx_data_count == LMS511_BUFFER_SIZE)
    lms511_buffer_tx_full = 1;

  return 1;
}

// size of data must be LMS511_BUFFER_SIZE 
int lms511_unload_rx(unsigned char *data)
{
  int i;
  int length_to_write = 0;
  
  if(lms511_buffer_rx_empty)
    return 0;

  lms511_buffer_rx_full = 0;
 
  if(lms511_buffer_rx_ptr_rd < lms511_buffer_rx_ptr_wr)
    length_to_write = (lms511_buffer_rx_ptr_wr - lms511_buffer_rx_ptr_rd);
  else
    length_to_write = (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd + lms511_buffer_rx_ptr_wr);

  for(i = 0; i < length_to_write; i++)
  {
    data[i] = lms511_buffer_rx[lms511_buffer_rx_ptr_rd];

    lms511_buffer_rx_data_count--;

    lms511_buffer_rx_ptr_rd++;

    if(lms511_buffer_rx_ptr_rd == LMS511_BUFFER_SIZE)
      lms511_buffer_rx_ptr_rd = 0;
  }

  if(lms511_buffer_rx_data_count == 0)
    lms511_buffer_rx_empty = 1;
	  
  return length_to_write;
}

// size of data must be LMS511_BUFFER_SIZE 
int lms511_unload_rx_filtered(char *data, char token)
{
  int length_to_write = 0;
  char lms511_buffer_rx_temp[LMS511_BUFFER_SIZE];
  char *token_ptr;

  if(lms511_buffer_rx_empty)
  {
    printf("Buffer empty\n");
    return 0;
  }

  lms511_buffer_rx_full = 0;

  if(lms511_buffer_rx_ptr_rd < lms511_buffer_rx_ptr_wr)
  {
    // Search into the buffer for the token
    memcpy(lms511_buffer_rx_temp, &lms511_buffer_rx[lms511_buffer_rx_ptr_rd], lms511_buffer_rx_ptr_wr - lms511_buffer_rx_ptr_rd);
    lms511_buffer_rx_temp[lms511_buffer_rx_ptr_wr - lms511_buffer_rx_ptr_rd] = '\0';
  }
  else
  {
    // Search into the buffer for the token
    memcpy(lms511_buffer_rx_temp, &lms511_buffer_rx[lms511_buffer_rx_ptr_rd], LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd);
	memcpy(&lms511_buffer_rx_temp[LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd], &lms511_buffer_rx[0], lms511_buffer_rx_ptr_wr);
	
    lms511_buffer_rx_temp[LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd + lms511_buffer_rx_ptr_wr] = '\0';
  }
  
  token_ptr = strrchr(lms511_buffer_rx_temp, token);
  
  if(token_ptr == NULL)
    return 0;
	
  length_to_write = (token_ptr - lms511_buffer_rx_temp);
  
  memcpy(data, lms511_buffer_rx_temp, length_to_write);
  
  if(lms511_buffer_rx_ptr_rd < lms511_buffer_rx_ptr_wr)
  {
    lms511_buffer_rx_ptr_rd += length_to_write;
	
    if(lms511_buffer_rx_ptr_rd == LMS511_BUFFER_SIZE)
      lms511_buffer_rx_ptr_rd = 0;
  }
  else
    lms511_buffer_rx_ptr_rd = length_to_write - (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd);
  
  lms511_buffer_rx_data_count -= length_to_write;
  
  if(lms511_buffer_rx_data_count == 0)
    lms511_buffer_rx_empty = 1;
	  
  return length_to_write;
}

int lms511_send(int device, struct sockaddr_in *dest_address)
{
  int bytes_sent = 0;
  int length_to_write = 0;
  unsigned char buffer[lms511_buffer_tx_data_count];
  
  if(device > 0)
  {
    if(lms511_buffer_tx_empty)
      return 0;
	
    if(lms511_buffer_tx_ptr_rd < lms511_buffer_tx_ptr_wr)
	{
      length_to_write = (lms511_buffer_tx_ptr_wr - lms511_buffer_tx_ptr_rd);
      bytes_sent = sendto(device, &lms511_buffer_tx[lms511_buffer_tx_ptr_rd], length_to_write, 0, (struct sockaddr *)dest_address, sizeof(*dest_address));
	  
      if(bytes_sent > 0)
      {
        lms511_buffer_tx_full = 0;
        lms511_buffer_tx_data_count -= bytes_sent;
        lms511_buffer_tx_ptr_rd += bytes_sent;

        if(lms511_buffer_tx_ptr_rd == LMS511_BUFFER_SIZE)
          lms511_buffer_tx_ptr_rd = 0;
        else if(lms511_buffer_tx_ptr_rd > LMS511_BUFFER_SIZE)
          printf("Circular buffer critical error\n");
      }
	}
    else
	{
      length_to_write = (LMS511_BUFFER_SIZE - lms511_buffer_tx_ptr_rd + lms511_buffer_tx_ptr_wr);
	  memcpy(buffer, &lms511_buffer_tx[lms511_buffer_tx_ptr_rd], (LMS511_BUFFER_SIZE - lms511_buffer_tx_ptr_rd));
	  memcpy(&buffer[LMS511_BUFFER_SIZE - lms511_buffer_tx_ptr_rd], &lms511_buffer_tx[0], lms511_buffer_tx_ptr_wr);
	  
      bytes_sent = sendto(device, buffer, length_to_write, 0, (struct sockaddr *)dest_address, sizeof(*dest_address));
	  
      if(bytes_sent > 0)
      {
        lms511_buffer_tx_full = 0;
        lms511_buffer_tx_data_count -= bytes_sent;
        lms511_buffer_tx_ptr_rd = bytes_sent - (LMS511_BUFFER_SIZE - lms511_buffer_tx_ptr_rd);

        if(lms511_buffer_tx_ptr_rd > LMS511_BUFFER_SIZE)
          printf("Circular buffer critical error\n");
      }
    }
  }
  
  if(lms511_buffer_tx_data_count == 0)
    lms511_buffer_tx_empty = 1;

  return bytes_sent;
}

int lms511_read(int device)
{
  int bytes_read = -1;
  char buffer[lms511_buffer_rx_get_space()];

  if(lms511_buffer_rx_full)
  {
    lms511_buffer_rx_overrun = 1;
    return -1;
  }

  if(device > 0)
  {
    bytes_read = recvfrom(device, buffer, lms511_buffer_rx_get_space(), 0, NULL, NULL);
	
    if(bytes_read > 0)
    {
      lms511_buffer_rx_empty = 0;
      lms511_buffer_rx_data_count += bytes_read;
 
      if(lms511_buffer_rx_data_count == LMS511_BUFFER_SIZE)
        lms511_buffer_rx_full = 1;

      if((lms511_buffer_rx_ptr_wr + bytes_read) <= LMS511_BUFFER_SIZE)
	  {
	    memcpy(&lms511_buffer_rx[lms511_buffer_rx_ptr_wr], buffer, bytes_read);
		lms511_buffer_rx_ptr_wr += bytes_read;
		
		if(lms511_buffer_rx_ptr_wr == LMS511_BUFFER_SIZE)
          lms511_buffer_rx_ptr_wr = 0;
	  }
      else
      {
        memcpy(&lms511_buffer_rx[lms511_buffer_rx_ptr_wr], buffer, (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_wr));
        memcpy(&lms511_buffer_rx[0], &buffer[(LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_wr)], (bytes_read - (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_wr)));

		lms511_buffer_rx_ptr_wr = (bytes_read - (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_wr));
      }
    }
  }

  return bytes_read;
}

int lms511_send_command(char *command)
{
  int bytes_sent;
  char buffer[256]; 
  
  sprintf(buffer, "%c%s%c", 0x02, command, LMS511_ENDLINE);
  
  bytes_sent = lms511_load_tx(buffer);
  
  return bytes_sent;
}

int lms511_init()
{
  lms511_config_request();
  return 1;
}

int lms511_parse(int socket_lms511)
{
  int bytes_read; // to check how many bytes has been read
  char lms511_buffer[LMS511_BUFFER_SIZE];
  char lms511_message_buffer[2048];
  char *lms511_message_token;
  char *lms511_command_token;
  char *lms511_value_token;
  char lms511_value_count;
  char lms511_endline[2];
  
  if(socket_lms511 < 0)
    return -1;

  lms511_endline[0] = LMS511_ENDLINE;
  lms511_endline[1] = '\0';
	
  bytes_read = lms511_read(socket_lms511);
		
  if(bytes_read > 0)
  {
    bytes_read = lms511_unload_rx_filtered(lms511_buffer, lms511_endline[0]);

    if(bytes_read > 0)
    {
      lms511_buffer[bytes_read] = '\0';

      lms511_message_token = strtok(lms511_buffer, lms511_endline);

      while(lms511_message_token != NULL)
      {
        if(lms511_message_token[0] == LMS511_STARTLINE)
        {
          /* searching for sRA STlms message */
          strcpy(lms511_message_buffer, lms511_message_token);
          lms511_command_token = strstr(lms511_message_buffer, "sRA STlms ");
				
          if(lms511_command_token != NULL)
          {
            // skip text
            lms511_value_token = lms511_command_token + sizeof("sRA STlms");
				  
            lms511_info.state = atoi(lms511_value_token);
            //printf("Status: %d\n", lms511_info.state);
				  
            lms511_value_token += 2;
            lms511_info.temperature_range_met = atoi(lms511_value_token);
            //printf("Temperature range: %d\n", lms511_info.temperature_range_met);
          }
				
          /* searching for sRA LMDscandata message */
          strcpy(lms511_message_buffer, lms511_message_token);
          lms511_command_token = strstr(lms511_message_buffer, "sRA LMDscandata ");
				
          if(lms511_command_token != NULL)
          {
            // skip text
            lms511_command_token += sizeof("sRA LMDscandata");
				  
            //printf("Scandata: %s\n", lms511_command_token);
            lms511_value_count = 0;
				  
            while((lms511_value_token = strchr(lms511_command_token, ' ')) != NULL)
            {
              *lms511_value_token = '\0';

              switch(lms511_value_count)
              {
                case 21: //scaling factor
                  sscanf(lms511_command_token, "%lx", &lms511_info.scaling_factor);
                  //printf("Value: %f\n", convert_to_float(lms511_info.scaling_factor));
                  break;
						
                case 23: //starting angle
                  sscanf(lms511_command_token, "%lx", &lms511_info.starting_angle);
                  //printf("Value: %ld\n", lms511_info.starting_angle);
                  break;
						
                case 24: //angular step width
                  sscanf(lms511_command_token, "%x", &lms511_info.angular_step);
                  //printf("Value: %d\n", lms511_info.angular_step);
                  break;
						
                case 25: //number of data
                  sscanf(lms511_command_token, "%x", &lms511_info.spot_number);
                  //printf("Value: %d\n", lms511_info.spot_number);
						
                  if(lms511_info.data.spot == NULL)
                  {
                    lms511_info.data.spot_number = lms511_info.spot_number;
                    //printf("Allocate %d instead %d\n",sizeof(*lms511_info.data.spot) * lms511_info.data.spot_number, lms511_info.data.spot_number);
                    lms511_info.data.spot = malloc(sizeof(*lms511_info.data.spot) * lms511_info.data.spot_number);
                  }
						
                  // If I haven't enough space then allocate it
                  if(lms511_info.data.spot_number < lms511_info.spot_number)
                  {
                    //printf("Resize data\n");
                    lms511_info.data.spot_number = lms511_info.spot_number;
                    free(lms511_info.data.spot);
                    lms511_info.data.spot = malloc(lms511_info.data.spot_number);
                  }
						
                  //printf("Get data\n");
                  for(lms511_count = 0; lms511_count < lms511_info.spot_number; lms511_count++)
                  {
                    // move to the next value
                    lms511_command_token += (lms511_value_token - lms511_command_token + 1);
                    lms511_value_token = strchr(lms511_command_token, ' ');

                    if(lms511_value_token != NULL)
                    {
                      *lms511_value_token = '\0';
                      sscanf(lms511_command_token, "%x", &lms511_info.data.spot[lms511_count]);

                      lms511_info.data.spot[lms511_count] *= convert_to_float(lms511_info.scaling_factor);
                      lms511_value_count++;
                    }
                    else
                    {
                      printf("Something wrong in lms datascan\n");
                      break;
                    }
                  }
						
                  break;
              }
					
              lms511_value_count++;
              lms511_command_token += (lms511_value_token - lms511_command_token + 1);	
            }				  
          }
        }
        //sprintf(nmea_message, "%s\n", lms511_message_token);
        //nmea_parse(&parser, nmea_message, (int)strlen(nmea_message), &info);
        lms511_message_token = strtok(NULL, lms511_endline);
      }
    }
  }
  
  return bytes_read;
}

void signal_handler(int signum)
{
  // Garbage collection
  printf("Terminating program...\n");
  
  lms511_dispose();
  exit(signum);
}

int main()
{
  int socket_lms511 = -1;
  struct sockaddr_in lms511_address;
  int spot_count;

  int bytes_sent;
  int select_result = -1; // value returned from select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  
  select_timeout.tv_sec = 0;
  select_timeout.tv_usec = 250000;
  
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  
  /* Init lms511 Client */  
  if(lms511_open(&socket_lms511, &lms511_address, LMS511_ADDRESS, LMS511_PORT) == -1)
  {
    perror("init lms511 client");
	return 1;
  }
  else
  {
    lms511_init();
  }
  
  while(1)
  { 
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);
  
    if(socket_lms511 > 0)
    {
      FD_SET(socket_lms511, &rd);
      nfds = max(nfds, socket_lms511);  
 
      if(lms511_buffer_tx_empty == 0)
      {
        FD_SET(socket_lms511, &wr);
        nfds = max(nfds, socket_lms511);
      }
    }
	
	select_result = select(nfds + 1, &rd, &wr, NULL, &select_timeout);

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
	
    if(socket_lms511 > 0)
    {
      if(FD_ISSET(socket_lms511, &rd))
      {
        lms511_parse(socket_lms511);
		
        printf("\n\n\n");
		for(spot_count = 0; spot_count < lms511_info.spot_number; spot_count++)
		{
		  printf("\033[K");  // clear line for cursor right
          if(lms511_info.data.spot[spot_count] > 1000)
          {
            printf("\033[1A");
            printf("\033[K");  // clear line for cursor right							  
          }

          if(lms511_info.data.spot[spot_count] > 2000)
          {
            printf("\033[1A");
            printf("\033[K");  // clear line for cursor right
          }

          if(lms511_info.data.spot[spot_count] > 3000)
          {
            printf("\033[1A");
            printf("\033[K");  // clear line for cursor right
          }
							  
          printf("_");

          if(lms511_info.data.spot[spot_count] > 1000)
            printf("\033[1B");

          if(lms511_info.data.spot[spot_count] > 2000)
            printf("\033[1B");

          if(lms511_info.data.spot[spot_count] > 3000)
            printf("\033[1B");
		}
		
		printf("\033[3A");
        continue;
	  }
	  
      if(FD_ISSET(socket_lms511, &wr))
      {
        bytes_sent = lms511_send(socket_lms511, &lms511_address);

        if(bytes_sent <= 0)
          perror("Error on lms511_send");
  
        continue;
      }
	}
	
	
	// On timeout
	// If I'm not in measure state then login, enable measure
	// mode and logout. So request the new status
	if(lms511_info.state != LMS511_MEASURE)
	{
      lms511_login_as_auth_client();
	  lms511_start_measure();
	  lms511_logout();
	  
	  lms511_query_status();
	}
	else
	  lms511_scan_request();
	
    select_timeout.tv_sec = 0;
    select_timeout.tv_usec = 250000;
  }
  
  return 0;
}

void lms511_dispose()
{
  if(lms511_info.data.spot != NULL)
    free(lms511_info.data.spot);
}