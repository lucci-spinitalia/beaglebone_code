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
#include "segway_config.h" // To use convert_to_float function

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

int lms511_buffer_rx_bookmark;

unsigned int lms511_count; // var for loop only
struct LMS511_INFO lms511_info;

/*float convert_to_float(__u32 value)
 {
 return (*((float *)&value));
 }*/

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

int lms511_unload_rx_multifiltered(char *data, char *token, char token_number)
{
  int length_to_write = 0;
  char lms511_buffer_rx_temp[LMS511_BUFFER_SIZE];

  char *token_ptr[10];
  char *token_winner = NULL;
  char *null_character = NULL;
  int token_count = 0;


  if(lms511_buffer_rx_empty)
    return 0;

  if(token_number > 10)
    return -1;

  // it checks if bookmark arrives to the end and if it rolled up

  // if the bookmark pass the buffer limit, then it must
  // starts from the begginning
  if(lms511_buffer_rx_bookmark >= LMS511_BUFFER_SIZE)
    lms511_buffer_rx_bookmark = lms511_buffer_rx_bookmark - LMS511_BUFFER_SIZE;

  // the bookmark must be less than write pointer
  if(lms511_buffer_rx_bookmark == lms511_buffer_rx_ptr_wr)
    return 0;

  // if it doesn't roll up then it copy message into temp buffer
  // else it copy the last part of the buffer and the first one until data
  // length
  if(lms511_buffer_rx_bookmark <= lms511_buffer_rx_ptr_wr)
  {
    length_to_write = lms511_buffer_rx_ptr_wr - lms511_buffer_rx_bookmark;
    memcpy(lms511_buffer_rx_temp, &lms511_buffer_rx[lms511_buffer_rx_bookmark],
        length_to_write);
  }
  else
  {
    length_to_write = (LMS511_BUFFER_SIZE - lms511_buffer_rx_bookmark);

    memcpy(lms511_buffer_rx_temp, &lms511_buffer_rx[lms511_buffer_rx_bookmark],
        length_to_write);
    memcpy(&lms511_buffer_rx_temp[length_to_write], lms511_buffer_rx,
        lms511_buffer_rx_ptr_wr + 1);

    length_to_write = length_to_write + lms511_buffer_rx_ptr_wr + 1;
  }

  // imposto il limite per la ricerca del token
  lms511_buffer_rx_temp[length_to_write] = '\0';

  // it checks for null character into the string
  // and replace it with 0x01 character.
  null_character = strchr(lms511_buffer_rx_temp, '\0');
  while(null_character != NULL)
  {
    if(null_character < &lms511_buffer_rx_temp[length_to_write])
      *null_character = 1;
    else
      break;

    null_character = strchr(lms511_buffer_rx_temp, '\0');
  }

  // it search for token
  for(token_count = 0; token_count < token_number; token_count++)
  {
    token_ptr[token_count] = strchr(lms511_buffer_rx_temp, token[token_count]);

    if(token_ptr[token_count] > token_winner)
      token_winner = token_ptr[token_count];
  }

  if(token_winner == NULL)
  {
    lms511_buffer_rx_bookmark = lms511_buffer_rx_ptr_wr;
    lms511_buffer_rx_empty = 1;

    return 0;
  }

  lms511_buffer_rx_full = 0;

  // Questa volta il controllo deve essere fatto sul token, perchè, anche se il
  // puntatore di scrittura ha passato il limite, il token potrebbe essere ancora
  // nella coda del buffer
  if((lms511_buffer_rx_bookmark >= lms511_buffer_rx_ptr_rd)
      && ((lms511_buffer_rx_ptr_rd + (token_winner - lms511_buffer_rx_temp + 1))
          < LMS511_BUFFER_SIZE))
  {
    length_to_write = (token_winner - lms511_buffer_rx_temp + 1)
        + (lms511_buffer_rx_bookmark - lms511_buffer_rx_ptr_rd);
    memcpy(data, &lms511_buffer_rx[lms511_buffer_rx_ptr_rd], length_to_write);
  }
  else
  {
    length_to_write = (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd);

    memcpy(data, &lms511_buffer_rx[lms511_buffer_rx_ptr_rd], length_to_write);

    if(lms511_buffer_rx_bookmark >= lms511_buffer_rx_ptr_rd)
      length_to_write = (token_winner - lms511_buffer_rx_temp + 1)
          + (lms511_buffer_rx_bookmark - lms511_buffer_rx_ptr_rd) - length_to_write;
    else
      length_to_write = (token_winner - lms511_buffer_rx_temp + 1) + lms511_buffer_rx_bookmark;

    memcpy(&data[LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd], lms511_buffer_rx,
        length_to_write);

    length_to_write += (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd);
  }

  data[length_to_write - 1] = 0;

  token_ptr[0] = strchr(data, '\377');

  if(token_ptr[0] != NULL)
  {
    lms511_buffer_rx_temp[token_ptr[0] - lms511_buffer_rx_temp] = '\0';
    strcat(lms511_buffer_rx_temp, &lms511_buffer_rx_temp[token_ptr[0] - lms511_buffer_rx_temp + 1]);

    memcpy(data, lms511_buffer_rx_temp, strlen(lms511_buffer_rx_temp));

    printf("Rs232 Frame error: %s\nat %i and long %i\n", lms511_buffer_rx_temp,
        token_ptr[0] - lms511_buffer_rx_temp, strlen(lms511_buffer_rx_temp));
  }

  lms511_buffer_rx_data_count -= length_to_write;

  if((lms511_buffer_rx_data_count < 0)
      || (lms511_buffer_rx_data_count > LMS511_BUFFER_SIZE))
    lms511_buffer_rx_empty = 1;

  if(lms511_buffer_rx_data_count == 0)
    lms511_buffer_rx_empty = 1;

  lms511_buffer_rx_ptr_rd += length_to_write;

  if(lms511_buffer_rx_ptr_rd >= LMS511_BUFFER_SIZE)
    lms511_buffer_rx_ptr_rd -= LMS511_BUFFER_SIZE;

  lms511_buffer_rx_bookmark = lms511_buffer_rx_ptr_rd;

  if((token_ptr[0] != NULL) && ((token_ptr[0] - lms511_buffer_rx_temp) <= length_to_write))
    //return -2;
    return strlen(lms511_buffer_rx_temp);
  else
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
    memcpy(lms511_buffer_rx_temp, &lms511_buffer_rx[lms511_buffer_rx_ptr_rd],
        lms511_buffer_rx_ptr_wr - lms511_buffer_rx_ptr_rd);
    lms511_buffer_rx_temp[lms511_buffer_rx_ptr_wr - lms511_buffer_rx_ptr_rd] = '\0';
  }
  else
  {
    // Search into the buffer for the token
    memcpy(lms511_buffer_rx_temp, &lms511_buffer_rx[lms511_buffer_rx_ptr_rd],
    LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd);
    memcpy(&lms511_buffer_rx_temp[LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd],
        &lms511_buffer_rx[0], lms511_buffer_rx_ptr_wr);

    lms511_buffer_rx_temp[LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_rd + lms511_buffer_rx_ptr_wr] =
        '\0';
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
      bytes_sent = sendto(device, &lms511_buffer_tx[lms511_buffer_tx_ptr_rd], length_to_write, 0,
          (struct sockaddr *) dest_address, sizeof(*dest_address));

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
      memcpy(buffer, &lms511_buffer_tx[lms511_buffer_tx_ptr_rd],
          (LMS511_BUFFER_SIZE - lms511_buffer_tx_ptr_rd));
      memcpy(&buffer[LMS511_BUFFER_SIZE - lms511_buffer_tx_ptr_rd], &lms511_buffer_tx[0],
          lms511_buffer_tx_ptr_wr);

      bytes_sent = sendto(device, buffer, length_to_write, 0, (struct sockaddr *) dest_address,
          sizeof(*dest_address));

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
        memcpy(&lms511_buffer_rx[lms511_buffer_rx_ptr_wr], buffer,
            (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_wr));
        memcpy(&lms511_buffer_rx[0], &buffer[(LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_wr)],
            (bytes_read - (LMS511_BUFFER_SIZE - lms511_buffer_rx_ptr_wr)));

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

/***
 * Converte il frame di risposta ad una richiesta di configurazione in dati.
 *
 * Input:
 *  lms511_command_token: puntatore alla stringa contenente i dati
 *  lms511_info: struttura dove memorizzare i dati tradotti
 *
 * Output:
 *  0: nessun errore
 *  1: frequenza non valida
 *  2: risoluzione angolare non valida
 *  3: frequenza e risoluzione angolare non validi
 *  4: area di scansione non valida
 *  5: errore generico
 */
int lms511_parse_config(char *lms511_command_token, struct LMS511_INFO *lms511_info)
{
  char lms511_value_count;
  char *lms511_value_token;

  // skip text
  lms511_command_token += sizeof("sAN mLMPsetscancfg");

  //printf("Scandata: %s\n", lms511_command_token);
  lms511_value_count = 0;

  while((lms511_value_token = strchr(lms511_command_token, ' ')) != NULL)
  {
    *lms511_value_token = '\0';

    switch(lms511_value_count)
    {
      case 0: //error
        sscanf(lms511_command_token, "%d", &lms511_info->error);

        if(lms511_info->error > 0)
          return lms511_info->error;
        break;

      case 1: //scanning frequency 1/100 Hz
        sscanf(lms511_command_token, "%lx", &lms511_info->scanning_frequency);
        break;

      case 3: //angular resolution
        sscanf(lms511_command_token, "%lx", &lms511_info->angle_resolution);
        //printf("Value: %d\n", lms511_info->angular_step);
        break;

      case 4: //starting angle
        sscanf(lms511_command_token, "%lx", &lms511_info->starting_angle);
        break;
    }

    lms511_value_count++;
    lms511_command_token += (lms511_value_token - lms511_command_token + 1);
  }

  // L'ultimo valore rimasto è ancora buono
  sscanf(lms511_command_token, "%lx", &lms511_info->stopping_angle);

  return 0;
}

int lms511_parse_data(char *lms511_command_token, struct LMS511_INFO *lms511_info)
{
  char lms511_value_count;
  char *lms511_value_token;

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
        sscanf(lms511_command_token, "%lx", &lms511_info->scaling_factor);
        //printf("Value: %f\n", convert_to_float(lms511_info->scaling_factor));
        break;

      case 23: //starting angle
        sscanf(lms511_command_token, "%lx", &lms511_info->starting_angle);
        //printf("Value: %ld\n", lms511_info->starting_angle);
        break;

      case 24: //angular step width
        sscanf(lms511_command_token, "%x", &lms511_info->angular_step);
        //printf("Value: %d\n", lms511_info->angular_step);
        break;

      case 25: //number of data
        sscanf(lms511_command_token, "%x", &lms511_info->spot_number);
        //printf("Value: %d\n", lms511_info->spot_number);

        if(lms511_info->data.spot == NULL)
        {
          lms511_info->data.spot_number = lms511_info->spot_number;
          //printf("Allocate %d instead %d\n",sizeof(*lms511_info->data.spot) * lms511_info->data.spot_number, lms511_info->data.spot_number);
          lms511_info->data.spot = malloc(
              sizeof(*lms511_info->data.spot) * lms511_info->data.spot_number);
        }

        // If I haven't enough space then allocate it
        if(lms511_info->data.spot_number < lms511_info->spot_number)
        {
          //printf("Resize data\n");
          lms511_info->data.spot_number = lms511_info->spot_number;
          free(lms511_info->data.spot);
          lms511_info->data.spot = malloc(
              sizeof(*lms511_info->data.spot) * lms511_info->data.spot_number);
        }

        //printf("Get data\n");
        for(lms511_count = 0; lms511_count < lms511_info->spot_number; lms511_count++)
        {
          // move to the next value
          lms511_command_token += (lms511_value_token - lms511_command_token + 1);
          lms511_value_token = strchr(lms511_command_token, ' ');

          if(lms511_value_token != NULL)
          {
            *lms511_value_token = '\0';
            sscanf(lms511_command_token, "%x", &lms511_info->data.spot[lms511_count]);

            lms511_info->data.spot[lms511_count] *= convert_to_float(lms511_info->scaling_factor);
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

  return 0;
}

int lms511_parse(int socket_lms511)
{
  int bytes_read; // to check how many bytes has been read
  char lms511_buffer[LMS511_BUFFER_SIZE];
  char lms511_message_buffer[2048];
  char *lms511_message_token;
  char *lms511_command_token;
  char *lms511_value_token;
  char lms511_endline[2];

  if(socket_lms511 < 0)
    return -1;

  lms511_endline[0] = LMS511_ENDLINE;
  lms511_endline[1] = '\0';

  bytes_read = lms511_read(socket_lms511);

  if(bytes_read > 0)
  {
    bytes_read = lms511_unload_rx_multifiltered(lms511_buffer, lms511_endline, 1);
    //bytes_read = lms511_unload_rx_filtered(lms511_buffer, lms511_endline[0]);

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
            lms511_parse_data(lms511_command_token, &lms511_info);

          /* searching for sSN LMDscandata message */
          strcpy(lms511_message_buffer, lms511_message_token);
          lms511_command_token = strstr(lms511_message_buffer, "sSN LMDscandata ");

          if(lms511_command_token != NULL)
            lms511_parse_data(lms511_command_token, &lms511_info);

          /* searching for sSN LMDscandata message */
          strcpy(lms511_message_buffer, lms511_message_token);
          lms511_command_token = strstr(lms511_message_buffer, "sAN mLMPsetscancfg ");

          if(lms511_command_token != NULL)
          {
            lms511_parse_config(lms511_command_token, &lms511_info);

            printf(
                "Configuration data acquire\n\terror: %d\n\tfrequency: %ld\n\tresolution: %ld\n\tstart: %ld\n\tstop: %ld\n",
                lms511_info.error, lms511_info.scanning_frequency, lms511_info.angle_resolution,
                lms511_info.starting_angle, lms511_info.stopping_angle);
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

/***
 * Configura il dispositivo LMS511
 *
 * Input:
 *  scanning_freq: frequenza di scansione in 1/100 Hz. I valori ammessi sono
 *                  2500
 *                  3500
 *                  5000
 *                  7500
 *                  10000
 *  angle_resolution: risoluzione angolare in 1/10000 °. I valori ammessi sono:
 *                  1667
 *                  2500
 *                  3333
 *                  5000
 *                  6667
 *                  7500
 *                  10000
 *  starting_angle: angolo di partenza in 1/10000 °. Deve essere compreso tra -50000 e +1850000
 *  stopping_angle: angolo di fine scansione in 1/10000 °. Deve essere compreso tra -50000 e +1850000
 *
 *  Output:
 *    0: successo
 *    -1: parametri errati
 *    -2: errore nella trasmissione del dato
 *
 *  Note:
 *    Non tutte le configurazioni sono possibili, ma bisogna attenersi alle seguenti:
 *
 *    --------------------------------------------------------------------------------
 *    |  25 Hz   | 25 Hz | 35 Hz |  50 Hz  | 50 Hz | 75 Hz | 75 Hz | 100 Hz | 100 Hz |
 *    |----------|-------|-------|---------|-------|-------|-------|--------|--------|
 *    | 0.1667°  | 0.25° | 0.25° |  0.333° |  0.5  |  0.5° |  1.0° | 0.667° | 1.0 °  |
 *    --------------------------------------------------------------------------------
 *
 */
int lms511_config_set(long scanning_freq, long angle_resolution, long starting_angle,
    long stopping_angle)
{
  // controllo la validità dei parametri passati
  switch(scanning_freq)
  {
    case 2500:
      if((angle_resolution != 1667) && (angle_resolution != 2500))
        return -1;
      break;

    case 3500:
      if(angle_resolution != 2500)
        return -1;
      break;

    case 5000:
      if((angle_resolution != 3333) && (angle_resolution != 5000))
        return -1;
      break;

    case 7500:
      if((angle_resolution != 5000) && (angle_resolution != 10000))
        return -1;
      break;

    case 10000:
      if((angle_resolution != 6667) && (angle_resolution != 10000))
        return -1;
      break;

    default:
      return -1;
      break;
  }

  if((starting_angle < -50000) || (starting_angle > 1850000))
    return -1;

  if((stopping_angle < -50000) || (stopping_angle > 1850000))
    return -1;

  char buffer[128];

  sprintf(buffer, "sMN mLMPsetscancfg %+ld +1 %+ld %+ld %+ld", scanning_freq, angle_resolution,
      starting_angle, stopping_angle);

  if(lms511_send_command(buffer) > 0)
    return 0;
  else
    return -2;
}

/*void signal_handler(int signum)
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

 // Init lms511 Client
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
 }*/

void lms511_dispose()
{
  if(lms511_info.data.spot != NULL)
    free(lms511_info.data.spot);
}
