#include <sys/select.h>
#include <linux/types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <linux/serial.h>
#include <termios.h>
#include <ctype.h>

#include "arm_rs485.h"

#define ARM_JOINT_AUTO_YZ_STEP_M 0.0009

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int signum(float n);
void arm_ee_tetha_xyz(float tetha0_rad, float tetha1_rad, float tetha2_rad, float *pw_x,
    float *pw_y, float *pw_z);

/* Global variable */
struct arm_info arm_link[MOTOR_NUMBER];
int actuator_last_action = 0;

struct arm_rs485_frame arm_rs485_buffer_tx[ARM_RS485_BUFFER_SIZE];
unsigned int arm_rs485_buffer_tx_ptr_wr = 0; // write position in tx buffer
unsigned int arm_rs485_buffer_tx_ptr_rd = 0; // read position to place data from
unsigned char arm_rs485_buffer_tx_empty = 1;
unsigned char arm_rs485_buffer_tx_full = 0;
unsigned char arm_rs485_buffer_tx_overrun = 0;
unsigned int arm_rs485_buffer_tx_data_count = 0;  // number of byte to transmit 

char arm_rs485_buffer_rx[ARM_RS485_BUFFER_SIZE];
unsigned int arm_rs485_buffer_rx_ptr_wr = 0; // write position in rx buffer
unsigned int arm_rs485_buffer_rx_ptr_rd = 0;
unsigned char arm_rs485_buffer_rx_empty = 1;
unsigned char arm_rs485_buffer_rx_full = 0;
unsigned char arm_rs485_buffer_rx_overrun = 0;
unsigned int arm_rs485_buffer_rx_data_count = 0;  // number of byte received

int arm_rs485_buffer_rx_bookmark = 0; /**< segna a che punto sono arrivato nella ricerca di un carattere nel buffer di ricezione */

const float arm_encoder_factor = 0.09;

char current_motion_file[256];
int motion_file_cursor_position = 0;
int motion_file_cursor_position_temp = 0;
double x, y, z;
int wrist_position_mode = 0;
int arm_auto_motion_xyz_mode = 0;
float arm_incremental_step_automotion_y = 0;
float arm_incremental_step_automotion_y_sign = 0;
float arm_incremental_step_automotion_z = 0;
float arm_incremental_step_automotion_z_sign = 0;
float arm_delta_y = 0;
float arm_delta_z = 0;

int signum(float n)
{
  return (n < 0) ? -1 : (n > 0) ? +1 : 0;
}

int arm_rs485_open(char *device_name, __u32 rate, char parity, int data_bits, int stop_bits)
{
  int fd;
  int local_rate = 0;
  int local_databits = 0;
  int local_stopbits = 0;
  int local_parity = 0;
  char upper_parity;
  struct termios newtio;

  // Init circular buffer
  arm_rs485_buffer_tx_ptr_wr = 0;
  arm_rs485_buffer_tx_ptr_rd = 0;
  arm_rs485_buffer_tx_empty = 1;
  arm_rs485_buffer_tx_full = 0;
  arm_rs485_buffer_tx_overrun = 0;
  arm_rs485_buffer_tx_data_count = 0;

  arm_rs485_buffer_rx_ptr_wr = 0;
  arm_rs485_buffer_rx_ptr_rd = 0;
  arm_rs485_buffer_rx_empty = 1;
  arm_rs485_buffer_rx_full = 0;
  arm_rs485_buffer_rx_overrun = 0;
  arm_rs485_buffer_rx_data_count = 0;

  fd = open(device_name, O_RDWR | O_NOCTTY);

  if(fd < 0)
    return fd;

  if(!isatty(fd))
    printf("Not a serial device\n");

  // Check fo valid values
  upper_parity = toupper(parity);

  if(((data_bits == 5) || (data_bits == 6) || (data_bits == 7) || (data_bits == 8))
      && ((stop_bits == 2) || (stop_bits == 1))
      && ((upper_parity == 'N') || (upper_parity == 'O') || (upper_parity == 'E'))
      && ((rate == 50) || (rate == 75) || (rate == 110) || (rate == 134) || (rate == 150)
          || (rate == 200) || (rate == 300) || (rate == 600) || (rate == 1200) || (rate == 2400)
          || (rate == 4800) || (rate == 9600) || (rate == 19200) || (rate == 38400)
          || (rate == 57600) || (rate == 115200)))
  {
    switch(rate)
    {
      case 50:
        local_rate = B50;
        break;
      case 75:
        local_rate = B75;
        break;
      case 110:
        local_rate = B110;
        break;
      case 134:
        local_rate = B134;
        break;
      case 150:
        local_rate = B150;
        break;
      case 200:
        local_rate = B200;
        break;
      case 300:
        local_rate = B300;
        break;
      case 600:
        local_rate = B600;
        break;
      case 1200:
        local_rate = B1200;
        break;
      case 1800:
        local_rate = B1800;
        break;
      case 2400:
        local_rate = B2400;
        break;
      case 4800:
        local_rate = B4800;
        break;
      case 9600:
        local_rate = B9600;
        break;
      case 19200:
        local_rate = B19200;
        break;
      case 38400:
        local_rate = B38400;
        break;
      case 57600:
        local_rate = B57600;
        break;
      case 115200:
        local_rate = B115200;
        break;
    }

    switch(data_bits)
    {
      case 5:
        local_databits = CS5;
        break;
      case 6:
        local_databits = CS6;
        break;
      case 7:
        local_databits = CS7;
        break;
      case 8:
        local_databits = CS8;
        break;
    }

    if(stop_bits == 2)
      local_stopbits = CSTOPB;
    else
      local_stopbits = 0;

    switch(upper_parity)
    {
      case 'E':
        local_parity = PARENB;
        break;
      case 'O':
        local_parity |= PARODD;
        break;
    }

    if(tcgetattr(fd, &newtio) < 0)  //save current port settings
      return -1;

    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    newtio.c_cflag |= (local_databits | local_stopbits | local_parity | CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARODD | CRTSCTS | PARENB);

    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    newtio.c_iflag &= ~(IGNPAR | IGNBRK | INLCR |
    ISTRIP | IXON | IXOFF | IXANY | IGNCR | PARMRK);

    newtio.c_iflag |= (BRKINT | INPCK | ICRNL);

    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    // 
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    newtio.c_oflag &= ~OPOST;

    //
    // No line processing:
    // echo off, echo newline off, canonical mode off, 
    // extended input processing off, signal chars off
    //
    //newtio.c_lflag |= ICANON;
    newtio.c_lflag &= ~(ECHO | ECHONL | ECHOE | ECHOK | IEXTEN | ISIG | ICANON);

    //newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; //inter-character timer unused
    newtio.c_cc[VMIN] = 1; //blocking read

    tcflush(fd, TCIFLUSH);

    if(cfsetispeed(&newtio, local_rate) < 0 || cfsetospeed(&newtio, local_rate) < 0)
      return -1;

    if(tcsetattr(fd, TCSANOW, &newtio) < 0)
      return -1;

    return fd;
  }
  else
  {
    close(fd);
    return -1;
  }
}

void arm_rs485_flush_device_input(int *device)
{
  tcflush(*device, TCIFLUSH);
}

void arm_rs485_flush_device_output(int *device)
{
  tcflush(*device, TCOFLUSH);
}

void arm_rs485_flush_buffer_tx(void)
{
  arm_rs485_buffer_tx_ptr_wr = 0;
  arm_rs485_buffer_tx_ptr_rd = 0;
  arm_rs485_buffer_tx_empty = 1;
  arm_rs485_buffer_tx_full = 0;
  arm_rs485_buffer_tx_overrun = 0;
  arm_rs485_buffer_tx_data_count = 0;
}

int arm_rs485_buffer_tx_get_space(void)
{
  return (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_tx_data_count);
}

int arm_rs485_buffer_rx_get_space(void)
{
  return (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_data_count);
}

int arm_rs485_load_tx(struct arm_rs485_frame data)
{
  if(arm_rs485_buffer_tx_full)
  {
    arm_rs485_buffer_tx_overrun = 1;
    return -1;
  }

  memcpy(&arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_wr], &data, sizeof(struct arm_rs485_frame));

  arm_rs485_buffer_tx_data_count++;
  arm_rs485_buffer_tx_ptr_wr++;

  if(arm_rs485_buffer_tx_ptr_wr == ARM_RS485_BUFFER_SIZE)
    arm_rs485_buffer_tx_ptr_wr = 0;

  arm_rs485_buffer_tx_empty = 0;

  if(arm_rs485_buffer_tx_data_count == ARM_RS485_BUFFER_SIZE)
    arm_rs485_buffer_tx_full = 1;

  return 1;
}

int arm_rs485_unload_rx(unsigned char *data)
{
  int length_to_write = 0;

  if(arm_rs485_buffer_rx_empty)
    return 0;

  arm_rs485_buffer_rx_full = 0;

  if(arm_rs485_buffer_rx_ptr_rd < arm_rs485_buffer_rx_ptr_wr)
  {
    length_to_write = arm_rs485_buffer_rx_ptr_wr - arm_rs485_buffer_rx_ptr_rd;
    memcpy(data, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);
    data[length_to_write] = '\0';
  }
  else
  {
    length_to_write = (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_rd);
    memcpy(data, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);
    memcpy(&data[length_to_write], &arm_rs485_buffer_rx, arm_rs485_buffer_rx_ptr_wr + 1);
    length_to_write = length_to_write + arm_rs485_buffer_rx_ptr_wr;
    data[length_to_write] = '\0';
  }

  arm_rs485_buffer_rx_data_count -= length_to_write;

  if(arm_rs485_buffer_rx_data_count == 0)
    arm_rs485_buffer_rx_empty = 1;

  arm_rs485_buffer_rx_ptr_rd += length_to_write;

  if(arm_rs485_buffer_rx_ptr_rd >= ARM_RS485_BUFFER_SIZE)
    arm_rs485_buffer_rx_ptr_rd -= ARM_RS485_BUFFER_SIZE;

  return length_to_write;
}

int arm_rs485_unload_interpolation(unsigned char *data)
{
  int length_to_write = 0;
  int message_length = 0;

  if(arm_rs485_buffer_rx_empty)
    return 0;

  if((arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd] != 0xF9)
      && (arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd] != 0xFE))
    return 0;

  if(arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd] == 0XF9)
    message_length = 6;
  else if(arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd] == 0XFE)
    message_length = 5;

  if(arm_rs485_buffer_rx_data_count < message_length)
    return 0;

  arm_rs485_buffer_rx_full = 0;

  if(arm_rs485_buffer_rx_ptr_rd < arm_rs485_buffer_rx_ptr_wr)
  {
    length_to_write = arm_rs485_buffer_rx_ptr_wr - arm_rs485_buffer_rx_ptr_rd;

    if(length_to_write > message_length)
      length_to_write = message_length;

    memcpy(data, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);
  }
  else
  {
    length_to_write = (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_rd);

    if(length_to_write > message_length)
      length_to_write = message_length;

    memcpy(data, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);

    if((length_to_write + arm_rs485_buffer_rx_ptr_wr + 1) > message_length)
    {
      memcpy(&data[length_to_write], &arm_rs485_buffer_rx, message_length - length_to_write);
      length_to_write = message_length;
    }
    else
    {
      memcpy(&data[length_to_write], &arm_rs485_buffer_rx, arm_rs485_buffer_rx_ptr_wr + 1);

      length_to_write = length_to_write + arm_rs485_buffer_rx_ptr_wr + 1;
    }
  }

  arm_rs485_buffer_rx_data_count -= length_to_write;

  if(arm_rs485_buffer_rx_data_count == 0)
    arm_rs485_buffer_rx_empty = 1;

  arm_rs485_buffer_rx_ptr_rd += length_to_write;

  if(arm_rs485_buffer_rx_ptr_rd >= ARM_RS485_BUFFER_SIZE)
    arm_rs485_buffer_rx_ptr_rd -= ARM_RS485_BUFFER_SIZE;

  arm_rs485_buffer_rx_bookmark = arm_rs485_buffer_rx_ptr_rd;

  return length_to_write;
}

int arm_rs485_unload_rx_filtered(char *data, char token)
{
  int length_to_write = 0;
  char rs485_buffer_rx_temp[ARM_RS485_BUFFER_SIZE];
  char *token_ptr;
  //int null_check_index = 0;

  //printf("unload empty: %d\n", arm_rs485_buffer_rx_empty);

  if(arm_rs485_buffer_rx_empty)
    return 0;

  // if it doesn't roll up then it copy message into temp buffer
  // else it copy the last part of the buffer and the first one until data
  // length
  if(arm_rs485_buffer_rx_ptr_rd < arm_rs485_buffer_rx_ptr_wr)
  {
    length_to_write = arm_rs485_buffer_rx_ptr_wr - arm_rs485_buffer_rx_ptr_rd;
    memcpy(rs485_buffer_rx_temp, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);
    rs485_buffer_rx_temp[length_to_write] = '\0';
  }
  else
  {
    length_to_write = (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_rd);
    memcpy(rs485_buffer_rx_temp, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);
    //printf("Copy first part...rd: %i bytes: %i\n",arm_rs485_buffer_rx_ptr_rd, length_to_write);
    memcpy(&rs485_buffer_rx_temp[length_to_write], &arm_rs485_buffer_rx,
        arm_rs485_buffer_rx_ptr_wr + 1);
    //printf("Copy second part...buffer rd: %i bytes: %i\n", length_to_write, arm_rs485_buffer_rx_ptr_wr);

    length_to_write = length_to_write + arm_rs485_buffer_rx_ptr_wr;
    rs485_buffer_rx_temp[length_to_write] = '\0';

    //printf("Rs232 rd string: %s\n", rs485_buffer_rx_temp);
  }

  token_ptr = strchr(rs485_buffer_rx_temp, token);

  //printf("token_ptr: %p Buffer: %s\n", token_ptr, rs485_buffer_rx_temp);
  if(token_ptr == NULL)
    return 0;
  else
    length_to_write = (token_ptr - rs485_buffer_rx_temp + 1);

  arm_rs485_buffer_rx_full = 0;

  token_ptr = strchr(rs485_buffer_rx_temp, 0xff);

  // if received a framing error then discard the whole message
  if(token_ptr == NULL)
    memcpy(data, rs485_buffer_rx_temp, length_to_write);
  else
  {
    rs485_buffer_rx_temp[token_ptr - rs485_buffer_rx_temp] = '\0';
    strcat(rs485_buffer_rx_temp, &rs485_buffer_rx_temp[token_ptr - rs485_buffer_rx_temp + 1]);

    memcpy(data, rs485_buffer_rx_temp, strlen(rs485_buffer_rx_temp));
    printf("Rs232 Frame error: %s\nat %i and long %i\n", rs485_buffer_rx_temp,
        token_ptr - rs485_buffer_rx_temp, strlen(rs485_buffer_rx_temp));
  }

  //printf("arm_rs485_buffer_rx_ptr_rd: %i, arm_rs485_buffer_rx_ptr_wr: %i \n", arm_rs485_buffer_rx_ptr_rd, arm_rs485_buffer_rx_ptr_wr);
  arm_rs485_buffer_rx_data_count -= length_to_write;

  if(arm_rs485_buffer_rx_data_count == 0)
    arm_rs485_buffer_rx_empty = 1;

  arm_rs485_buffer_rx_ptr_rd += length_to_write;

  if(arm_rs485_buffer_rx_ptr_rd >= ARM_RS485_BUFFER_SIZE)
  {
    //printf("Buffer rx ptr rd: %i of %i\tBytes read: %i\n", arm_rs485_buffer_rx_ptr_rd, ARM_RS485_BUFFER_SIZE, length_to_write);
    arm_rs485_buffer_rx_ptr_rd -= ARM_RS485_BUFFER_SIZE;
    //printf("Buffer rx ptr rd After: %i of %i\n", arm_rs485_buffer_rx_ptr_rd, ARM_RS485_BUFFER_SIZE);
  }

  if((token_ptr != NULL) && ((token_ptr - rs485_buffer_rx_temp) <= length_to_write))
    //return -2;
    return strlen(rs485_buffer_rx_temp);
  else
    return length_to_write;
}

int arm_rs485_unload_rx_multifiltered(char *data, char *token, char token_number)
{
  int length_to_write = 0;
  char rs485_buffer_rx_temp[ARM_RS485_BUFFER_SIZE];

  char *token_ptr[10];
  char *token_winner = NULL;
  char *null_character = NULL;
  int token_count = 0;

  if(arm_rs485_buffer_rx_empty)
    return 0;

  if(token_number > 10)
    return -1;

  // it checks if bookmark arrives to the end and if it rolled up

  // if the bookmark pass the buffer limit, then it must
  // starts from the begginning
  if(arm_rs485_buffer_rx_bookmark >= ARM_RS485_BUFFER_SIZE)
    arm_rs485_buffer_rx_bookmark = arm_rs485_buffer_rx_bookmark - ARM_RS485_BUFFER_SIZE;

  // the bookmark must be less than write pointer
  if(arm_rs485_buffer_rx_bookmark == arm_rs485_buffer_rx_ptr_wr)
    return 0;

  // if it doesn't roll up then it copy message into temp buffer
  // else it copy the last part of the buffer and the first one until data
  // length
  if(arm_rs485_buffer_rx_bookmark <= arm_rs485_buffer_rx_ptr_wr)
  {
    length_to_write = arm_rs485_buffer_rx_ptr_wr - arm_rs485_buffer_rx_bookmark;
    memcpy(rs485_buffer_rx_temp, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_bookmark],
        length_to_write);
  }
  else
  {
    length_to_write = (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_bookmark);

    memcpy(rs485_buffer_rx_temp, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_bookmark],
        length_to_write);
    memcpy(&rs485_buffer_rx_temp[length_to_write], arm_rs485_buffer_rx,
        arm_rs485_buffer_rx_ptr_wr + 1);

    length_to_write = length_to_write + arm_rs485_buffer_rx_ptr_wr + 1;
  }

  // imposto il limite per la ricerca del token
  rs485_buffer_rx_temp[length_to_write] = '\0';

  // it checks for null character into the string
  // and replace it with 0x01 character.
  null_character = strchr(rs485_buffer_rx_temp, '\0');
  while(null_character != NULL)
  {
    if(null_character < &rs485_buffer_rx_temp[length_to_write])
      *null_character = 1;
    else
      break;

    null_character = strchr(rs485_buffer_rx_temp, '\0');
  }

  // it search for token
  for(token_count = 0; token_count < token_number; token_count++)
  {
    token_ptr[token_count] = strchr(rs485_buffer_rx_temp, token[token_count]);

    if(token_ptr[token_count] > token_winner)
      token_winner = token_ptr[token_count];
  }

  if(token_winner == NULL)
  {
    arm_rs485_buffer_rx_bookmark = arm_rs485_buffer_rx_ptr_wr;
    arm_rs485_buffer_rx_empty = 1;

    return 0;
  }

  arm_rs485_buffer_rx_full = 0;

  // Questa volta il controllo deve essere fatto sul token, perchè, anche se il
  // puntatore di scrittura ha passato il limite, il token potrebbe essere ancora
  // nella coda del buffer
  //if(arm_rs485_buffer_rx_ptr_rd < arm_rs485_buffer_rx_ptr_wr)
  if((arm_rs485_buffer_rx_bookmark >= arm_rs485_buffer_rx_ptr_rd)
      && ((arm_rs485_buffer_rx_ptr_rd + (token_winner - rs485_buffer_rx_temp + 1))
          < ARM_RS485_BUFFER_SIZE))
  {
    length_to_write = (token_winner - rs485_buffer_rx_temp + 1)
        + (arm_rs485_buffer_rx_bookmark - arm_rs485_buffer_rx_ptr_rd);
    memcpy(data, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);
  }
  else
  {
    length_to_write = (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_rd);

    memcpy(data, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);

    if(arm_rs485_buffer_rx_bookmark >= arm_rs485_buffer_rx_ptr_rd)
      length_to_write = (token_winner - rs485_buffer_rx_temp + 1)
          + (arm_rs485_buffer_rx_bookmark - arm_rs485_buffer_rx_ptr_rd) - length_to_write;
    else
      length_to_write = (token_winner - rs485_buffer_rx_temp + 1) + arm_rs485_buffer_rx_bookmark;

    memcpy(&data[ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_rd], arm_rs485_buffer_rx,
        length_to_write);

    length_to_write += (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_rd);
  }

  data[length_to_write - 1] = 0;

  token_ptr[0] = strchr(data, '\377');

  if(token_ptr[0] != NULL)
  {
    rs485_buffer_rx_temp[token_ptr[0] - rs485_buffer_rx_temp] = '\0';
    strcat(rs485_buffer_rx_temp, &rs485_buffer_rx_temp[token_ptr[0] - rs485_buffer_rx_temp + 1]);

    memcpy(data, rs485_buffer_rx_temp, strlen(rs485_buffer_rx_temp));

    printf("Rs232 Frame error: %s\nat %i and long %i\n", rs485_buffer_rx_temp,
        token_ptr[0] - rs485_buffer_rx_temp, strlen(rs485_buffer_rx_temp));
  }

  arm_rs485_buffer_rx_data_count -= length_to_write;

  if((arm_rs485_buffer_rx_data_count < 0)
      || (arm_rs485_buffer_rx_data_count > ARM_RS485_BUFFER_SIZE))
    arm_rs485_buffer_rx_empty = 1;

  if(arm_rs485_buffer_rx_data_count == 0)
    arm_rs485_buffer_rx_empty = 1;

  arm_rs485_buffer_rx_ptr_rd += length_to_write;

  if(arm_rs485_buffer_rx_ptr_rd >= ARM_RS485_BUFFER_SIZE)
    arm_rs485_buffer_rx_ptr_rd -= ARM_RS485_BUFFER_SIZE;

  arm_rs485_buffer_rx_bookmark = arm_rs485_buffer_rx_ptr_rd;

  if((token_ptr[0] != NULL) && ((token_ptr[0] - rs485_buffer_rx_temp) <= length_to_write))
    //return -2;
    return strlen(rs485_buffer_rx_temp);
  else
    return length_to_write;
}

int arm_rs485_read(int device)
{
  int bytes_read = -1;
  char rs485_buffer_rx_temp[ARM_RS485_BUFFER_SIZE];

  if(arm_rs485_buffer_rx_full)
  {
    arm_rs485_buffer_rx_overrun = 1;
    return -2;
  }

  if(device > 0)
  {
    bytes_read = read(device, rs485_buffer_rx_temp, arm_rs485_buffer_rx_get_space());

    if(bytes_read <= 0)
    {
      printf("rx space: %d, ptr write: %d, ptr read: %d, count: %d, full: %d\n",
          arm_rs485_buffer_rx_get_space(), arm_rs485_buffer_rx_ptr_wr, arm_rs485_buffer_rx_ptr_rd,
          arm_rs485_buffer_rx_data_count, arm_rs485_buffer_rx_full);

      if(bytes_read == -1)
        printf("Bad file\n");

      return 0;
    }
    /*else
     {
     int i;
     printf("data: ");
     for(i = 0; i < bytes_read; i++)
     printf("%x ", rs485_buffer_rx_temp[i]);

     printf("\n");
     }*/

    if((ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr) >= bytes_read)
      memcpy(&arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_wr], rs485_buffer_rx_temp, bytes_read);
    else
    {
      //printf("Buffer reset-------------------------------------------------------------->\n");
      memcpy(&arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_wr], rs485_buffer_rx_temp,
          (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr));
      //printf("Copy first part...wr: %i bytes: %i\n",arm_rs485_buffer_rx_ptr_wr, (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr));
      memcpy(arm_rs485_buffer_rx,
          &rs485_buffer_rx_temp[ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr],
          bytes_read - (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr));
      //printf("Copy second part...buffer: %i bytes: %i\n", ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr, bytes_read - (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr));
    }

    //rs485_buffer_rx_temp[bytes_read] = '\0';
    //printf("Rs232 read: %s \nWrite pointer: %i Bytes read: %i\n", rs485_buffer_rx_temp, arm_rs485_buffer_rx_ptr_wr, bytes_read);

    if(bytes_read > 0)
    {
      arm_rs485_buffer_rx_empty = 0;
      arm_rs485_buffer_rx_data_count += bytes_read;

      if(arm_rs485_buffer_rx_data_count == ARM_RS485_BUFFER_SIZE)
        arm_rs485_buffer_rx_full = 1;

      arm_rs485_buffer_rx_ptr_wr += bytes_read;

      if(arm_rs485_buffer_rx_ptr_wr >= ARM_RS485_BUFFER_SIZE)
      {
        //printf("Buffer rx ptr wr: %i of %i\tBytes read: %i\n", arm_rs485_buffer_rx_ptr_wr, ARM_RS485_BUFFER_SIZE, bytes_read);
        arm_rs485_buffer_rx_ptr_wr -= ARM_RS485_BUFFER_SIZE;
        //printf("Buffer rx ptr wr After: %i of %i\n", arm_rs485_buffer_rx_ptr_wr, ARM_RS485_BUFFER_SIZE);
      }
    }
  }

  return bytes_read;
}

int arm_rs485_get_last_message_write(struct arm_rs485_frame *arm_rs485_buffer)
{
  if(arm_rs485_buffer_tx_ptr_rd > 0)
    memcpy(arm_rs485_buffer, &arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd - 1],
        sizeof(struct arm_rs485_frame));
  else
    memcpy(arm_rs485_buffer, &arm_rs485_buffer_tx[ARM_RS485_BUFFER_SIZE - 1],
        sizeof(struct arm_rs485_frame));

  return 0;
}

int arm_rs485_write(int device, int *query_link, unsigned char *request_position,
    unsigned char *request_trajectory_status, unsigned char *request_interpolation_status,
    unsigned char *request_error_status)
{
  int bytes_sent = 0;
  int bytes_to_send = 0;

  if(device > 0)
  {
    if(arm_rs485_buffer_tx_empty)
      return 0;

    if(isprint(arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.command[1]))
      bytes_to_send = strlen(
          arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.command);
    else
    {
      // nel conto dei caratteri ce n'� uno in pi� (il primo) che indica l'indirizzo del motore
      switch(arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.command[1])
      {
        case 0xFA:
          bytes_to_send = 6;
          break;
        case 0xFB:
          bytes_to_send = 4;
          break;
        case 0xFD:
          bytes_to_send = 6;
          break;
        default:
          bytes_to_send = strlen(
              arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.command);
          break;
      }
    }

    /*int i;
     printf("Transmit [%i]:", bytes_to_send);
     for(i = 0; i < bytes_to_send; i++)
     printf("%x ", arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.command[i]);

     printf("\n");*/

    bytes_sent = write(device,
        arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.command, bytes_to_send);

    if(bytes_sent > 0)
    {
      *request_position =
          arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.request_position;
      *request_trajectory_status =
          arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.request_trajectory_status;
      *request_interpolation_status =
          arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.request_interpolation_status;
      *request_error_status =
          arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.request_error_status;

      if(*request_position || *request_trajectory_status || *request_interpolation_status
          || *request_error_status)
        *query_link = arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.index;
      else
        *query_link = -1;

      arm_rs485_buffer_tx_full = 0;
      arm_rs485_buffer_tx_data_count--;
      arm_rs485_buffer_tx_ptr_rd++;

      if(arm_rs485_buffer_tx_ptr_rd == ARM_RS485_BUFFER_SIZE)
        arm_rs485_buffer_tx_ptr_rd = 0;
      else if(arm_rs485_buffer_tx_ptr_rd > ARM_RS485_BUFFER_SIZE)
        printf("Circular buffer critical error\n");
    }
  }

  if(arm_rs485_buffer_tx_data_count == 0)
    arm_rs485_buffer_tx_empty = 1;

  return bytes_sent;
}

int arm_set_command_without_value(int index, char *command)
{
  int bytes_sent;
  int index_start;
  int index_end;
  int i;
  struct arm_rs485_frame buffer;

  if(index == 0)
  {
    // Se non è un messaggio di richiesta, posso inviarlo
    // in broadcast senza preoccuparmi di conflitti sul bus
    if((command[0] != 'R') && (command[0] != 'Q'))
    {
      index_start = 0;
      index_end = 0;
    }
    else
    {
      index_start = 1;
      index_end = MOTOR_NUMBER;
    }
  }
  else
  {
    index_start = index;
    index_end = index;
  }

  // Ci sono dei comandi che possono essere inviati in broadcast senza
  // problemi di conflitti, come quelli che non prevedono una risposta
  for(i = index_start; i <= index_end; i++)
  {
    buffer.arm_command_param.index = i;

    if((strncmp(command, "RPA", strlen("RPA")) == 0))
    {
      buffer.arm_command_param.request_position = 1;
      buffer.arm_command_param.request_trajectory_status = 0;
      buffer.arm_command_param.request_interpolation_status = 0;
      buffer.arm_command_param.request_error_status = 0;
    }
    else if(strncmp(command, "RB(0,2)", strlen("RB(0,2)")) == 0)
    {
      buffer.arm_command_param.request_position = 0;
      buffer.arm_command_param.request_trajectory_status = 1;
      buffer.arm_command_param.request_interpolation_status = 0;
      buffer.arm_command_param.request_error_status = 0;
    }
    else if((strcmp(command, "Q") == 0) && (i <= SMART_MOTOR_NUMBER))
    {
      buffer.arm_command_param.request_position = 0;
      buffer.arm_command_param.request_trajectory_status = 0;
      buffer.arm_command_param.request_interpolation_status = 1;
      buffer.arm_command_param.request_error_status = 0;
    }
    else if((strncmp(command, "Q1", strlen("Q1")) == 0) && (i == SMART_MOTOR_SYNC_INDEX))
    {
      buffer.arm_command_param.request_position = 0;
      buffer.arm_command_param.request_trajectory_status = 0;
      buffer.arm_command_param.request_interpolation_status = 1;
      buffer.arm_command_param.request_error_status = 0;
    }
    else if((strncmp(command, "R", strlen("R")) == 0) && (i <= SMART_MOTOR_NUMBER))
    {
      buffer.arm_command_param.request_position = 0;
      buffer.arm_command_param.request_trajectory_status = 0;
      buffer.arm_command_param.request_interpolation_status = 0;
      buffer.arm_command_param.request_error_status = 1;
    }
    else
    {
      buffer.arm_command_param.request_position = 0;
      buffer.arm_command_param.request_trajectory_status = 0;
      buffer.arm_command_param.request_interpolation_status = 0;
      buffer.arm_command_param.request_error_status = 0;
    }

    sprintf(buffer.arm_command_param.command, "%c%s%c%c", i + 128, command, 0x0d, 0);

    if(i == 0)
      bytes_sent = arm_rs485_load_tx(buffer);
    else if(arm_link[i - 1].timeout_counter < LINK_TIMEOUT_LIMIT)
      bytes_sent = arm_rs485_load_tx(buffer);
  }

  return bytes_sent;
}

int arm_set_command(int index, char *command, long value)
{
  int bytes_sent;
  struct arm_rs485_frame buffer;

  buffer.arm_command_param.index = index;
  sprintf(buffer.arm_command_param.command, "%c%s=%ld%c%c", index + 128, command, value, 0x0d, 0);

  buffer.arm_command_param.request_position = 0;
  buffer.arm_command_param.request_trajectory_status = 0;
  buffer.arm_command_param.request_interpolation_status = 0;
  buffer.arm_command_param.request_error_status = 0;

  if(arm_link[index - 1].timeout_counter < LINK_TIMEOUT_LIMIT)
    bytes_sent = arm_rs485_load_tx(buffer);

  return bytes_sent;
}

int actuator_request_position()
{
  int bytes_sent;
  struct arm_rs485_frame buffer;

  buffer.arm_command_param.index = 7;
  sprintf(buffer.arm_command_param.command, "$?%c", 0);

  buffer.arm_command_param.request_position = 1;
  buffer.arm_command_param.request_trajectory_status = 0;
  buffer.arm_command_param.request_interpolation_status = 0;
  buffer.arm_command_param.request_error_status = 0;

  bytes_sent = arm_rs485_load_tx(buffer);

  return bytes_sent;
}

int actuator_request_trajectory()
{
  int bytes_sent;
  struct arm_rs485_frame buffer;

  buffer.arm_command_param.index = 7;
  sprintf(buffer.arm_command_param.command, "$?%c", 0);

  buffer.arm_command_param.request_position = 0;
  buffer.arm_command_param.request_trajectory_status = 1;
  buffer.arm_command_param.request_interpolation_status = 0;
  buffer.arm_command_param.request_error_status = 0;

  bytes_sent = arm_rs485_load_tx(buffer);

  return bytes_sent;
}

int actuator_set_command(long command)
{
  int bytes_sent;
  //struct arm_rs485_frame buffer;
  static int actuator_prev_state = 0;

  if(command > 10000) // opening
    actuator_last_action = 1;
  else if(command < -10000) // closing
    actuator_last_action = 2;
  else
    // stop
    actuator_last_action = 0;

  if(actuator_last_action == actuator_prev_state)
    return 0;
  else
    actuator_prev_state = actuator_last_action;

  switch(actuator_last_action)
  {
    case 0:
      //printf("ACTUATOR STOP\n");
      if(arm_set_command(7, "PT", 0) <= 0)
        return -1;
      break;

    case 1:
      //printf("ACTUATOR OPENING\n");
      if(arm_set_command(7, "PT", 1) <= 0)
        return -1;
      break;

    case 2:
      //printf("ACTUATOR CLOSING\n");
      if(arm_set_command(7, "PT", 2) <= 0)
        return -1;
      break;

    default:
      //printf("ACTUATOR STOP DEFAULT\n");
      if(arm_set_command(7, "PT", 0) <= 0)
        return -1;
      break;
  }

  if(arm_set_command_without_value(7, "G") <= 0)
    return -1;

  return 1;
}

int arm_init(int index, long kp, long ki, long kl, long kd, long kv, long adt, long vt, long amps,
    float upper_limit[], float lower_limit[])
{
  int i;
  long gear[MOTOR_NUMBER];
  /*  long link_length[MOTOR_NUMBER];
   long link_offset_x[MOTOR_NUMBER];
   long link_offset_y[MOTOR_NUMBER];
   long link_offset_z[MOTOR_NUMBER]; */
  int cursor_position = 0;

  // Load gear parameters
  if(arm_read_path_step(ARM_GEAR_FILE, gear, &cursor_position) > 0)
  {
    if(index == 0)
    {
      for(i = 0; i < MOTOR_NUMBER; i++)
      {
        arm_link[i].gear = gear[i];
        arm_link[i].position_initialized = 0;
        arm_link[i].timeout_counter = 0;
      }
    }
    else
    {
      arm_link[index - 1].gear = gear[index - 1];
      arm_link[index - 1].position_initialized = 0;
      arm_link[index - 1].timeout_counter = 0;
    }
  }
  else
    return -1;

  cursor_position = 0;
  // Load length parameters
  /*if(arm_read_path_step(ARM_LINK_LENGTH_FILE, link_length, &cursor_position) > 0)
   {
   if(index == 0)
   {
   for(i = 0; i < (MOTOR_NUMBER - 1); i++)
   arm_link[i].link_length = link_length[i];
   }
   else if(index < MOTOR_NUMBER)
   arm_link[index - 1].link_length = link_length[index - 1];
   else
   arm_link[index - 1].link_length = 0;
   }
   else
   {
   printf("Read path error\n");
   return -1;
   }

   // Load offsetx parameters
   if(arm_read_path_step(ARM_LINK_LENGTH_FILE, link_offset_x, &cursor_position) > 0)
   {
   if(index == 0)
   {
   for(i = 0; i < (MOTOR_NUMBER - 1); i++)
   arm_link[i].link_offset_x = link_offset_x[i];
   }
   else if(index < MOTOR_NUMBER)
   arm_link[index - 1].link_offset_x = link_offset_x[index - 1];
   else
   arm_link[index - 1].link_offset_x = 0;
   }
   else
   {
   printf("Read path error\n");
   return -1;
   }

   // Load offsety parameters
   if(arm_read_path_step(ARM_LINK_LENGTH_FILE, link_offset_y, &cursor_position) > 0)
   {
   if(index == 0)
   {
   for(i = 0; i < (MOTOR_NUMBER - 1); i++)
   arm_link[i].link_offset_y = link_offset_y[i];
   }
   else if(index < MOTOR_NUMBER)
   arm_link[index - 1].link_offset_y = link_offset_y[index - 1];
   else
   arm_link[index - 1].link_offset_y = 0;
   }
   else
   {
   printf("Read path error\n");
   return -1;
   }

   // Load offsetz parameters
   if(arm_read_path_step(ARM_LINK_LENGTH_FILE, link_offset_z, &cursor_position) > 0)
   {
   if(index == 0)
   {
   for(i = 0; i < (MOTOR_NUMBER - 1); i++)
   arm_link[i].link_offset_z = link_offset_z[i];
   }
   else if(index < MOTOR_NUMBER)
   arm_link[index - 1].link_offset_z = link_offset_z[index - 1];
   else
   arm_link[index - 1].link_offset_z = 0;
   }
   else
   {
   printf("Read path error\n");
   return -1;
   }*/

  if(index == 0)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].velocity_target_limit = arm_link[i].gear * vt;
      arm_link[i].velocity_target = 0;
    }
  }
  else
  {
    arm_link[index - 1].velocity_target_limit = arm_link[index - 1].gear * vt;
    arm_link[index - 1].velocity_target = 0;
  }

  // If there's no pending request
  // Set motor parameter
  if(arm_set_command(index, "KP", kp) <= 0)
    return -1;

  if(arm_set_command(index, "KI", ki) <= 0)
    return -1;

  if(arm_set_command(index, "KL", kl) <= 0)
    return -1;

  if(arm_set_command(index, "KD", kd) <= 0)
    return -1;

  if(arm_set_command(index, "KV", kv) <= 0)
    return -1;

  if(arm_set_command(index, "ADT", adt) <= 0)  // set acceleration and deceleration
    return -1;

  if(arm_set_command(index, "VT", 0) <= 0) //set velocity target
    return -1;

  if(arm_set_command_without_value(index, "MV") <= 0) //set velocity mode
    return -1;

  if(arm_set_command(index, "AMPS", amps) <= 0) // set pwm drive signal limit
    return -1;

  if(arm_set_command_without_value(index, "F") <= 0) // active settings
    return -1;

  if(index == 0)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].angle_upper_limit = upper_limit[i];
      arm_link[i].angle_lower_limit = lower_limit[i];

      if(arm_set_command(i + 1, "SLN",
          (long) (arm_link[i].angle_lower_limit * arm_link[i].gear * 180
              / (arm_encoder_factor * M_PI))) <= 0) // set soft left limit
        return -1;

      if(arm_set_command(i + 1, "SLP",
          (long) (arm_link[i].angle_upper_limit * arm_link[i].gear * 180
              / (arm_encoder_factor * M_PI))) <= 0) // set soft right limit
        return -1;
    }
  }
  else
  {
    arm_link[index - 1].angle_upper_limit = upper_limit[index - 1];
    arm_link[index - 1].angle_lower_limit = lower_limit[index - 1];

    if(arm_set_command(index, "SLN",
        (long) (arm_link[index - 1].angle_lower_limit * arm_link[index - 1].gear * 180
            / (arm_encoder_factor * M_PI))) <= 0) // set soft left limit
      return -1;

    if(arm_set_command(index, "SLP",
        (long) (arm_link[index - 1].angle_upper_limit * arm_link[index - 1].gear * 180
            / (arm_encoder_factor * M_PI))) <= 0) // set soft right limit
      return -1;
  }

  if(arm_set_command_without_value(0, "SLE") <= 0)  // Enable software limit  
    return -1;

  if(arm_set_command_without_value(0, "BRKSRV") <= 0)	//release brake only with servo active 
    return -1;

  if(arm_set_command_without_value(0, "G") <= 0) //set position mode
    return -1;

  if(arm_set_command_without_value(0, "X") <= 0) //set position mode
    return -1;

  if(arm_set_command_without_value(0, "OFF") <= 0) //set position mode
    return -1;

  return 1;
}

int arm_home_start(int index)
{
  if(arm_set_command_without_value(0, "ZS") <= 0)	//Clear faults
    return -1;

  if(arm_set_command_without_value(index, "MDS") <= 0)	//Clear faults
    return -1;

  if(arm_set_command(index, "KP", 3200) <= 0) // limiting current
    return -1;

  if(arm_set_command(index, "KD", 10200) <= 0) // limiting current
    return -1;

  if(arm_set_command_without_value(index, "F") <= 0)	//Clear faults
    return -1;

  if(arm_set_command_without_value(index, "SLD") <= 0)  // Disable software limit  
    return -1;

  if(arm_set_command(index, "AMPS", 200) <= 0) // limiting current
    return -1;

  if(arm_set_command(index, "VT", 100 * arm_link[index - 1].gear) <= 0) // set velocity
    return -1;

  /*if(arm_set_command(index, "ADT", 1000) <= 0) // set acceleration
   return -1;*/

  if(arm_set_command_without_value(index, "MV") <= 0)  // Set velocity mode  
    return -1;

  arm_set_command(0, "c", 0);

  if(arm_set_command_without_value(index, "G") <= 0)  // Go
    return -1;

  if(arm_set_command_without_value(index, "REA") <= 0)  // Report error position
    return -1;

  return 0;
}

void arm_set_max_velocity(int index, long velocity)
{
  int i;

  if(index == 0)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].velocity_target_limit = arm_link[i].gear * velocity;
      arm_link[i].velocity_target = 0;
    }
  }
  else
  {
    arm_link[index - 1].velocity_target_limit = arm_link[index - 1].gear * velocity;
    arm_link[index - 1].velocity_target = 0;
  }
}

int arm_start_xyz(void)
{
  int i;

  for(i = 0; i < MOTOR_NUMBER; i++)
  {
    arm_link[i].trajectory_status = 0;
    //arm_link[i].timeout_counter = 0;
  }

  if(arm_set_command_without_value(0, "ZS") <= 0)	//Clear faults
    return -1;

  wrist_position_mode = -1;

  if(arm_set_command(0, "VT", 0) <= 0) //set velocity target
    return -1;

  if(arm_set_command(1, "PT", arm_link[0].actual_position) <= 0) //set velocity target
    return -1;

  if(arm_set_command(2, "PT", arm_link[1].actual_position) <= 0) //set velocity target
    return -1;

  if(arm_set_command(3, "PT", arm_link[2].actual_position) <= 0) //set velocity target
    return -1;

  if(arm_set_command(4, "PT", arm_link[3].actual_position) <= 0) //set velocity target
    return -1;

  if(arm_set_command(5, "PT", arm_link[4].actual_position) <= 0) //set velocity target
    return -1;

  if(arm_set_command(6, "PT", arm_link[5].actual_position) <= 0) //set velocity target
    return -1;

  if(arm_set_command_without_value(0, "MP") <= 0)	//set position mode 
    return -1;

  if(arm_set_command(1, "VT", arm_link[0].velocity_target_limit) <= 0) //set velocity target
    return -1;

  if(arm_set_command(2, "VT", arm_link[1].velocity_target_limit) <= 0) //set velocity target
    return -1;

  if(arm_set_command(3, "VT", arm_link[2].velocity_target_limit) <= 0) //set velocity target
    return -1;

  if(arm_set_command(4, "VT", arm_link[3].velocity_target_limit) <= 0) //set velocity target
    return -1;

  if(arm_set_command(5, "VT", arm_link[4].velocity_target_limit) <= 0) //set velocity target
    return -1;

  if(arm_set_command(6, "VT", arm_link[5].velocity_target_limit) <= 0) //set velocity target
    return -1;

  arm_set_command_without_value(0, "G");
  return 1;
}

int arm_check_trajectory()
{
  int i;

  // check if motor has been arrived to stop
  for(i = 0; i < MOTOR_NUMBER; i++)
  {
    // Controllo se ha finito la tragliettoria solo se il link è attivo
    if((arm_link[i].trajectory_status > 0) && (arm_link[i].timeout_counter < LINK_TIMEOUT_LIMIT))
      return 0;

    // se sono arrivato all'ultimo motore, allora posso dire che le
    // tragliettorie sono concluse
    if(i == (MOTOR_NUMBER - 1))
      return 1;
  }

  return 0;
}

int arm_stop(int index)
{
  if(arm_set_command_without_value(index, "X") <= 0) //slow motion to stop
    return -1;

  // Send stop message to motors
  /*if(index == 0)
   {
   if(arm_set_command_without_value(index, "X") <= 0) //slow motion to stop
   return -1;

   if((MOTOR_NUMBER > SMART_MOTOR_NUMBER) && (arm_link[6].timeout_counter < LINK_TIMEOUT_LIMIT))
   actuator_set_command(0);
   }
   else
   {
   if((MOTOR_NUMBER > SMART_MOTOR_NUMBER) && (index == MOTOR_NUMBER))
   {
   if(arm_link[6].timeout_counter < LINK_TIMEOUT_LIMIT)
   actuator_set_command(0);
   }
   else
   {
   if(arm_set_command_without_value(index, "X") <= 0) //slow motion to stop
   return -1;
   }
   }*/

  return 0;
}

int arm_move(unsigned char triplet_selected, float value1, float value2, float value3)
{
  int i;

  /*Velocity convertion
   VT = Velocity *  ((enc. counts per rev.) / (sample rate)) * 65536 [rev/s]

   enc count per rev = 4000
   sample rate = 8000

   VT = Velocity * 32768 * 2 * PI [rad/s]
   */
  switch(triplet_selected)
  {
    case 1:
      /*for(i = 0; i < 3; i++)
       arm_link[i].trajectory_status = 1;*/

      arm_set_command_without_value(0, "MV");
      arm_set_command_without_value(0, "SLD");

      arm_set_command(1, "VT", (long) (value1 * arm_link[0].velocity_target_limit));
      arm_set_command(2, "VT", -((long) (value2 * arm_link[1].velocity_target_limit)));
      arm_set_command(3, "VT", (long) (value3 * arm_link[2].velocity_target_limit));

      arm_set_command_without_value(1, "G");
      arm_set_command_without_value(2, "G");
      arm_set_command_without_value(3, "G");
      arm_set_command(0, "c", 0);
      break;

    case 2:
      arm_set_command_without_value(0, "MV");
      /*for(i = 3; i < 6; i++)
       arm_link[i].trajectory_status = 1;*/

      //printf("Send VT Command\n");
      arm_set_command(4, "VT", (long) (value1 * arm_link[3].velocity_target_limit));
      arm_set_command(5, "VT", -((long) (value2 * arm_link[4].velocity_target_limit)));
      arm_set_command(6, "VT", (long) (value3 * arm_link[5].velocity_target_limit));

      arm_set_command_without_value(4, "G");
      arm_set_command_without_value(5, "G");
      arm_set_command_without_value(6, "G");
      arm_set_command(0, "c", 0);
      break;

    case 3:
      if(arm_link[6].timeout_counter < LINK_TIMEOUT_LIMIT)
      {
        if(value1 > 0)
          actuator_set_command(30000);
        else if(value1 < 0)
          actuator_set_command(-30000);
        else
          actuator_set_command(0);
      }
      break;

    default:
      /*for(i = 3; i < MOTOR_NUMBER; i++)
       arm_link[i].trajectory_status = 1;*/

      arm_set_command(0, "VT", 0);
      arm_set_command_without_value(0, "G");
      arm_set_command(0, "c", 0);
      break;
  } // end switch  

  return 1;
}

/* This function send appropriate command to motor
 triplet_selected: at which triplet the valueX is referred to
 value1: rapresent joint1's velocity normalized to [-1, 1]
 value2: for the first triplet rapresent the y coordinate , otherwise it's the joint2's velocity, always normalized to [-1, 1]
 value3: for the first triplet rapresent the z coordinate, for the second triplet it's the joint3's velocity, always normalized to [-1, 1],
 for the third triplet rapresent the command for the actuator
 */
int arm_move_xyz(unsigned char triplet_selected, float value1, float value2, float value3)
{
  int i;
  float tetha[MOTOR_NUMBER];
  float x, y;
  static float arm_length = 0;
  long motor_step[3];

  static double link_5_actual_position = M_PI_2;
  static double link_6_actual_position = M_PI_2;

  /*Velocity convertion
   VT = Velocity *  ((enc. counts per rev.) / (sample rate)) * 65536 [rev/s]

   enc count per rev = 4000
   sample rate = 8000

   VT = Velocity * 32768 * 2 * PI [rad/s]
   */
  switch(triplet_selected)
  {
    case 1:

      tetha[0] = arm_link[0].actual_position * M_PI * arm_encoder_factor / (180 * arm_link[0].gear);

      if(wrist_position_mode != 1)
      {
        // la prima volta ho bisogno di calcolare la lunghezza del braccio
        for(i = 0; i < 3; i++)
          motor_step[i] = arm_link[i].actual_position;

        arm_ee_xyz(motor_step, &x, &y, NULL);
        arm_length = sqrt(pow(x, 2) + pow(y, 2));

        arm_ik_ang(x, y, value3, &tetha[0], &tetha[1], &tetha[2]);

        // Devo tener conto dei limiti di giunto e delle rotazioni possibili
        for(i = 0; i < 3; i++)
        {
          if((tetha[i] < arm_link[i].angle_lower_limit)
              && (tetha[i] > arm_link[i].angle_upper_limit))
            arm_link[i].position_target = arm_link[i].actual_position;
          else
          {
            if(tetha[i] < arm_link[i].angle_lower_limit)
              tetha[i] += (2 * M_PI);

            if(tetha[i] > arm_link[i].angle_upper_limit)
              tetha[i] -= (2 * M_PI);
          }

          arm_link[i].position_target = (long) (tetha[i] * 180 * (double) arm_link[i].gear
              / (M_PI * arm_encoder_factor));
        }

        arm_set_command_without_value(1, "MV");

        wrist_position_mode = 1;
        link_5_actual_position = arm_link[4].actual_position * M_PI * arm_encoder_factor
            / (180 * arm_link[4].gear) - tetha[1] - tetha[2];
        link_6_actual_position = arm_link[5].actual_position * M_PI * arm_encoder_factor
            / (180 * arm_link[5].gear) - tetha[0];
        break;
      }

      // la variabile value2, in questo caso, rappresenta l'incremento nell'estensione del braccio
      arm_length += value2;

      x = X12 * cos(tetha[0]) - arm_length * sin(tetha[0]);
      y = X12 * sin(tetha[0]) + arm_length * cos(tetha[0]);

      arm_ik_ang(x, y, value3, &tetha[0], &tetha[1], &tetha[2]);

      // Devo tener conto dei limiti di giunto e delle rotazioni possibili
      for(i = 0; i < 3; i++)
      {
        if((tetha[i] < arm_link[i].angle_lower_limit) && (tetha[i] > arm_link[i].angle_upper_limit))
          arm_link[i].position_target = arm_link[i].actual_position;
        else
        {
          if(tetha[i] < arm_link[i].angle_lower_limit)
            tetha[i] += (2 * M_PI);

          if(tetha[i] > arm_link[i].angle_upper_limit)
            tetha[i] -= (2 * M_PI);
        }

        arm_link[i].position_target = (long) (tetha[i] * 180 * (double) arm_link[i].gear
            / (M_PI * arm_encoder_factor));
      }

      arm_set_command(1, "VT", (long) (value1 * arm_link[0].velocity_target_limit));
      arm_set_command(2, "PT", arm_link[1].position_target);

      arm_set_command(3, "PT", arm_link[2].position_target);

      tetha[4] = link_5_actual_position + tetha[1] + tetha[2];
      tetha[5] = link_6_actual_position + tetha[0];

      for(i = 3; i < SMART_MOTOR_NUMBER; i++)
      {
        if((tetha[i] < arm_link[i].angle_lower_limit) && (tetha[i] > arm_link[i].angle_upper_limit))
          arm_link[i].position_target = arm_link[i].actual_position;
        else
        {
          if(tetha[i] < arm_link[i].angle_lower_limit)
            tetha[i] += (2 * M_PI);

          if(tetha[i] > arm_link[i].angle_upper_limit)
            tetha[i] -= (2 * M_PI);
        }

        arm_link[i].position_target = (long) (tetha[i] * 180 * (double) arm_link[i].gear
            / (M_PI * arm_encoder_factor));
      }

      arm_set_command(5, "PT", arm_link[4].position_target);
      arm_set_command(6, "PT", arm_link[5].position_target);

      arm_set_command_without_value(0, "G");
      arm_set_command(0, "c", 0);
      break;

    case 2:

      if(wrist_position_mode != 0)
      {
        arm_set_command(0, "VT", 0);
        arm_set_command_without_value(4, "MV");
        arm_set_command_without_value(5, "MV");
        arm_set_command_without_value(6, "MV");

        wrist_position_mode = 0;
      }

      /*for(i = 3; i < 6; i++)
       arm_link[i].trajectory_status = 1;*/

      //printf("Send VT Command\n");
      arm_set_command(4, "VT", (long) (value1 * arm_link[3].velocity_target_limit));
      arm_set_command(5, "VT", -((long) (value2 * arm_link[4].velocity_target_limit)));
      arm_set_command(6, "VT", (long) (value3 * arm_link[5].velocity_target_limit));

      arm_set_command_without_value(4, "G");
      arm_set_command_without_value(5, "G");
      arm_set_command_without_value(6, "G");
      arm_set_command(0, "c", 0);
      break;

    case 3:
      if(arm_link[6].timeout_counter < LINK_TIMEOUT_LIMIT)
      {
        if(value1 > 0)
          actuator_set_command(30000);
        else if(value1 < 0)
          actuator_set_command(-30000);
        else
          actuator_set_command(0);
      }
      break;

    default:
      /*for(i = 3; i < MOTOR_NUMBER; i++)
       arm_link[i].trajectory_status = 1;*/

      arm_set_command(0, "VT", 0);
      arm_set_command_without_value(0, "G");
      arm_set_command(0, "c", 0);
      break;
  } // end switch  

  return 1;
}

/* Get link's index where to send a query and start to count for 
 timeout. This function must to be called at the same loop's frequency
 otherwise the timeout variable can't be incremented.*/
int arm_query_position(int link_to_query)
{
  arm_set_command_without_value(link_to_query, "RPA");

  return 1;
}

int arm_query_trajectory(int link_to_query)
{
  int automatic_query = 0;

  if(link_to_query > 0)
  {
    // If it's a smartmotor
    arm_set_command_without_value(link_to_query, "RB(0,2)");
  }
  else
  {
    for(automatic_query = 1; automatic_query <= MOTOR_NUMBER; automatic_query++)
    {
      //if(arm_link[automatic_query - 1].trajectory_status > 0)
      arm_set_command_without_value(automatic_query, "RB(0,2)");
    }
  }

  return 1;
}

void arm_automatic_motion_xyz_update_cursor()
{
  motion_file_cursor_position = motion_file_cursor_position_temp;
}

/* Read position target from file and call arm_start routine */
int arm_automatic_motion_xyz_start(char *motion_file)
{
  int i;
  int return_value;
  float motor_position_target[MOTOR_NUMBER];
  float yz_position_target[MOTOR_NUMBER];
  //float max_yz_range;
  long motor_step[MOTOR_NUMBER];

  if(motion_file != NULL)
  {
    motion_file_cursor_position = 0;
    motion_file_cursor_position_temp = 0;
    strcpy(current_motion_file, motion_file);
  }
  else
    motion_file_cursor_position_temp = motion_file_cursor_position;

  // get final xyz coordinates
  return_value = arm_read_path_xyz(current_motion_file, motor_position_target,
      &motion_file_cursor_position_temp);

  /*if(motion_file != NULL)
   arm_start_xyz();*/

  // if collect an error load 
  if(return_value == -1)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].position_target = arm_link[i].actual_position;

      //if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0)
      //  return -1;
    }

    arm_auto_motion_xyz_mode = 0;
    return -1;
  }
  else if(return_value == 0)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].position_target = arm_link[i].actual_position;

      //if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0) 
      //  return -1;
    }

    arm_auto_motion_xyz_mode = 0;
    return 0;
  }
  else if(return_value == 4)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      //arm_link[i].trajectory_status = 1;

      if(motion_file != NULL)
        motor_step[i] = arm_link[i].actual_position;
      else
        motor_step[i] = arm_link[i].position_target;
    }

    float arm_length = 0;
    float x, y, tetha0;
    for(i = 3; i >= 0; i--)
    {
      // Se leggo il carattere "don't care" ("x"), allora ho intenzione di mantenere
      // uno dei parametri fissi. Però non si tratta di coordinate cartesiane, ma:
      //  "don't care" sulla x -> angolo tetha0 fisso ed y rappresenta la lunghezza
      //  "don't care" sulla y -> lunghezza proiettata sul piano xy fissa ed x rappresenta l'angolo
      //  "don't care" sulla z -> z fissa
      //  "don't care" sulla pinza -> pinza fissa nell'ultimo stato
      if(motor_position_target[i] != motor_position_target[i])
      {
        switch(i)
        {
          case 0:
            arm_length = motor_position_target[1];
            tetha0 = motor_step[0] * M_PI * arm_encoder_factor / (180 * arm_link[0].gear);

            motor_position_target[0] = X12 * cos(tetha0) - arm_length * sin(tetha0);
            motor_position_target[1] = X12 * sin(tetha0) + arm_length * cos(tetha0);
            break;

          case 1:
            if(motor_position_target[0] != motor_position_target[0])
            {
              // se anche la prima coordinata è un don't care, mi basta
              // inserire le coordinate correnti
              arm_ee_xyz(motor_step, &motor_position_target[0], &motor_position_target[1], NULL);
            }
            else
            {
              // se il don't care è solo della coordinata y, allora devo mantenere
              // invariata la lunghezza del braccio proiettato sul piano xy.
              // In questo caso la coordinata x rappresenta l'angolo del giunto 1
              arm_ee_xyz(motor_step, &x, &y, NULL);
              arm_length = sqrt(pow(x, 2) + pow(y, 2));

              tetha0 = motor_position_target[0];
              motor_position_target[0] = X12 * cos(tetha0) - arm_length * sin(tetha0);
              motor_position_target[1] = X12 * sin(tetha0) + arm_length * cos(tetha0);
            }
            break;

          case 2:
            arm_ee_xyz(motor_step, NULL, NULL, &motor_position_target[2]);
            break;

          case 3:
            motor_position_target[3] = arm_link[6].actual_position;
            break;
        }

      }
    }

    // set position for last motors
    arm_ik_ang(motor_position_target[0], motor_position_target[1], motor_position_target[2],
        &yz_position_target[0], &yz_position_target[1], &yz_position_target[2]);

    yz_position_target[3] = 0;

    // Devo tener conto dei limiti di giunto e delle rotazioni possibili
    for(i = 0; i < 3; i++)
    {
      if((yz_position_target[i] < arm_link[i].angle_lower_limit)
          && (yz_position_target[i] > arm_link[i].angle_upper_limit))
        arm_link[i].position_target = arm_link[i].actual_position;
      else
      {
        if(yz_position_target[i] < arm_link[i].angle_lower_limit)
          yz_position_target[i] += (2 * M_PI);

        if(yz_position_target[i] > arm_link[i].angle_upper_limit)
          yz_position_target[i] -= (2 * M_PI);
      }

      arm_link[i].position_target = (long) (yz_position_target[i] * 180 * (double) arm_link[i].gear
          / (M_PI * arm_encoder_factor));
    }

    yz_position_target[4] = M_PI / 2 + yz_position_target[1] + yz_position_target[2];

    yz_position_target[5] = M_PI / 2 + yz_position_target[0];

    for(i = 3; i < SMART_MOTOR_NUMBER; i++)
    {
      if((yz_position_target[i] < arm_link[i].angle_lower_limit)
          && (yz_position_target[i] > arm_link[i].angle_upper_limit))
        arm_link[i].position_target = arm_link[i].actual_position;
      else
      {
        if(yz_position_target[i] < arm_link[i].angle_lower_limit)
          yz_position_target[i] += (2 * M_PI);

        if(yz_position_target[i] > arm_link[i].angle_upper_limit)
          yz_position_target[i] -= (2 * M_PI);
      }

      arm_link[i].position_target = (long) (yz_position_target[i] * 180 * (double) arm_link[i].gear
          / (M_PI * arm_encoder_factor));
    }

    if(MOTOR_NUMBER > SMART_MOTOR_NUMBER)
      arm_link[6].position_target = (long) (motor_position_target[3]);

    arm_auto_motion_xyz_mode = 1;
  }
  else if(return_value >= SMART_MOTOR_NUMBER)
  {
    for(i = 0; i < SMART_MOTOR_NUMBER; i++)
    {
      //arm_link[i].trajectory_status = 1;
      if(motor_position_target[i] != motor_position_target[i])
      {
        if(motion_file != NULL)
          arm_link[i].position_target = arm_link[i].actual_position;
      }
      else
        arm_link[i].position_target = (long) motor_position_target[i];

      //if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0)
      //  return -1;
    }

    //arm_set_command_without_value(0, "G");

    if(return_value == MOTOR_NUMBER)
    {
      //arm_link[MOTOR_NUMBER - 1].trajectory_status = 1;
      if(motor_position_target[MOTOR_NUMBER - 1] != motor_position_target[MOTOR_NUMBER - 1])
        arm_link[MOTOR_NUMBER - 1].position_target = arm_link[MOTOR_NUMBER - 1].actual_position;
      else
        arm_link[MOTOR_NUMBER - 1].position_target = (long) motor_position_target[MOTOR_NUMBER - 1];

      /*if(arm_link[6].position_target > 0)
       actuator_set_command(30000);
       else
       actuator_set_command(-30000);*/
    }

    arm_auto_motion_xyz_mode = 0;
  }
  else
    return -1;

  /*for(i = 0; i < MOTOR_NUMBER; i++)
   {
   printf("Target Position [%d]: %ld\n", i, arm_link[i].position_target);
   }

   printf("\n");*/
  return return_value;
}

void arm_automatic_motion_abort()
{
  motion_file_cursor_position = 0;
  motion_file_cursor_position_temp = 0;
}

/* At every call this function check if a motor have been arrived to
 position. If it is not the case, send a position request to update
 the position. This function must to be called at the same loop's frequency
 otherwise the timeout variable can't be incremented.
 Function arm_automatic_motion_velocity_start must be called once before this one*/
/*int arm_automatic_motion_velocity_update()
 {
 static int last_link_queried = 0;
 long arm_direction = 0;

 // Update link command
 if(last_link_queried != 0)
 {
 arm_direction = arm_link[last_link_queried - 1].actual_position - arm_link[last_link_queried -1].position_target;

 if((arm_direction >= 0))
 {
 // if link is far from the target point, set the max velocity target
 if(arm_link[last_link_queried - 1].actual_position >= (arm_link[last_link_queried -1].position_target + 10 * arm_link[last_link_queried -1].gear / arm_encoder_factor))
 arm_link[last_link_queried - 1].velocity_target = -(long)arm_link[last_link_queried - 1].velocity_target_limit;
 else if(arm_link[last_link_queried - 1].actual_position >= (arm_link[last_link_queried -1].position_target + 5 * arm_link[last_link_queried -1].gear / arm_encoder_factor))
 arm_link[last_link_queried - 1].velocity_target = -(long)arm_link[last_link_queried - 1].velocity_target_limit/2;
 else
 arm_link[last_link_queried - 1].velocity_target = -(long)arm_link[last_link_queried - 1].velocity_target_limit/2;

 //printf("velocity target for %i: %ld, velocity_target_limit: %ld\n",last_link_queried, arm_link[last_link_queried - 1].velocity_target, arm_link[last_link_queried - 1].velocity_target_limit);
 if((arm_link[last_link_queried - 1].actual_position < (arm_link[last_link_queried -1].position_target + (long)(arm_link[last_link_queried -1].gear/(2 * arm_encoder_factor)))) &&
 ((link_homing_complete & (int)pow(2, last_link_queried - 1)) == 0))
 {
 //printf("Motor%i Actual position: %ld, Position target range: %ld\n", last_link_queried, arm_link[last_link_queried - 1].actual_position, arm_link[last_link_queried -1].position_target + 400);
 if(arm_stop(last_link_queried))
 {
 //printf("link_homing_complete |= %i\n", (int)pow(2, last_link_queried - 1));
 arm_link[last_link_queried - 1].velocity_target = 0;
 link_homing_complete |= (int)pow(2, last_link_queried - 1);
 }
 }
 else
 {
 //printf("velocity target for %i: %ld\n",last_link_queried, arm_link[last_link_queried - 1].velocity_target
 arm_set_command(last_link_queried, "VT", (arm_link[last_link_queried - 1].velocity_target));
 arm_set_command_without_value(last_link_queried, "G");
 arm_set_command(last_link_queried, "c", 0);
 }
 }

 if((arm_direction < 0))
 {
 // if link is far from the target point, set the max velocity target
 if(arm_link[last_link_queried - 1].actual_position <= (arm_link[last_link_queried -1].position_target - 10 * arm_link[last_link_queried -1].gear / arm_encoder_factor))
 arm_link[last_link_queried - 1].velocity_target = arm_link[last_link_queried - 1].velocity_target_limit;
 else if(arm_link[last_link_queried - 1].actual_position <= (arm_link[last_link_queried -1].position_target - 5 * arm_link[last_link_queried -1].gear / arm_encoder_factor))
 arm_link[last_link_queried - 1].velocity_target = (long)arm_link[last_link_queried - 1].velocity_target_limit/2;
 else
 arm_link[last_link_queried - 1].velocity_target = (long)arm_link[last_link_queried - 1].velocity_target_limit/4;

 //printf("velocity target for %i: %ld\n",last_link_queried, arm_link[last_link_queried - 1].velocity_target);
 if((arm_link[last_link_queried - 1].actual_position > (arm_link[last_link_queried -1].position_target - (long)(arm_link[last_link_queried -1].gear / (2* arm_encoder_factor)))) &&
 ((link_homing_complete & (int)pow(2, last_link_queried - 1))  == 0))
 {
 //printf("Motor%i Arm direction: %ld, Actual position: %ld, Position target range: %ld\n", last_link_queried, arm_direction, arm_link[last_link_queried - 1].actual_position, arm_link[last_link_queried -1].position_target - 400);
 if(arm_stop(last_link_queried))
 {
 //printf("link_homing_complete |= %i\n", (int)pow(2, last_link_queried - 1));
 arm_link[last_link_queried - 1].velocity_target = 0;
 link_homing_complete |= (int)pow(2, last_link_queried - 1);
 }
 }
 else
 {
 arm_set_command(last_link_queried, "VT", (arm_link[last_link_queried - 1].velocity_target));
 arm_set_command_without_value(last_link_queried, "G");
 arm_set_command(last_link_queried, "c", 0);
 }
 }
 }

 // Check if the movement has been completed
 if(link_homing_complete == (pow(2, MOTOR_NUMBER) - 1))
 {
 // read the next target. If there's not other one, then send homing complete message
 if(arm_automatic_motion_velocity_start(NULL) < 1)
 return 1;
 }

 last_link_queried++;
 if(last_link_queried > MOTOR_NUMBER)
 last_link_queried = 1;

 // if the homing for a link has been completed, then pass to the next link
 while(link_homing_complete & (int)pow(2, last_link_queried - 1))
 {
 // Checking for homing complete again. This is must be done due to the change in link_homing_complete
 // by timeout condition
 if(link_homing_complete == (pow(2, MOTOR_NUMBER) - 1))
 {
 // read the next target. If there's not other one, then send homing complete message
 if(arm_automatic_motion_velocity_start(NULL) < 1)
 return 1;
 }

 last_link_queried++;
 if(last_link_queried > MOTOR_NUMBER)
 last_link_queried = 1;

 }

 //printf("Query link %i, homing complete: %i\n", last_link_queried, link_homing_complete);
 // Get the timeout value for the link to known if a timeout accurred for the next request
 arm_query_position(last_link_queried);

 return 0;
 }*/

/* At every call this function check if a motor have been arrived to
 position.
 */
/*int arm_automatic_motion_xyz_update(int index)
 {
 int i;
 float motor_position_target[3];

 if(index == 0)
 {
 if(arm_auto_motion_xyz_mode == 1)
 {
 // calculate final destination in xyz coordinates
 arm_ee_tetha_xyz(0,
 arm_link[1].position_target * arm_encoder_factor * M_PI / (arm_link[1].gear * 180),
 arm_link[2].position_target * arm_encoder_factor * M_PI / (arm_link[2].gear * 180),
 &motor_position_target[0], &motor_position_target[1], &motor_position_target[2]);

 if(((motor_position_target[1] - arm_incremental_step_automotion_y)
 * arm_incremental_step_automotion_y_sign > 0)
 || ((motor_position_target[2] - arm_incremental_step_automotion_z)
 * arm_incremental_step_automotion_z_sign > 0))
 {
 if((motor_position_target[1] - arm_incremental_step_automotion_y)
 * arm_incremental_step_automotion_y_sign > 0)
 arm_incremental_step_automotion_y += arm_delta_y;

 if((motor_position_target[2] - arm_incremental_step_automotion_z)
 * arm_incremental_step_automotion_z_sign > 0)
 arm_incremental_step_automotion_z += arm_delta_z;

 // compute angles from current position plus delta
 arm_ik_ang(0, arm_incremental_step_automotion_y, arm_incremental_step_automotion_z,
 &motor_position_target[0], &motor_position_target[1], &motor_position_target[2]);

 // convert from radiant to step
 motor_position_target[1] *= 180 * arm_link[1].gear / (M_PI * arm_encoder_factor);
 motor_position_target[2] *= 180 * arm_link[2].gear / (M_PI * arm_encoder_factor);

 arm_set_command(2, "PT", (long) motor_position_target[1]);
 arm_set_command(3, "PT", (long) motor_position_target[2]);

 arm_set_command_without_value(0, "G");

 return 0;
 }
 else
 {
 motor_position_target[1] = arm_link[1].position_target;
 motor_position_target[2] = arm_link[2].position_target;
 }

 for(i = 1; i < 4; i++)
 {
 if((arm_link[i - 1].actual_position
 > (arm_link[i - 1].position_target
 + (long) (arm_link[i - 1].gear / (3 * arm_encoder_factor))))
 || (arm_link[i - 1].actual_position
 < (arm_link[i - 1].position_target
 - (long) (arm_link[i - 1].gear / (3 * arm_encoder_factor)))))
 {
 return 0;
 }
 }
 }
 else
 {
 for(i = 1; i < MOTOR_NUMBER; i++)
 {
 if((arm_link[i - 1].actual_position
 > (arm_link[i - 1].position_target
 + (long) (arm_link[i - 1].gear / (3 * arm_encoder_factor))))
 || (arm_link[i - 1].actual_position
 < (arm_link[i - 1].position_target
 - (long) (arm_link[i - 1].gear / (3 * arm_encoder_factor)))))
 {
 return 0;
 }
 }
 }

 if(arm_link[MOTOR_NUMBER - 1].actual_position != 0)
 return 0;
 }
 else
 {
 if(index != MOTOR_NUMBER)
 {
 if((arm_link[index - 1].actual_position
 > (arm_link[index - 1].position_target
 + (long) (arm_link[index - 1].gear / (3 * arm_encoder_factor))))
 || (arm_link[index - 1].actual_position
 < (arm_link[index - 1].position_target
 - (long) (arm_link[index - 1].gear / (3 * arm_encoder_factor)))))
 {
 return 0;
 }
 }
 else if(arm_link[MOTOR_NUMBER - 1].actual_position != 0)
 return 0;
 }

 return 1;
 }*/

int arm_read_path_step(const char *file_path, long *motor_position_target, int *cursor_position)
{
  FILE *file = NULL;
  char *line = NULL;
  size_t len = 0;
  ssize_t read;

  char *token;
  int count = 0;

  // Init Log File
  file = fopen(file_path, "r");

  if(file == NULL)
    return -1;

  if(fseek(file, *cursor_position, SEEK_SET) == -1)
  {
    fclose(file);
    return -1;
  }

  if((read = getline(&line, &len, file)) != -1)
  {
    //printf("Line read: %s Byte read: %d\n", line, read);

    if(read < 13)
    {
      fclose(file);
      return -1;
    }

    //arm_message_log("arm_read_path_step", line);
    token = strtok(line, ",\n");
    while(token != NULL)
    {
      if(count < MOTOR_NUMBER)
        motor_position_target[count] = atol(token);

      token = strtok(NULL, ",\n");
      count++;
    }

    *cursor_position += read;
  }
  else
  {
    //*cursor_position = 0;
    free(line);
    fclose(file);
    return 0;
  }

  free(line);
  fclose(file);
  return count;
}

int arm_read_path_xyz(const char *file_path, float *motor_position_target, int *cursor_position)
{
  FILE *file = NULL;
  char *line = NULL;
  size_t len = 0;
  ssize_t read;

  char *token;
  int count = 0;

  // Init Log File
  file = fopen(file_path, "r");

  if(file == NULL)
    return -1;

  if(fseek(file, *cursor_position, SEEK_SET) == -1)
  {
    fclose(file);
    return -1;
  }

  while((read = getline(&line, &len, file)) != -1)
  {
    //printf("Line read: %s Byte read: %d, Cursor: %d\n, File: %s", line, read, *cursor_position, file_path);
    if((*line == '#') || (read < 7))
    {
      *cursor_position += read;
      continue;
    }

    //arm_message_log("arm_read_path_step", line);
    token = strtok(line, ",\r");
    while(token != NULL)
    {
      if(count < MOTOR_NUMBER)
      {
        if(strncmp(token, "x", 1) == 0)
          motor_position_target[count] = NAN;
        else
          motor_position_target[count] = atof(token);
      }
      else
      {
        // error 
        count = 0;
        break;
      }

      count++;
      token = strtok(NULL, ",\n");
    }

    *cursor_position += read;

    if(count > 0)
      break;
  }

  if(read == -1)  //end of file
  {
    free(line);
    fclose(file);
    return 0;
  }

  free(line);
  fclose(file);
  return count;
}

void arm_ik_ang(float pw_x, float pw_y, float pw_z, float *Teta1, float *Teta2, float *Teta3)
{
  double s2;
  double c2;

  double c3;
  double s3;

  // Se i parametri d'ingresso sono uguale a zero, allora restituisco gli angoli
  // attuali
  if((pw_x == 0) && (pw_y == 0))
    goto invalid_value;

  double teta_comp = asin(LENGTH_OFFSET * sin(TETHA_OFFSET) / sqrt(pow(pw_x, 2) + pow(pw_y, 2)));

  double tetha1 = ((atan2(pw_x, pw_y) - teta_comp));
  //double arm_length = length_offset * sin(TETHA_OFFSET - teta_comp) / sin(teta_comp);

  // devo prima applicare la rotazione per Teta e poi traslare tutto per gli offset
  //double x = pw_x * cos(*Teta1) - pw_y * sin(*Teta1) - x12;
  double y = pw_y * cos(tetha1) + pw_x * sin(tetha1) - Y12;
  double z = pw_z + WRIST_SIZE - Z12;

  c3 = (pow(y, 2) + pow(z, 2) - pow(A2, 2) - pow(A3, 2)) / (2 * A2 * A3);

  if((c3 > 1) || (c3 < -1))
    goto invalid_value;

  s3 = -sqrt(1 - pow(c3, 2));

  if(Teta3 != NULL)
  {
    if(y >= 0)
      *Teta3 = atan2(s3, c3);
    else
      *Teta3 = atan2(-s3, c3);
  }

  if(Teta2 != NULL)
  {
    s2 = ((A2 + A3 * c3) * z - A3 * s3 * y) / (pow(y, 2) + pow(z, 2));
    c2 = ((A2 + A3 * c3) * y + A3 * s3 * z) / (pow(y, 2) + pow(z, 2));

    if(pw_y >= 0)
      *Teta2 = atan2(s2, c2);
    else
      *Teta2 = atan2(s2, c2);
  }

  // devo cambiare segno ed aggiungere/sottrarre 2 pi per aggiustare il verso di rotazione
  // che in questo caso rimane sempre lo stesso
  if(Teta1 != NULL)
  {
    if(pw_x < 0)
      *Teta1 = -((atan2(pw_x, pw_y) - teta_comp) + 2 * M_PI);
    else
      *Teta1 = -((atan2(pw_x, pw_y) - teta_comp));
  }

  return;

  invalid_value:
  if(Teta1 != NULL)
    *Teta1 = arm_link[0].actual_position * M_PI * arm_encoder_factor / (arm_link[0].gear * 180);

  if(Teta2 != NULL)
    *Teta2 = arm_link[1].actual_position * M_PI * arm_encoder_factor / (arm_link[1].gear * 180);

  if(Teta3 != NULL)
    *Teta3 = arm_link[2].actual_position * M_PI * arm_encoder_factor / (arm_link[2].gear * 180);

  return;
}

void arm_ee_xyz(long motor_step[], float *pw_x, float *pw_y, float *pw_z)
{
  double y, z;
  float q[3];

  q[0] = motor_step[0] * M_PI * arm_encoder_factor / (arm_link[0].gear * 180);
  q[1] = motor_step[1] * M_PI * arm_encoder_factor / (arm_link[1].gear * 180);
  q[2] = motor_step[2] * M_PI * arm_encoder_factor / (arm_link[2].gear * 180);

  // calcolo le componenti considerando l'angolo teta 0 pari a zero
  y = A2 * cos(q[1]) + A3 * cos(q[2] + q[1]);
  z = A2 * sin(q[1]) + A3 * sin(q[1] + q[2]);

  // devo applicare una traslazione per gli offset
  y += Y12;
  z += (Z12 - WRIST_SIZE);

  // a questo punto posso applicare la rotazione lungo l'asse z
  // la y, in questo punto, può essere anche vista come l'estensione del braccio
  // sul piano xy
  if(pw_x != NULL)
    *pw_x = X12 * cos(q[0]) - y * sin(q[0]);

  if(pw_y != NULL)
    *pw_y = X12 * sin(q[0]) + y * cos(q[0]);

  if(pw_z != NULL)
    *pw_z = z;
}

void arm_ee_tetha_xyz(float tetha0_rad, float tetha1_rad, float tetha2_rad, float *pw_x,
    float *pw_y, float *pw_z)
{
  float a2;
  float a3;

  a2 = 0.5;
  a3 = 0.512;

  if(pw_y != NULL)
    *pw_y = a2 * cos(tetha1_rad) + a3 * cos(tetha2_rad + tetha1_rad);

  if(pw_z != NULL)
    *pw_z = a2 * sin(tetha1_rad) + a3 * sin(tetha1_rad + tetha2_rad);

  if(pw_x != NULL)
    *pw_x = 0;  //' Vector's components that representes the end-effector position
}
