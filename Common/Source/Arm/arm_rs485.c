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
void arm_ee_tetha_xyz(float tetha0_rad, float tetha1_rad, float tetha2_rad, float *pw_x, float *pw_y, float *pw_z);

/* Global variable */
struct arm_info arm_link[MOTOR_NUMBER];

struct arm_rs485_frame arm_rs485_buffer_tx[ARM_RS485_BUFFER_SIZE];
unsigned int arm_rs485_buffer_tx_ptr_wr = 0;
unsigned int arm_rs485_buffer_tx_ptr_rd = 0;
unsigned char arm_rs485_buffer_tx_empty = 1;
unsigned char arm_rs485_buffer_tx_full = 0;
unsigned char arm_rs485_buffer_tx_overrun = 0;
unsigned int arm_rs485_buffer_tx_data_count = 0;  

char arm_rs485_buffer_rx[ARM_RS485_BUFFER_SIZE];
unsigned int arm_rs485_buffer_rx_ptr_wr = 0;
unsigned int arm_rs485_buffer_rx_ptr_rd = 0;
unsigned char arm_rs485_buffer_rx_empty = 1;
unsigned char arm_rs485_buffer_rx_full = 0;
unsigned char arm_rs485_buffer_rx_overrun = 0;
unsigned int arm_rs485_buffer_rx_data_count = 0; 

const unsigned char arm_encoder_factor = 11;

char current_motion_file[256];
int motion_file_cursor_position = 0;
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
  struct termios oldtio, newtio;

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


    if(tcgetattr(fd, &oldtio) < 0)  //save current port settings
      return -1;

    bzero(&newtio, sizeof(newtio));
    
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    newtio.c_cflag = local_rate | local_databits | local_stopbits | local_parity | CLOCAL | CREAD;
    newtio.c_cflag &= ~(PARODD | PARENB | CRTSCTS);
    
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    newtio.c_iflag &= ~(IGNPAR | IGNBRK | ICRNL | INLCR |
                        ISTRIP | IXON | IXOFF | IXANY| IGNCR) | BRKINT | PARMRK | INPCK;
                        
    newtio.c_iflag |=  BRKINT | PARMRK | INPCK;
    
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
    newtio.c_lflag &= ~(ECHO | ECHONL | ECHOE | ICANON | IEXTEN | ISIG);
    
    //newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; //inter-character timer unused
    newtio.c_cc[VMIN] = 1; //blocking read until 5 chars received

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

  printf("rs232_unload_rx start\n");
  if(arm_rs485_buffer_rx_empty)
    return 0;

  arm_rs485_buffer_rx_full = 0;
 
  if(arm_rs485_buffer_rx_ptr_rd < arm_rs485_buffer_rx_ptr_wr)
    length_to_write = (arm_rs485_buffer_rx_ptr_wr - arm_rs485_buffer_rx_ptr_rd);
  else
    length_to_write = (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_rd);

  memcpy(data, &arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_rd], length_to_write);
	
  arm_rs485_buffer_rx_data_count -= length_to_write;

  if(arm_rs485_buffer_rx_data_count == 0)
    arm_rs485_buffer_rx_empty = 1;

  arm_rs485_buffer_rx_ptr_rd += length_to_write;

  if(arm_rs485_buffer_rx_ptr_rd >= ARM_RS485_BUFFER_SIZE)
    arm_rs485_buffer_rx_ptr_rd -= ARM_RS485_BUFFER_SIZE;

  //if(arm_rs485_buffer_rx_ptr_rd == ARM_RS485_BUFFER_SIZE)
  // arm_rs485_buffer_rx_ptr_rd = 0;

//  printf("\nempty: %i, data_count: %i\n", arm_rs485_buffer_rx_empty,arm_rs485_buffer_rx_data_count);
//  printf("full: %i, rd pointer: %i\n", arm_rs485_buffer_rx_full, arm_rs485_buffer_rx_ptr_rd);
//  printf("\n");

  printf("rs232_unload_rx stop\n");
  return length_to_write;
}

int arm_rs485_unload_rx_filtered(char *data, char token)
{
  int length_to_write = 0;
  char rs485_buffer_rx_temp[ARM_RS485_BUFFER_SIZE];
  char *token_ptr;

  if(arm_rs485_buffer_rx_empty)
    return 0;

  arm_rs485_buffer_rx_full = 0;
 
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
    memcpy(&rs485_buffer_rx_temp[length_to_write], &arm_rs485_buffer_rx, arm_rs485_buffer_rx_ptr_wr);
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
  {
    length_to_write = (token_ptr - rs485_buffer_rx_temp + 1);
    
    token_ptr = strchr(rs485_buffer_rx_temp, '\377');
  
    // if received a framing error then discard the whole message
    if(token_ptr == NULL)
      memcpy(data, rs485_buffer_rx_temp, length_to_write);
    else
    {
      rs485_buffer_rx_temp[token_ptr - rs485_buffer_rx_temp] = '\0';
      strcat(rs485_buffer_rx_temp, &rs485_buffer_rx_temp[token_ptr  - rs485_buffer_rx_temp + 1]);
      
      memcpy(data, rs485_buffer_rx_temp, strlen(rs485_buffer_rx_temp));
      //printf("Rs232 Frame error: %s\nat %i and long %i\n", rs485_buffer_rx_temp, token_ptr - rs485_buffer_rx_temp, strlen(rs485_buffer_rx_temp));
    }

    //printf("Rs232 string: %s\n", rs485_buffer_rx_temp);
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

int arm_rs485_read(int device)
{
  int bytes_read = -1;
  char rs485_buffer_rx_temp[ARM_RS485_BUFFER_SIZE];

  if(arm_rs485_buffer_rx_full)
  {
    arm_rs485_buffer_rx_overrun = 1;
    return -1;
  }

  if(device > 0)
  {
    bytes_read = read(device, rs485_buffer_rx_temp, arm_rs485_buffer_rx_get_space());
  
    if((ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr) >= bytes_read)
      memcpy(&arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_wr], rs485_buffer_rx_temp, bytes_read);
    else
    {
      //printf("Buffer reset-------------------------------------------------------------->\n");
      memcpy(&arm_rs485_buffer_rx[arm_rs485_buffer_rx_ptr_wr], rs485_buffer_rx_temp, (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr));
      //printf("Copy first part...wr: %i bytes: %i\n",arm_rs485_buffer_rx_ptr_wr, (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr));
      memcpy(arm_rs485_buffer_rx, &rs485_buffer_rx_temp[ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr], bytes_read - (ARM_RS485_BUFFER_SIZE - arm_rs485_buffer_rx_ptr_wr));
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

int arm_rs485_write(int device, int *query_link, unsigned char *request_position, unsigned char *request_trajectory_status)
{
  int bytes_sent = 0;

  if(device > 0)
  {
    if(arm_rs485_buffer_tx_empty)
      return 0;

    bytes_sent = write(device, arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.command, strlen(arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.command));

    if(bytes_sent > 0)
    {
      *request_position = arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.request_position;
      *request_trajectory_status = arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.request_trajectory_status;
      
      if(*request_position || *request_trajectory_status)
        *query_link = arm_rs485_buffer_tx[arm_rs485_buffer_tx_ptr_rd].arm_command_param.index;
      else
        *query_link = -1;
      
      arm_rs485_buffer_tx_full = 0;
      arm_rs485_buffer_tx_data_count--;
      arm_rs485_buffer_tx_ptr_rd ++;

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
  struct arm_rs485_frame buffer;
  
  buffer.arm_command_param.index = index;
  
  if((strncmp(command, "RPA", strlen("RPA")) == 0))
  {
    buffer.arm_command_param.request_position = 1;
    buffer.arm_command_param.request_trajectory_status = 0;
  }
  else if(strncmp(command, "RB(0,2)", strlen("RB(0,2)")) == 0)
  {
    buffer.arm_command_param.request_position = 0;
    buffer.arm_command_param.request_trajectory_status = 1;
  }
  else
  {
    buffer.arm_command_param.request_position = 0;
    buffer.arm_command_param.request_trajectory_status = 0;
  }    

  sprintf(buffer.arm_command_param.command, "%c%s%c%c", index + 128, command, 0x0d, 0);
  
  bytes_sent = arm_rs485_load_tx(buffer);

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
  
  bytes_sent = arm_rs485_load_tx(buffer);

  return bytes_sent;
}

int actuator_set_command(long command)
{
  int bytes_sent;
  struct arm_rs485_frame buffer;
  static int actuator_state = 0;
  static int actuator_prev_state = 0;
  
  if(command > 10000) // opening
    actuator_state = 1;
  else if(command < -10000) // closing
    actuator_state = 2;
  else // stop
    actuator_state = 0;

  if(actuator_state == actuator_prev_state)
    return 0;
  else
    actuator_prev_state = actuator_state;
	
  buffer.arm_command_param.index = 7;
  switch(actuator_state)
  {
    case 0: //stop
	  //printf("ACTUATOR STOP\n");
    sprintf(buffer.arm_command_param.command, "$t%c", 0);
	  break;

	case 1: //open
	  //printf("ACTUATOR OPENING\n");
	  sprintf(buffer.arm_command_param.command, "$a%c", 0);
    arm_link[MOTOR_NUMBER - 1].trajectory_status = 1;
	  break;

	case 2: //close
      //printf("ACTUATOR CLOSING\n");
      sprintf(buffer.arm_command_param.command, "$c%c", 0);
      arm_link[MOTOR_NUMBER - 1].trajectory_status = 1;
	  break;
	   
	default:
      sprintf(buffer.arm_command_param.command, "$t%c", 0);
	  break;
  }

  buffer.arm_command_param.request_position = 0;
  buffer.arm_command_param.request_trajectory_status = 0;
  
  bytes_sent = arm_rs485_load_tx(buffer);

  return bytes_sent;
}

int arm_init(int index, long kp, long ki, long kl, long kd, long kv, long adt, long vt, long amps)
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
      
      arm_link[MOTOR_NUMBER - 1].actual_position = 1;
    }
    else
    {
      arm_link[index - 1].gear = gear[index - 1];
      arm_link[index - 1].position_initialized = 0;
      arm_link[index - 1].timeout_counter = 0;
      
      if(index == MOTOR_NUMBER)
        arm_link[index - 1].actual_position = 1;
    }
    
    arm_link[ - 1].actual_position = 1;
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
    
    arm_link[MOTOR_NUMBER - 1].actual_position = 1;
  }
  else
  {
    arm_link[index - 1].velocity_target_limit = arm_link[index - 1].gear * vt;
    arm_link[index - 1].velocity_target = 0;
    
    if(index == MOTOR_NUMBER)
      arm_link[index - 1].actual_position = 1;
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
  
  if(arm_set_command(index, "AMPS", amps) <= 0) // set pwm drive signal limit
    return -1;
    
  if(arm_set_command_without_value(index, "F") <= 0) // active settings
    return -1;
      
  if(arm_set_command(1, "SLN", -490000) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(1, "SLP", 280000) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(2, "SLN", -468909) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(2, "SLP", 1381600) <= 0) // set pwm drive signal limit
    return -1;
     
  if(arm_set_command(3, "SLN", -362963) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(3, "SLP", 347340) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(4, "SLN", -138600) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(4, "SLP", 138600) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(5, "SLN", -253440) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(5, "SLP", 298496) <= 0) // set pwm drive signal limit
    return -1;
    
  if(arm_set_command(6, "SLN", -844800) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command(6, "SLP", 844800) <= 0) // set pwm drive signal limit
    return -1;
      
  if(arm_set_command_without_value(0, "SLE") <= 0)  // Disable software limit  
    return -1;

  if(arm_set_command_without_value(0, "BRKSRV") <= 0)	//release brake only with servo active 
    return -1;
    
  return 1;
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

/*int arm_start(void)
{
  int i;
  //printf("Arm Start\n");
  // Init trajectory status
  for(i = 0; i < MOTOR_NUMBER; i++)
  {
    arm_link[i].trajectory_status = 0;
    arm_link[i].timeout_counter = 0;
  }
  
  if(arm_set_command_without_value(0, "ZS") <= 0)	//Clear faults
    return -1;

  if(arm_set_command_without_value(0, "BRKSRV") <= 0)	//release brake only with servo active 
    return -1;

  if(arm_set_command(0, "VT", 0) <= 0) //set velocity target
    return -1;
      
  if(arm_set_command_without_value(0, "MV") <= 0)	//set velocity mode 
    return -1;
	
  if(arm_set_command_without_value(0, "F") <= 0) // active settings
    return -1;
    
  return 1;
}*/

int arm_start_xyz(void)
{
  int i;

  for(i = 0; i < MOTOR_NUMBER; i++)
  {
    arm_link[i].trajectory_status = 0;
    arm_link[i].timeout_counter = 0;
  }
  
  arm_link[MOTOR_NUMBER - 1].actual_position = 1;
  
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

  if(arm_set_command_without_value(0, "MP") <= 0)	//set velocity mode 
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

  return 1;
}

int arm_check_trajectory()
{
  int i;
  
  // check if motor has been arrived to stop
  for(i = 0; i < MOTOR_NUMBER; i++)
  {
    if(arm_link[i].trajectory_status > 0)
      return 0;
    
    if(i == (MOTOR_NUMBER - 1))
      return 1;
  }
  
  return 0;
}

int arm_stop(int index)
{
  // Send stop message to motors
  if(index == 0)
  {
    if(arm_set_command_without_value(index, "X") <= 0) //slow motion to stop
      return -1;

    actuator_set_command(0);
  }
  else
  {
    if(index == MOTOR_NUMBER)
      actuator_set_command(0);
    else
    {
      if(arm_set_command_without_value(index, "X") <= 0) //slow motion to stop
        return -1;
    }
  }

  return 0;
}

/*int arm_move(struct wwvi_js_event jse, __u16 joy_max_value)
{
  int i;
  int bytes_sent = 0;
  char selected_link;
  static int last_link_queried = 0;
  
  //if no new command then off
  selected_link = jse.button[0] + (jse.button[1] << 1) + (jse.button[2] << 2);

  switch(selected_link)
  {
    case 1:
      last_link_queried++;
      if(last_link_queried > 3)
        last_link_queried = 1;

      for(i = 0; i < 3; i++)
        arm_link[i].trajectory_status = 1;
      
      arm_set_command(1, "VT", ((float)jse.stick_x / joy_max_value) * arm_link[0].velocity_target_limit);
      arm_set_command(2, "VT", ((float)jse.stick_y / joy_max_value) * arm_link[1].velocity_target_limit);
      arm_set_command(3, "VT", ((float)jse.stick_z / joy_max_value) * arm_link[2].velocity_target_limit);
      arm_set_command_without_value(1, "G");
      arm_set_command_without_value(2, "G");
      arm_set_command_without_value(3, "G");
      arm_set_command(0, "c", 0);
      break;

    case 2:
      last_link_queried++;
      if((last_link_queried > 6) || (last_link_queried < 4))
        last_link_queried = 4;

      for(i = 3; i < 6; i++)
        arm_link[i].trajectory_status = 1;
        
      arm_set_command(4, "VT", ((float)jse.stick_x / joy_max_value) * arm_link[3].velocity_target_limit);
      arm_set_command(5, "VT", ((float)jse.stick_y / joy_max_value) * arm_link[4].velocity_target_limit);
      arm_set_command(6, "VT", ((float)jse.stick_z / joy_max_value) * arm_link[5].velocity_target_limit);
      arm_set_command_without_value(4, "G");
      arm_set_command_without_value(5, "G");
      arm_set_command_without_value(6, "G");
      arm_set_command(0, "c", 0);
      break;

    case 4:
      last_link_queried = 7;

      actuator_set_command(jse.stick_z);
      break;
  
    default:
      arm_set_command(0, "VT", 0);
      arm_set_command(0, "c", 0);
      break;
  } // end switch
  
  // If it's a smartmotor
  if(last_link_queried < MOTOR_NUMBER)
    arm_set_command_without_value(last_link_queried, "RPA");
  else
    actuator_request_trajectory();
    
  return bytes_sent;
}*/

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
      for(i = 0; i < 3; i++)
        arm_link[i].trajectory_status = 1;

      arm_set_command_without_value(0, "MV");
    arm_set_command_without_value(0, "SLD");

      arm_set_command(1, "VT", (long)(value1 * arm_link[0].velocity_target_limit));
      arm_set_command(2, "VT", -((long)(value2 * arm_link[1].velocity_target_limit)));
      arm_set_command(3, "VT", (long)(value3 * arm_link[2].velocity_target_limit));


     arm_set_command_without_value(1, "G");
      arm_set_command_without_value(2, "G");
      arm_set_command_without_value(3, "G");
      arm_set_command(0, "c", 0);
      break;

    case 2:
      arm_set_command_without_value(0, "MV");
      for(i = 3; i < 6; i++)
        arm_link[i].trajectory_status = 1;
        

      //printf("Send VT Command\n");
      arm_set_command(4, "VT", (long)(value1 * arm_link[3].velocity_target_limit));
      arm_set_command(5, "VT", -((long)(value2 * arm_link[4].velocity_target_limit)));
      arm_set_command(6, "VT", (long)(value3 * arm_link[5].velocity_target_limit));

      arm_set_command_without_value(4, "G");
      arm_set_command_without_value(5, "G");
      arm_set_command_without_value(6, "G");
      arm_set_command(0, "c", 0);
      break;

    case 3:
      if(value1 > 0)
        actuator_set_command(30000);
      else if(value1 < 0)
        actuator_set_command(-30000);
      else
        actuator_set_command(0);
      break;
  
    default:
      for(i = 3; i < MOTOR_NUMBER; i++)
        arm_link[i].trajectory_status = 1;
        
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
  float tetha0, tetha1, tetha2;

  static double link_4_actual_position = M_PI_2;
  static double link_5_actual_position = M_PI_2;
  
  /*Velocity convertion
    VT = Velocity *  ((enc. counts per rev.) / (sample rate)) * 65536 [rev/s]
    
    enc count per rev = 4000
    sample rate = 8000
    
    VT = Velocity * 32768 * 2 * PI [rad/s]
    */  
  switch(triplet_selected)
  {
    case 1:
      tetha0 = arm_link[0].actual_position * M_PI / (180 * arm_encoder_factor * arm_link[0].gear);
      
      arm_ik_ang(0, value2, value3, &tetha0, &tetha1, &tetha2);

      if(wrist_position_mode != 1)
      {
        arm_start_xyz();
            
        arm_set_command_without_value(1, "MV");
    
        wrist_position_mode = 1;
        link_4_actual_position = arm_link[4].actual_position * M_PI / (180 * arm_encoder_factor * arm_link[4].gear) - tetha1 - tetha2;
        link_5_actual_position = arm_link[5].actual_position * M_PI / (180 * arm_encoder_factor * arm_link[5].gear) - tetha0;
        break;
      }

      for(i = 0; i < 3; i++)
        arm_link[i].trajectory_status = 1;
        
      arm_link[4].trajectory_status = 1;
      arm_link[5].trajectory_status = 1;
      
      arm_set_command(1, "VT", (long)(value1 * arm_link[0].velocity_target_limit));
      arm_set_command(2, "PT", (long)(tetha1 * 180 * arm_encoder_factor * arm_link[1].gear / M_PI));
      arm_set_command(3, "PT", (long)(tetha2 * 180 * arm_encoder_factor * arm_link[2].gear / M_PI));
      arm_set_command(5, "PT", (long)((link_4_actual_position + tetha1 + tetha2) * 180 * arm_encoder_factor * arm_link[4].gear / M_PI));
      arm_set_command(6, "PT", (long)((link_5_actual_position + tetha0) * 180 * arm_encoder_factor * arm_link[5].gear / M_PI));

      //printf("2 %ld 3 %ld\n", (long)(tetha1 * 180 * arm_encoder_factor * arm_link[1].gear / M_PI), (long)(tetha2 * 180 * arm_encoder_factor * arm_link[2].gear / M_PI));
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

      for(i = 3; i < 6; i++)
        arm_link[i].trajectory_status = 1;
        
      //printf("Send VT Command\n");
      arm_set_command(4, "VT", (long)(value1 * arm_link[3].velocity_target_limit));
      arm_set_command(5, "VT", -((long)(value2 * arm_link[4].velocity_target_limit)));
      arm_set_command(6, "VT", (long)(value3 * arm_link[5].velocity_target_limit));

      arm_set_command_without_value(4, "G");
      arm_set_command_without_value(5, "G");
      arm_set_command_without_value(6, "G");
      arm_set_command(0, "c", 0);
      break;

    case 3:
      if(value1 > 0)
        actuator_set_command(30000);
      else if(value1 < 0)
        actuator_set_command(-30000);
      else
        actuator_set_command(0);
      break;
  
    default:
      for(i = 3; i < MOTOR_NUMBER; i++)
        arm_link[i].trajectory_status = 1;
        
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
  int automatic_query = 0;
  
  if(link_to_query > 0)
  {
    // If it's a smartmotor
    if(link_to_query < MOTOR_NUMBER)
      arm_set_command_without_value(link_to_query, "RPA");
    else
      actuator_request_position();
  }
  else
  {
    for(automatic_query = 1; automatic_query < MOTOR_NUMBER; automatic_query++)
      arm_set_command_without_value(automatic_query, "RPA");
      
    actuator_request_position();
  }

  return 1;
}

int arm_query_trajectory(int link_to_query)
{
  int automatic_query = 0;
  
  if(link_to_query > 0)
  {
    // If it's a smartmotor
    if(link_to_query < MOTOR_NUMBER)
      arm_set_command_without_value(link_to_query, "RB(0,2)");
    else
      actuator_request_trajectory();
  }
  else
  {
    for(automatic_query = 1; automatic_query <= MOTOR_NUMBER; automatic_query++)
    {
      if(arm_link[automatic_query - 1].trajectory_status > 0)
      {
        if(automatic_query == MOTOR_NUMBER)
           actuator_request_trajectory();
        else
          arm_set_command_without_value(automatic_query, "RB(0,2)");
      }
    }
  }

  return 1;
}

/* Read position target from file and call arm_start routine */
/*int arm_automatic_motion_velocity_start(char *motion_file)
{
  int i;
  int return_value;
  long motor_position_target[MOTOR_NUMBER];
  
  link_homing_complete = 0;
  
  if(motion_file != NULL)
  {
    motion_file_cursor_position = 0;
    strcpy(current_motion_file, motion_file);
  }
  
  return_value = arm_read_path_step(current_motion_file, motor_position_target, &motion_file_cursor_position);
  
  if(return_value < 0)
    return -1;
  else if(return_value < 1)
    return 0;

  for(i = 0; i < MOTOR_NUMBER; i++)
  {
    arm_link[i].position_target = motor_position_target[i];
    //printf("Target %i: %ld\n", i, arm_link[i].position_target);

    if(i == (MOTOR_NUMBER - 1))
    {
      if(arm_link[i].position_target > 0)
        actuator_set_command(30000);
      else
        actuator_set_command(-30000);
    }
  }
  
  arm_start();
  return 1;
}*/

/* Read position target from file and call arm_start routine */
int arm_automatic_motion_xyz_start(char *motion_file)
{
  int i;
  int return_value;
  float motor_position_target[MOTOR_NUMBER];
  float yz_position_target[3];
  float max_yz_range;
  
  if(motion_file != NULL)
  {
    motion_file_cursor_position = 0;
    strcpy(current_motion_file, motion_file);
  }
  
  // get final xyz coordinates
  return_value = arm_read_path_xyz(current_motion_file, motor_position_target, &motion_file_cursor_position);
  
  if(motion_file != NULL)
    arm_start_xyz();
    
  // if collect an error load 
  if(return_value == -1)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].position_target = arm_link[i].actual_position;
      
      if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0)
        return -1;
    }
    
    arm_auto_motion_xyz_mode = 0;
    return -1;
  }
  else if(return_value == 0)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].position_target = arm_link[i].actual_position;
      
      if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0) 
        return -1;
    }
    
    arm_auto_motion_xyz_mode = 0;
    return 0;
  }
  else if(return_value == 4)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
      arm_link[i].trajectory_status = 1;
      
    for(i = 0; i < 4; i++)
    {
      // if read a "don't care" character
      if(motor_position_target[i] != motor_position_target[i])
      {
        switch(i)
        {
          case 0:
            motor_position_target[0] = (double)arm_link[0].actual_position * M_PI / (180 * arm_encoder_factor * arm_link[0].gear);
            break;
            
          case 1:
            arm_ee_xyz(NULL, &motor_position_target[1], NULL);
            break;
            
          case 2:
            arm_ee_xyz(NULL, NULL, &motor_position_target[2]);
            break;
            
          case 3:
            arm_link[6].position_target = arm_link[6].actual_position;
            break;
        }
      }
    }

    // set the final angle
    arm_ik_ang(0, motor_position_target[1], motor_position_target[2], 
               &yz_position_target[0], &yz_position_target[1], &yz_position_target[2]);

    arm_link[0].position_target = (long)(motor_position_target[0] * 180 * arm_encoder_factor * arm_link[0].gear / M_PI);
    
    for(i = 1; i < 3; i++)
      arm_link[i].position_target = (long)(yz_position_target[i] * 180 * arm_encoder_factor * arm_link[i].gear / M_PI);

    arm_link[3].position_target = 0;
    arm_link[4].position_target = (long)((double)90 * arm_encoder_factor + (double)arm_link[1].actual_position / arm_link[1].gear + (double)arm_link[2].actual_position / arm_link[2].gear) * arm_link[4].gear;
    arm_link[5].position_target = (long)((double)90 * arm_encoder_factor + (double)arm_link[0].actual_position / arm_link[0].gear) * arm_link[5].gear;
    
    //arm_link[4].position_target = (long)((double)90 * arm_encoder_factor + (double)arm_link[1].position_target / arm_link[1].gear + (double)arm_link[2].position_target / arm_link[2].gear) * arm_link[4].gear;

//    if(arm_link[4].position_target > LINK5_SLP) 
//      arm_link[4].position_target = LINK5_SLP - arm_encoder_factor * arm_link[4].gear;
          
  //  if(arm_link[4].position_target < LINK5_SLN)
     // arm_link[4].position_target = LINK5_SLN + arm_encoder_factor * arm_link[4].gear; 
          
//    arm_link[5].position_target = (long)((double)90 * arm_encoder_factor + (double)arm_link[0].position_target / arm_link[0].gear) * arm_link[5].gear;

  //  if(arm_link[5].position_target > LINK6_SLP) 
    //  arm_link[5].position_target = LINK6_SLP - arm_encoder_factor * arm_link[5].gear;
          
//    if(arm_link[5].position_target < LINK6_SLN)
    //  arm_link[5].position_target = LINK6_SLN + arm_encoder_factor * arm_link[5].gear;
    
    arm_link[6].position_target = (long)(motor_position_target[3]);
    
    // get current position
    arm_ee_xyz(0, &arm_incremental_step_automotion_y, &arm_incremental_step_automotion_z);
                       
    arm_incremental_step_automotion_y_sign = signum(motor_position_target[1] - arm_incremental_step_automotion_y);
    arm_incremental_step_automotion_z_sign = signum(motor_position_target[2] - arm_incremental_step_automotion_z);
    
    max_yz_range = max((motor_position_target[1] - arm_incremental_step_automotion_y) * arm_incremental_step_automotion_y_sign,
                       (motor_position_target[2] - arm_incremental_step_automotion_z) * arm_incremental_step_automotion_z_sign);
    
    if(max_yz_range != 0)
    {    
      arm_delta_y = (motor_position_target[1] - arm_incremental_step_automotion_y) * ARM_JOINT_AUTO_YZ_STEP_M / max_yz_range;
      arm_delta_z = (motor_position_target[2] - arm_incremental_step_automotion_z) * ARM_JOINT_AUTO_YZ_STEP_M / max_yz_range;
    }
    else
    {    
      arm_delta_y = 0;
      arm_delta_z = 0;
    }
    arm_incremental_step_automotion_y += (motor_position_target[1] - arm_incremental_step_automotion_y) * arm_delta_y * arm_incremental_step_automotion_y_sign;
    
    if((motor_position_target[1] - arm_incremental_step_automotion_y) * arm_incremental_step_automotion_y_sign  < 0)
      arm_incremental_step_automotion_y = motor_position_target[1];

    arm_incremental_step_automotion_z += (motor_position_target[2] - arm_incremental_step_automotion_z) * arm_delta_z * arm_incremental_step_automotion_z_sign;
    
    if((motor_position_target[2] - arm_incremental_step_automotion_z) * arm_incremental_step_automotion_z_sign < 0)
      arm_incremental_step_automotion_z = motor_position_target[2];

    // compute angles from current position plus delta
    arm_ik_ang(0, arm_incremental_step_automotion_y, arm_incremental_step_automotion_z, 
               &motor_position_target[0], &motor_position_target[1], &motor_position_target[2]);

    // convert from radiant to step
    motor_position_target[1] *= 180 * arm_encoder_factor * arm_link[1].gear / M_PI;
    motor_position_target[2] *= 180 * arm_encoder_factor * arm_link[2].gear / M_PI;
        
    if(arm_set_command(1, "PT", arm_link[0].position_target) <= 0)
      return -1;
        
    for(i = 2; i <= 3; i++)
    {
      if(arm_set_command(i, "PT", (long)motor_position_target[i - 1]) <= 0)
        return -1;
    }
    
    for(i = 4; i <= 6; i++)
    {
      if(arm_set_command(i, "PT", arm_link[i - 1].position_target) <= 0)
        return -1;
    }

    arm_set_command_without_value(0, "G");
    
    if(arm_link[6].position_target > 0)
      actuator_set_command(30000);
    else
      actuator_set_command(-30000);
    
    arm_auto_motion_xyz_mode = 1;
  }
  else if(return_value == MOTOR_NUMBER)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].trajectory_status = 1;
      if(motor_position_target[i] != motor_position_target[i])
        arm_link[i].position_target = arm_link[i].actual_position;
      else
        arm_link[i].position_target = (long)motor_position_target[i];
      
      if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0)
        return -1;
    }
    
    arm_set_command_without_value(0, "G");

    if(arm_link[6].position_target > 0)
      actuator_set_command(30000);
    else
      actuator_set_command(-30000);
    
    arm_auto_motion_xyz_mode = 0;
  }
  else 
   return -1;
 
  return return_value;
}

void arm_automatic_motion_abort()
{
  motion_file_cursor_position = 0;
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
      if(arm_link[last_link_queried - 1].actual_position >= (arm_link[last_link_queried -1].position_target + 10 * arm_encoder_factor * arm_link[last_link_queried -1].gear))
        arm_link[last_link_queried - 1].velocity_target = -(long)arm_link[last_link_queried - 1].velocity_target_limit;
      else if(arm_link[last_link_queried - 1].actual_position >= (arm_link[last_link_queried -1].position_target + 5 * arm_encoder_factor * arm_link[last_link_queried -1].gear))
        arm_link[last_link_queried - 1].velocity_target = -(long)arm_link[last_link_queried - 1].velocity_target_limit/2;
      else
        arm_link[last_link_queried - 1].velocity_target = -(long)arm_link[last_link_queried - 1].velocity_target_limit/2;
        
      //printf("velocity target for %i: %ld, velocity_target_limit: %ld\n",last_link_queried, arm_link[last_link_queried - 1].velocity_target, arm_link[last_link_queried - 1].velocity_target_limit);
      if((arm_link[last_link_queried - 1].actual_position < (arm_link[last_link_queried -1].position_target + (long)(arm_encoder_factor * arm_link[last_link_queried -1].gear/2))) && 
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
      if(arm_link[last_link_queried - 1].actual_position <= (arm_link[last_link_queried -1].position_target - 10*arm_encoder_factor * arm_link[last_link_queried -1].gear))
        arm_link[last_link_queried - 1].velocity_target = arm_link[last_link_queried - 1].velocity_target_limit;
      else if(arm_link[last_link_queried - 1].actual_position <= (arm_link[last_link_queried -1].position_target - 5*arm_encoder_factor * arm_link[last_link_queried -1].gear))
        arm_link[last_link_queried - 1].velocity_target = (long)arm_link[last_link_queried - 1].velocity_target_limit/2;
      else
        arm_link[last_link_queried - 1].velocity_target = (long)arm_link[last_link_queried - 1].velocity_target_limit/4;
  
      //printf("velocity target for %i: %ld\n",last_link_queried, arm_link[last_link_queried - 1].velocity_target);
      if((arm_link[last_link_queried - 1].actual_position > (arm_link[last_link_queried -1].position_target - (long)(arm_encoder_factor * arm_link[last_link_queried -1].gear / 2))) &&
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
int arm_automatic_motion_xyz_update(int index)
{
  int i;
  float motor_position_target[3];
  
  if(index == 0)
  {
    if(arm_auto_motion_xyz_mode == 1)
    {
      // calculate final destination in xyz coordinates
      arm_ee_tetha_xyz(0, arm_link[1].position_target * M_PI/ (arm_encoder_factor * arm_link[1].gear * 180), 
                       arm_link[2].position_target * M_PI/ (arm_encoder_factor * arm_link[2].gear * 180), 
                       &motor_position_target[0], &motor_position_target[1], &motor_position_target[2]);
      
      if(((motor_position_target[1] - arm_incremental_step_automotion_y) * arm_incremental_step_automotion_y_sign > 0) ||
         ((motor_position_target[2] - arm_incremental_step_automotion_z) * arm_incremental_step_automotion_z_sign > 0))
      {
        if((motor_position_target[1] - arm_incremental_step_automotion_y) * arm_incremental_step_automotion_y_sign > 0)
          arm_incremental_step_automotion_y += arm_delta_y;

        if((motor_position_target[2] - arm_incremental_step_automotion_z) * arm_incremental_step_automotion_z_sign > 0)
          arm_incremental_step_automotion_z += arm_delta_z;
      
        // compute angles from current position plus delta
        arm_ik_ang(0, arm_incremental_step_automotion_y, arm_incremental_step_automotion_z, 
                   &motor_position_target[0], &motor_position_target[1], &motor_position_target[2]);

        // convert from radiant to step
        motor_position_target[1] *= 180 * arm_encoder_factor * arm_link[1].gear / M_PI;
        motor_position_target[2] *= 180 * arm_encoder_factor * arm_link[2].gear / M_PI;
      
        arm_set_command(2, "PT", (long)motor_position_target[1]);
        arm_set_command(3, "PT", (long)motor_position_target[2]);
        
        arm_set_command_without_value(0, "G");
        
        return 0;
      }
      else
      {
        motor_position_target[1] = arm_link[1].position_target;
        motor_position_target[2] = arm_link[2].position_target;
      }

      /****** Checks if all joint have arrived to the final position ********/
      for(i = 1; i < 4; i++)
      {
        if((arm_link[i - 1].actual_position > (arm_link[i -1].position_target + (long)(arm_encoder_factor * arm_link[i -1].gear / 3))) ||
           (arm_link[i - 1].actual_position < (arm_link[i -1].position_target - (long)(arm_encoder_factor * arm_link[i -1].gear / 3))))
        {
          return 0;
        }
      }
    }
    else
    {
      for(i = 1; i < MOTOR_NUMBER; i++)
      {
        if((arm_link[i - 1].actual_position > (arm_link[i -1].position_target + (long)(arm_encoder_factor * arm_link[i -1].gear / 3))) ||
           (arm_link[i - 1].actual_position < (arm_link[i -1].position_target - (long)(arm_encoder_factor * arm_link[i -1].gear / 3))))
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
      if((arm_link[index - 1].actual_position > (arm_link[index -1].position_target + (long)(arm_encoder_factor * arm_link[index -1].gear / 3))) ||
         (arm_link[index - 1].actual_position < (arm_link[index -1].position_target - (long)(arm_encoder_factor * arm_link[index -1].gear / 3))))
      {
        return 0;
      }
    }
    else if(arm_link[MOTOR_NUMBER - 1].actual_position != 0)
      return 0;
  }
  
  return 1;
}

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
    token = strtok(line,",\n");
    while(token != NULL)
    {
      if(count < MOTOR_NUMBER)
        motor_position_target[count] = atol(token);

      token = strtok(NULL,",\n");
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
    //printf("Line read: %s Byte read: %d\n", line, read);
    if((*line == '#') || (read < 7))
    {
      *cursor_position += read;
       continue;
    }
    
    //arm_message_log("arm_read_path_step", line);
    token = strtok(line,",\r");
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
      token = strtok(NULL,",\n");
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
  double c3;
  double s3;

  double a2;
  double a3;

  a2 = 0.5;
  a3 = 0.512;

  c3 = (pow(pw_x, 2) + pow(pw_y, 2) + pow(pw_z, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
  
  s3 = -sqrt(1 - pow(c3, 2));

  if(pw_y >= 0)
    *Teta3 = atan2(s3, c3);
  else
    *Teta3 = atan2(-s3, c3);

  if(pw_y >= 0)
    *Teta2 = atan2((a2 + a3 * c3) * pw_z - a3 * s3 * sqrt(pow(pw_x, 2) + pow(pw_y, 2)), (a2 + a3 * c3) * sqrt(pow(pw_x, 2) + pow(pw_y, 2)) + a3 * s3 * pw_z);
  else
    *Teta2 = atan2((a2 + a3 * c3) * pw_z - a3 * s3 * sqrt(pow(pw_x, 2) + pow(pw_y, 2)), -((a2 + a3 * c3) * sqrt(pow(pw_x, 2) + pow(pw_y, 2)) + a3 * s3 * pw_z));
    
  //Teta1 = Atan2(pw_y, pw_x) '
}

void arm_ee_xyz(float *pw_x, float *pw_y, float *pw_z)
{
  float a2;
  float a3;

  a2 = 0.5;
  a3 = 0.512;

  float q[3];
 
  q[0] = arm_link[0].actual_position * M_PI / (arm_encoder_factor * arm_link[0].gear * 180);
  q[1] = arm_link[1].actual_position * M_PI/ (arm_encoder_factor * arm_link[1].gear * 180);
  q[2] = arm_link[2].actual_position * M_PI/ (arm_encoder_factor * arm_link[2].gear * 180);
  
  if(pw_y != NULL)
    *pw_y = a2 * cos(q[1]) + a3 * cos(q[2] + q[1]);
    
  if(pw_z != NULL)
    *pw_z = a2 * sin(q[1]) + a3 * sin(q[1] + q[2]);
    
  if(pw_x != NULL)
    *pw_x = 0;  //' Vector's components that representes the end-effector position
  
}

void arm_ee_tetha_xyz(float tetha0_rad, float tetha1_rad, float tetha2_rad, float *pw_x, float *pw_y, float *pw_z)
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
