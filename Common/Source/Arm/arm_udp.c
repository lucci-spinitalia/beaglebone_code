#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>
#include <linux/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "arm_udp.h"
#include "joystick.h"

#define ARM_BUFFER_SIZE 1024
#define LOG_FILE "/var/log/arm_udp"

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
void arm_message_log(const char *, const char *); 

/* Global variable */
//struct wwvi_js_event arm_buffer_tx[ARM_BUFFER_SIZE];
struct arm_info arm_link[MOTOR_NUMBER];

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

volatile int query_link = -1;
unsigned char query_stop = -1;
volatile unsigned char arm_net_down = 1;
unsigned char link_homing_complete = 0;
char current_motion_file[256];
int motion_file_cursor_position = 0;
double x, y, z;
int wrist_position_mode = 0;
int arm_auto_motion_xyz_mode = 0;
char arm_message_count = 0;

const unsigned char arm_encoder_factor = 11; // = (4000/360) each motor has an encorder with 4000 step

  
// Debug timer
struct timespec debug_timer_start, debug_timer_stop;
unsigned char debug_timer = 0;
long debug_elapsed_time = 0;
long max_debug_elapsed_time = 0;
long min_debug_elapsed_time = 2000000000;

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

int arm_open(int *socket, struct sockaddr_in *address, int src_port, char *ip_address, int dest_port)
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
  
  if(init_client2(socket, address, src_port, ip_address, dest_port) == -1)
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

  data.arm_command_param.count++;
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

int arm_set_command_without_value(int index, char *command)
{
  int bytes_sent;
  struct arm_frame buffer;
  
  sprintf(buffer.arm_command, "%c%s%c%c", index + 128, command, 0x0d, 0);
  
  arm_message_count++;
  buffer.arm_command_param.count = arm_message_count;
  
  bytes_sent = arm_load_tx(buffer);

  return bytes_sent;
}

int arm_set_command(int index, char *command, long value)
{
  int bytes_sent;
  struct arm_frame buffer; 
  
  sprintf(buffer.arm_command, "%c%s=%ld%c%c", index + 128, command, value, 0x0d, 0);
  
  arm_message_count++;
  buffer.arm_command_param.count = arm_message_count;
  
  bytes_sent = arm_load_tx(buffer);
  
  //printf("%i%s=%ld\n", index + 128, command, value);
  return bytes_sent;
}

int actuator_request_trajectory()
{
  int bytes_sent;
  struct arm_frame buffer; 
  
  buffer.arm_command_param.index = 0x87;
  sprintf(buffer.arm_command_param.command, "$?%c%c", 0x0d, 0);
  
  arm_message_count++;
  buffer.arm_command_param.count = arm_message_count;
  
  bytes_sent = arm_load_tx(buffer);

  return bytes_sent;
}

int actuator_set_command(long command)
{
  int bytes_sent;
  struct arm_frame buffer; 
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
	
  buffer.arm_command_param.index = 0x87;
  switch(actuator_state)
  {
    case 0: //stop
	  //printf("ACTUATOR STOP\n");
    sprintf(buffer.arm_command_param.command, "$t%c%c", 0x0d, 0);
	  break;

	case 1: //open
	  //printf("ACTUATOR OPENING\n");
	  sprintf(buffer.arm_command_param.command, "$a%c%c", 0x0d, 0);
	  break;

	case 2: //close
      //printf("ACTUATOR CLOSING\n");
      sprintf(buffer.arm_command_param.command, "$c%c%c", 0x0d, 0);
	  break;
	   
	default:
      sprintf(buffer.arm_command_param.command, "$t%c%c", 0x0d, 0);
	  break;
  }
  
  arm_message_count++;
  buffer.arm_command_param.count = arm_message_count;
  bytes_sent = arm_load_tx(buffer);

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
        arm_link[i].request_actual_position = 0;
        arm_link[i].request_trajectory_status = 0;
        arm_link[i].request_timeout = 0;
      }
    }
    else
    {
      arm_link[index - 1].gear = gear[index - 1];
      arm_link[index - 1].position_initialized = 0;
      arm_link[index - 1].request_actual_position = 0;
      arm_link[index - 1].request_trajectory_status = 0;
      arm_link[index - 1].request_timeout = 0;
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
  if(query_link == -1)
  {
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
  }
    
  return 1;
}

int arm_start(void)
{
  int i;
  //printf("Arm Start\n");
  // Init trajectory status
  for(i = 0; i < MOTOR_NUMBER; i++)
    arm_link[i].trajectory_status = 1;
  
  if(query_link == -1)
  {
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
  }
    
  return 1;
}

int arm_start_xyz(void)
{
  int i;

  //printf("Arm Start\n");
  // Init trajectory status
  for(i = 0; i < MOTOR_NUMBER; i++)
    arm_link[i].trajectory_status = 1;
  
  if(arm_set_command_without_value(0, "ZS") <= 0)	//Clear faults
    return -1;

  if(arm_set_command_without_value(0, "BRKSRV") <= 0)	//release brake only with servo active 
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

int arm_stop(int index)
{
  static int last_link_queried = 0;
  static int request_time = 0;
  const int timeout_index = 1;
  int i;
  
  if(query_link == -1)
    request_time = 0;
    
  request_time++;
  
  if(request_time > (timeout_index + 10))
  {
    printf("Timeout arm_stop\n");
    // If timeout accurred increment timeout flag up to 100
    if(arm_link[query_link - 1].request_timeout < 100)
    {
      arm_link[query_link - 1].request_timeout++;
      //printf("Link %i Timout number %i\n", query_link, arm_link[query_link - 1].request_timeout);
    }
    
    arm_link[query_link - 1].request_trajectory_status = 0;
    
    // if I obtain too much timeout then consider motor off
    if(arm_link[query_link - 1].request_timeout  > 10)
      arm_link[query_link - 1].trajectory_status = 0;
    
    request_time = 0;
    query_link = -1;
  }
  
  // wait for stop
  //printf("Request time: %d Timeout index: %d\n", request_time, timeout_index);
  if(request_time == timeout_index)
  {
    if(query_link == -1)
    {
      if(index == 0)
      {
        for(i = 0; i < MOTOR_NUMBER; i++)
        {
          //printf("send stop command to all: %i %i\n", i, arm_link[i].trajectory_status);
          if(arm_link[i].trajectory_status > 0)
            break;
            
          if(i == (MOTOR_NUMBER - 1))
          {
            //if(arm_set_command_without_value(0, "OFF") <= 0) //stop servoing the motors
            //  return -1;  

            request_time = 0;
            return 1;
          }
        }
      }
      else
      {
        //printf("check stop command to all: %i %i\n", index, arm_link[index - 1].trajectory_status);
        if(arm_link[index - 1].trajectory_status == 0)
        {
          //if(arm_set_command_without_value(index, "OFF") <= 0) //stop servoing the motors
          //  return -1;  
          request_time = 0;
          return 1;
        }
      }
  
      if(index == 0)
      {
        last_link_queried++;
        if(last_link_queried > MOTOR_NUMBER)
          last_link_queried = 1;

        query_link = last_link_queried;
      }
      else
        query_link = index;
      
      //printf("Query link: %i\n", query_link);
      //Send stop command
      arm_link[query_link - 1].request_trajectory_status = 1;

      //printf("Send trajectory request to %i\n", query_link);
  
      if(query_link == MOTOR_NUMBER)
      {
        actuator_set_command(0);
        actuator_request_trajectory();
      }
      else
      {
        if(arm_set_command_without_value(query_link, "X") <= 0) //slow motion to stop
          return -1;
  
        arm_set_command_without_value(query_link, "RB(0,2)");
      }
    }
  }

  
  return 0;
}

int arm_move(struct wwvi_js_event jse, __u16 joy_max_value)
{
  int bytes_sent = 0;
  char selected_link;
  static int request_time = 0;
  static int last_link_queried = 0;
  
  const int timeout_index = 1;

  if(query_link == -1)
    request_time = 0;
    
  request_time++; // multiple of timeout

  if(request_time > (timeout_index + 10)) // if the message has been received or timeout accured
  {
    // If timeout accurred increment timeout flag up to 100
    if(arm_link[query_link - 1].request_timeout < 100)
    {
      arm_link[query_link - 1].request_timeout++;
      //printf("Link %i Timout number %i\n", query_link, arm_link[query_link - 1].request_timeout);
    }
    
    // check if is a motor or the actuator
    if(query_link < MOTOR_NUMBER)
      arm_link[query_link - 1].request_actual_position = 0;
    else
      arm_link[query_link - 1].request_trajectory_status = 0;

    // if I obtain too much timeout then consider motor off
    if(arm_link[query_link - 1].request_timeout  > 10)
      arm_link[query_link - 1].trajectory_status = 0;
    
    request_time = 0;
    query_link = -1;
  }
  
  if(request_time == timeout_index)
  {
    if(query_link == -1)
    {
      //if no new command then off
      selected_link = jse.button[0] + (jse.button[1] << 1) + (jse.button[2] << 2);

      switch(selected_link)
      {
        case 1:
          if(query_link == -1)
          {
            last_link_queried++;
            if(last_link_queried > 3)
              last_link_queried = 1;
          }

          arm_set_command(1, "VT", ((float)jse.stick_x / joy_max_value) * arm_link[0].velocity_target_limit);
          arm_set_command(2, "VT", ((float)jse.stick_y / joy_max_value) * arm_link[1].velocity_target_limit);
          arm_set_command(3, "VT", ((float)jse.stick_z / joy_max_value) * arm_link[2].velocity_target_limit);
          arm_set_command_without_value(1, "G");
          arm_set_command_without_value(2, "G");
          arm_set_command_without_value(3, "G");
          arm_set_command(0, "c", 0);
          break;

        case 2:
          if(query_link == -1)
          {
            last_link_queried++;
            if((last_link_queried > 6) || (last_link_queried < 4))
              last_link_queried = 4;
          }

          arm_set_command(4, "VT", ((float)jse.stick_x / joy_max_value) * arm_link[3].velocity_target_limit);
          arm_set_command(5, "VT", ((float)jse.stick_y / joy_max_value) * arm_link[4].velocity_target_limit);
          arm_set_command(6, "VT", ((float)jse.stick_z / joy_max_value) * arm_link[5].velocity_target_limit);
          arm_set_command_without_value(4, "G");
          arm_set_command_without_value(5, "G");
          arm_set_command_without_value(6, "G");
          arm_set_command(0, "c", 0);
          break;

        case 4:
          if(query_link == -1)
            last_link_queried = 7;

          actuator_set_command(jse.stick_z);
          break;
  
        default:
          arm_set_command(0, "VT", 0);
          arm_set_command(0, "c", 0);
          break;
      } // end switch

      //Get position (Get_enc_pos with RPA)
      query_link = last_link_queried;
  
      // If it's a smartmotor
      if(query_link < MOTOR_NUMBER)
      {
        arm_set_command_without_value(query_link, "RPA");
        arm_link[query_link - 1].request_actual_position = 1;
      }
      else
      {
        actuator_request_trajectory();
        arm_link[query_link - 1].request_trajectory_status= 1;
      }
    }
    else
      printf("Request busy\n");
  }
    
  return bytes_sent;
}

int arm_move_xyz(double *x, double *y, double *z, struct wwvi_js_event jse, __u16 joy_max_value)
{
  int bytes_sent = 0;
  char selected_link;
  static int last_link_queried = 0;
  double tetha0, tetha1, tetha2;
  const int timeout_index = 1;
  static int request_time = 0;
  static int automatic_query = 0;
  static double link_4_actual_position = M_PI_2;
  static double link_5_actual_position = M_PI_2;
  
  if(query_link == -1)
    request_time = 0;

  request_time++; // multiple of timeout
  
  if(request_time > (timeout_index + 10)) // if the message has been received or timeout accured
  {
    printf("---------------->timeout arm_move_xyz on %i\n", query_link);
    //exit(1);
    // If timeout accurred increment timeout flag up to 100
    if(arm_link[query_link - 1].request_timeout < 100)
    {
      arm_link[query_link - 1].request_timeout++;
      //printf("Link %i Timout number %i\n", query_link, arm_link[query_link - 1].request_timeout);
    }
    
    // check if is a motor or the actuator
    if(query_link < MOTOR_NUMBER)
      arm_link[query_link - 1].request_actual_position = 0;
    else
      arm_link[query_link - 1].request_trajectory_status = 0;

    // if I obtain too much timeout then consider motor off
    if(arm_link[query_link - 1].request_timeout  > 10)
      arm_link[query_link - 1].trajectory_status = 0;
    
    request_time = 0;
    query_link = -1;
  }
  
  if(request_time == timeout_index)
  {
    if(query_link == -1)
    {
      tetha0 = arm_link[0].actual_position * M_PI / (180 * arm_encoder_factor * arm_link[0].gear);
      //if no new command then off
      selected_link = jse.button[0] + (jse.button[1] << 1) + (jse.button[2] << 2);

      switch(selected_link)
      {
        case 1:
          if(query_link == -1)
          {
            last_link_queried++;
            if(last_link_queried == 4)
              last_link_queried = 5;
            else if(last_link_queried > 6)
              last_link_queried = 1;
          }

          *y += (float)jse.stick_y / (joy_max_value * 5000);
          *z += (float)jse.stick_z / (joy_max_value * 5000);
      
          arm_ik_ang(*x, *y, *z, &tetha0, &tetha1, &tetha2);

          //printf("x: %f y: %f z: %f tetha0: %f tetha1: %f tetha2: %f\n", *x, *y, *z, tetha0, tetha1, tetha2);
          if(wrist_position_mode != 1)
          {
            arm_start_xyz();
            
            arm_set_command_without_value(1, "MV");
    
            wrist_position_mode = 1;
            link_4_actual_position = arm_link[4].actual_position * M_PI / (180 * arm_encoder_factor * arm_link[4].gear) - tetha1 - tetha2;
            link_5_actual_position = arm_link[5].actual_position * M_PI / (180 * arm_encoder_factor * arm_link[5].gear) - tetha0;
            break;
          }

          //printf("Send VT Command\n");
          arm_set_command(1, "VT", ((float)jse.stick_x / joy_max_value) * arm_link[0].velocity_target_limit);
          arm_set_command(2, "PT", (long)(tetha1 * 180 * arm_encoder_factor * arm_link[1].gear / M_PI));
          arm_set_command(3, "PT", (long)(tetha2 * 180 * arm_encoder_factor * arm_link[2].gear / M_PI));
          arm_set_command(5, "PT", (long)((link_4_actual_position + tetha1 + tetha2) * 180 * arm_encoder_factor * arm_link[4].gear / M_PI));
          arm_set_command(6, "PT", (long)((link_5_actual_position + tetha0) * 180 * arm_encoder_factor * arm_link[5].gear / M_PI));

          //printf("2 %ld 3 %ld\n", (long)(tetha1 * 180 * arm_encoder_factor * arm_link[1].gear / M_PI), (long)(tetha2 * 180 * arm_encoder_factor * arm_link[2].gear / M_PI));
          arm_set_command_without_value(0, "G");
          arm_set_command(0, "c", 0);
          break;

        case 2:
          if(query_link == -1)
          {
            last_link_queried++;
            if((last_link_queried > 6) || (last_link_queried < 4))
              last_link_queried = 4;
          }
     
          if(wrist_position_mode != 0)
          {
            arm_set_command(0, "VT", 0);
            arm_set_command_without_value(4, "MV");
            arm_set_command_without_value(5, "MV");
            arm_set_command_without_value(6, "MV");
            //if(arm_set_command_without_value(0, "F") <= 0) // active settings
            //  return -1;
          
            wrist_position_mode = 0;
          }

          //printf("Send VT Command\n");
          arm_set_command(4, "VT", ((float)jse.stick_x / joy_max_value) * arm_link[3].velocity_target_limit);
          arm_set_command(5, "VT", -((float)jse.stick_y / joy_max_value) * arm_link[4].velocity_target_limit);
          arm_set_command(6, "VT", ((float)jse.stick_z / joy_max_value) * arm_link[5].velocity_target_limit);

          arm_set_command_without_value(4, "G");
          arm_set_command_without_value(5, "G");
          arm_set_command_without_value(6, "G");
          arm_set_command(0, "c", 0);
          break;

        case 4:
          if(query_link == -1)
            last_link_queried = 7;

          actuator_set_command(jse.stick_z);
          break;
  
        default:
          arm_set_command(0, "VT", 0);
          arm_set_command_without_value(0, "G");
          arm_set_command(0, "c", 0);
          break;
      } // end switch
  

      //Get position (Get_enc_pos with RPA)
      if(last_link_queried > 0)
        query_link = last_link_queried;
      else
      {
        automatic_query++;
        
        if(automatic_query > MOTOR_NUMBER)
          automatic_query = 1;
          
        query_link = automatic_query;
      }
      
      // If it's a smartmotor
      if(query_link < MOTOR_NUMBER)
      {
        //printf("Send RPA command at %i\n", query_link);
        arm_set_command_without_value(query_link, "RPA");
        arm_link[query_link - 1].request_actual_position = 1;
      }
      else
      {
        //printf("Send RPA command at actuator\n");
        actuator_request_trajectory();
        arm_link[query_link - 1].request_trajectory_status= 1;
      }
      
      //printf("Start Timer\n");
    }
    else
    {
      //printf("Timer back\n");
      return 0;
    }
  }

  //arm_query_position(last_link_queried);
  return bytes_sent;
}

/* Get link's index where to send a query and start to count for 
   timeout. This function must to be called at the same loop's frequency
   otherwise the timeout variable can't be incremented.*/
int arm_query_position(int link_to_query)
{
  const int timeout_index = 1;
  static int request_time = 0;
  static int automatic_query = 0;
  
  if(query_link == -1)
    request_time = 0;
    
  request_time++; // multiple of timeout
  
  if(request_time == timeout_index)
  {
    if(query_link == -1)
    {
      if(link_to_query > 0)
        query_link = link_to_query;
      else
      {
        automatic_query++;
        
        if(automatic_query > MOTOR_NUMBER)
          automatic_query = 1;
          
        query_link = automatic_query;
      }
      
      // If it's a smartmotor
      if(query_link < MOTOR_NUMBER)
      {
        arm_set_command_without_value(query_link, "RPA");
        arm_link[query_link - 1].request_actual_position = 1;
      }
      else
      {
        actuator_request_trajectory();
        arm_link[query_link - 1].request_trajectory_status= 1;
      }
    }
    else
      return 0;
  }
  else if(request_time > (timeout_index + 10)) // if the message has been received or timeout accured
  {
    printf("Link %i timeout\n", query_link);
    // If timeout accurred increment timeout flag up to 100
    if(arm_link[query_link - 1].request_timeout < 100)
    {
      arm_link[query_link - 1].request_timeout++;
      //printf("Link %i Timout number %i\n", query_link, arm_link[query_link - 1].request_timeout);
    }
    
    // check if is a motor or the actuator
    if(query_link < MOTOR_NUMBER)
      arm_link[query_link - 1].request_actual_position = 0;
    else
      arm_link[query_link - 1].request_trajectory_status = 0;

    // if I obtain too much timeout then consider motor off
    if(arm_link[query_link - 1].request_timeout  > 10)
      arm_link[query_link - 1].trajectory_status = 0;
    
    request_time = 0;
    query_link = -1;
  }
    
  return 1;
}

/* Read position target from file and call arm_start routine */
int arm_automatic_motion_velocity_start(char *motion_file)
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
}

/* Read position target from file and call arm_start routine */
int arm_automatic_motion_xyz_start(char *motion_file)
{
  int i;
  int return_value;
  double motor_position_target[MOTOR_NUMBER];
  
  link_homing_complete = 0;
  
  if(motion_file != NULL)
  {
    motion_file_cursor_position = 0;
    strcpy(current_motion_file, motion_file);
  }
  
  return_value = arm_read_path_xyz(current_motion_file, motor_position_target, &motion_file_cursor_position);
  
  if(motion_file != NULL)
    arm_start_xyz();
    
  // if collect an error load 
  if(return_value == -1)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      arm_link[i].position_target = arm_link[i].actual_position;
      
      if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0) //set velocity target
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
      
      if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0) //set velocity target
        return -1;
    }
    
    arm_auto_motion_xyz_mode = 0;
    return 0;
  }
  else if(return_value == 4)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
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
    
    arm_ik_ang(0, motor_position_target[1], motor_position_target[2], 
               &motor_position_target[0], &motor_position_target[1], &motor_position_target[2]);

    for(i = 0; i < 3; i++)
      arm_link[i].position_target = (long)(motor_position_target[i] * 180 * arm_encoder_factor * arm_link[i].gear / M_PI);

    arm_link[3].position_target = 0;
    arm_link[4].position_target = (long)((double)90 * arm_encoder_factor + (double)arm_link[1].actual_position / arm_link[1].gear + (double)arm_link[2].actual_position / arm_link[2].gear) * arm_link[4].gear;
    arm_link[5].position_target = (long)((double)90 * arm_encoder_factor + (double)arm_link[0].actual_position / arm_link[0].gear) * arm_link[5].gear;
    arm_link[6].position_target = (long)(motor_position_target[3]);
    
    if(arm_set_command(1, "PT", arm_link[0].position_target) <= 0) //set velocity target
     return -1;
    
    if(arm_set_command(2, "PT", arm_link[1].position_target) <= 0) //set velocity target
      return -1;
    
    if(arm_set_command(3, "PT", arm_link[2].position_target) <= 0) //set velocity target
      return -1;
    
    if(arm_set_command(4, "PT", arm_link[3].position_target) <= 0) //set velocity target
      return -1;
    
    if(arm_set_command(5, "PT", arm_link[4].position_target) <= 0) //set velocity target
      return -1;
    
    if(arm_set_command(6, "PT", arm_link[5].position_target) <= 0) //set velocity target
      return -1;
    
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
      if(motor_position_target[i] != motor_position_target[i])
      {
        printf("Nan number %i\n", i);
        arm_link[i].position_target = arm_link[i].actual_position;
      }
      else
        arm_link[i].position_target = (long)motor_position_target[i];
      
      if(arm_set_command(i + 1, "PT", arm_link[i].position_target) <= 0) //set velocity target
       return -1;
       
      //printf("Target %i: %ld\n", i, arm_link[i].position_target);
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
 
  return 1;
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
int arm_automatic_motion_velocity_update()
{
  static int last_link_queried = 0;
  static int last_link_timeout = 0;
  long arm_direction = 0;
  int link_count;

  // Update link number
  if(query_link == -1)
  {
    /* Update link command */
    if(last_link_queried != 0)
    {
      if(last_link_timeout == arm_link[last_link_queried - 1].request_timeout)
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
    }

    /* Check if the movement has been completed */
    if(link_homing_complete == (pow(2, MOTOR_NUMBER) - 1))
    {
      // reset timeout flags
      for(link_count = 0; link_count < MOTOR_NUMBER; link_count++)
        arm_link[link_count].request_timeout = 0;

     // read the next target. If there's not other one, then send homing complete message
     if(arm_automatic_motion_velocity_start(NULL) < 1)
       return 1;
    }
  
    last_link_queried++;
    if(last_link_queried > MOTOR_NUMBER)
      last_link_queried = 1;

    // Check if the link is active, otherwise stop motion
    if(arm_link[last_link_queried - 1].request_timeout > 10)
    {
      link_homing_complete = (int)(pow(2, MOTOR_NUMBER) - 1);
      printf("Timeout accurred for link %i\n", last_link_queried);
    }
      
    // if the homing for a link has been completed, then pass to the next link 
    while(link_homing_complete & (int)pow(2, last_link_queried - 1))
    {
      // Checking for homing complete again. This is must be done due to the change in link_homing_complete
      // by timeout condition
      if(link_homing_complete == (pow(2, MOTOR_NUMBER) - 1))
      {
        // reset timeout flags
        for(link_count = 0; link_count < MOTOR_NUMBER; link_count++)
          arm_link[link_count].request_timeout = 0;

        // read the next target. If there's not other one, then send homing complete message
        if(arm_automatic_motion_velocity_start(NULL) < 1)
          return 1;
      }
        
      last_link_queried++;
      if(last_link_queried > MOTOR_NUMBER)
        last_link_queried = 1;

      // Check if the link is active
      if(arm_link[last_link_queried - 1].request_timeout > 10)
      {
        link_homing_complete = (int)(pow(2, MOTOR_NUMBER) - 1);
        printf("Timeout accurred for link %i\n", last_link_queried);
      }
    }
  }
  
  //printf("Query link %i, homing complete: %i\n", last_link_queried, link_homing_complete);
  // Get the timeout value for the link to known if a timeout accurred for the next request
  last_link_timeout = arm_link[last_link_queried - 1].request_timeout;
  arm_query_position(last_link_queried);
  
  return 0;
}

/* At every call this function check if a motor have been arrived to
   position. If it is not the case, send a position request to update
   the position. This function must to be called at the same loop's frequency
   otherwise the timeout variable can't be incremented.
   Function arm_automatic_motion_xyz_start must be called once before this one.
   */
int arm_automatic_motion_xyz_update()
{
  static int last_link_queried = 0;
  static int last_link_timeout = 0;
  int link_count;

  // Update link number
  if(query_link == -1)
  {
    /* Update link command */
    if(last_link_queried != 0)
    {
      /* check if the last link goes in timeout */
      if(last_link_timeout == arm_link[last_link_queried - 1].request_timeout)
      {
        //printf("velocity target for %i: %ld, velocity_target_limit: %ld\n",last_link_queried, arm_link[last_link_queried - 1].velocity_target, arm_link[last_link_queried - 1].velocity_target_limit);
        if((arm_link[last_link_queried - 1].actual_position < (arm_link[last_link_queried -1].position_target + (long)(arm_encoder_factor * arm_link[last_link_queried -1].gear / 4))) && 
           (arm_link[last_link_queried - 1].actual_position > (arm_link[last_link_queried -1].position_target - (long)(arm_encoder_factor * arm_link[last_link_queried -1].gear / 4))) &&
           ((link_homing_complete & (int)pow(2, last_link_queried - 1)) == 0))
        {
                
          //printf("Motor%i Actual position: %ld, Position target range: %ld\n", last_link_queried, arm_link[last_link_queried - 1].actual_position, arm_link[last_link_queried -1].position_target + 400);
          if(arm_stop(last_link_queried))
          {         
            //if I'm in arm_auto_motion_xyz_mode link 4 and 5 stop with link 0, 1 and 2
            if(arm_auto_motion_xyz_mode == 0)
              link_homing_complete |= (int)pow(2, last_link_queried - 1);
            else if(last_link_queried == 1)
            {
              if(arm_stop(6))
              {
                link_homing_complete |= (int)pow(2, last_link_queried - 1);  
                link_homing_complete |= (int)pow(2, 5);
              }
            }
            else if(last_link_queried == 3)
            {
              if(link_homing_complete & (int)pow(2, 1) && arm_stop(5))
              {
                link_homing_complete |= (int)pow(2, last_link_queried - 1);  
                link_homing_complete |= (int)pow(2, 4);
              }
            }
            else
              link_homing_complete |= (int)pow(2, last_link_queried - 1); 
          }
        } 
        else
        {
          if(arm_auto_motion_xyz_mode)
          {
            if(last_link_queried == 1)
            {
              arm_link[5].position_target = (long)((double)90 * arm_encoder_factor + (double)arm_link[0].actual_position / arm_link[0].gear) * arm_link[5].gear;
              
              if(arm_set_command(6, "PT", arm_link[5].position_target) <= 0) //set velocity target
                return -1;
 
              arm_set_command_without_value(6, "G");
            }
            else if((last_link_queried == 2) || (last_link_queried == 3))
            {
              arm_link[4].position_target = (long)((double)90 * arm_encoder_factor + (double)arm_link[1].actual_position / arm_link[1].gear + (double)arm_link[2].actual_position / arm_link[2].gear) * arm_link[4].gear;
             
              if(arm_set_command(5, "PT", arm_link[4].position_target) <= 0) //set velocity target
                return -1;
        
              arm_set_command_without_value(4, "G");
              arm_set_command_without_value(5, "G");
            }
          }
        }        
      }
    }

    /* Check if the movement has been completed */
    if(link_homing_complete == (pow(2, MOTOR_NUMBER) - 1))
    {
      // reset timeout flags
      for(link_count = 0; link_count < MOTOR_NUMBER; link_count++)
        arm_link[link_count].request_timeout = 0;

     // read the next target. If there's not other one, then send homing complete message
     if(arm_automatic_motion_xyz_start(NULL) < 1)
       return 1;
     else
       link_homing_complete = 0;
    }
  
    last_link_queried++;
    
    if(last_link_queried > MOTOR_NUMBER)
      last_link_queried = 1;

    // Check if the link is active, otherwise stop motion
    if(arm_link[last_link_queried - 1].request_timeout > 10)
    {
      link_homing_complete = (int)(pow(2, MOTOR_NUMBER) - 1);
      printf("Timeout accurred for link %i\n", last_link_queried);
    }

    // if the homing for a link has been completed, then pass to the next link 
    while(link_homing_complete & (int)pow(2, last_link_queried - 1))
    {
      // Checking for homing complete again. This is must be done due to the change in link_homing_complete
      // by timeout condition
      if(link_homing_complete == (pow(2, MOTOR_NUMBER) - 1))
      {
        // reset timeout flags
        for(link_count = 0; link_count < MOTOR_NUMBER; link_count++)
          arm_link[link_count].request_timeout = 0;

        // read the next target. If there's not other one, then send homing complete message
        if(arm_automatic_motion_xyz_start(NULL) < 1)
          return 1;
      }
        
      last_link_queried++;
      if(last_link_queried > MOTOR_NUMBER)
        last_link_queried = 1;

      // Check if the link is active
      if(arm_link[last_link_queried - 1].request_timeout > 10)
      {
        link_homing_complete = (int)(pow(2, MOTOR_NUMBER) - 1);
        printf("Timeout accurred for link %i\n", last_link_queried);
      }
    }
    
    //printf("Link selected: %i\n", last_link_queried);
  }

  //printf("Query link %i, homing complete: %i\n", last_link_queried, link_homing_complete);
  // Get the timeout value for the link to known if a timeout accurred for the next request
  last_link_timeout = arm_link[last_link_queried - 1].request_timeout;
  arm_query_position(last_link_queried);
  
  return 0;
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

int arm_read_path_xyz(const char *file_path, double *motor_position_target, int *cursor_position)
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

void arm_message_log(const char *scope, const char *message) 
{
  char buffer[32];
  struct tm *ts;
  size_t last;
  time_t timestamp = time(NULL);
  FILE *file = NULL;

  // Init Log File
  file = fopen(LOG_FILE, "a");
 
  if(!file)
  {
    perror("logfile fopen:");
    return;
  }

  ts = localtime(&timestamp);
  last = strftime(buffer, 32, "%b %d %T", ts);
  buffer[last] = '\0';

  fprintf(file, "[%s]%s: %s\n", buffer, scope, message);

  fclose(file);
}

void arm_ik_ang(double pw_x, double pw_y, double pw_z, double *Teta1, double *Teta2, double *Teta3)
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

void arm_ee_xyz(double *pw_x, double *pw_y, double *pw_z)
{
  double a2;
  double a3;

  a2 = 0.5;
  a3 = 0.512;

  double q[3];
 
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
    success = 0;

  return success;
}
