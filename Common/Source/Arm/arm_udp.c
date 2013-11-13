// implementare waitforstop in arm_stop

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

  memcpy(&arm_buffer_tx[arm_buffer_tx_ptr_wr], &data, sizeof(struct arm_frame));

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
 
  memcpy(data, &arm_buffer_rx[arm_buffer_rx_ptr_rd], sizeof(struct arm_frame));

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

int arm_read(int device)
{
  int bytes_read = -1;

  if(arm_buffer_rx_full)
  {
    arm_buffer_rx_overrun = 1;
    return -1;
  }

  if(device > 0)
  {
    bytes_read = recvfrom(device, &arm_buffer_rx[arm_buffer_rx_ptr_wr], sizeof(struct arm_frame), 0, NULL, NULL);

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
  sprintf(buffer.frame, "%c%s%c%c", index + 128, command, 0x0d, 0);
  
  bytes_sent = arm_load_tx(buffer);

  return bytes_sent;
}

int arm_set_command(int index, char *command, long value)
{
  int bytes_sent;
  
  struct arm_frame buffer; 
  sprintf(buffer.frame, "%c%s=%ld%c%c", index + 128, command, value, 0x0d, 0);
  
  bytes_sent = arm_load_tx(buffer);
  
  //printf("%i%s=%ld\n", index + 128, command, value);
  return bytes_sent;
}

int actuator_request_trajectory()
{
  int bytes_sent;
  struct arm_frame buffer; 
  
  sprintf(buffer.frame, "$?%c%c", 0x0d, 0);
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
	
  switch(actuator_state)
  {
    case 0: //stop
	  //printf("ACTUATOR STOP\n");
      sprintf(buffer.frame, "$t%c%c", 0x0d, 0);
	  break;

	case 1: //open
	  //printf("ACTUATOR OPENING\n");
	  sprintf(buffer.frame, "$a%c%c", 0x0d, 0);
	  break;

	case 2: //close
      //printf("ACTUATOR CLOSING\n");
      sprintf(buffer.frame, "$c%c%c", 0x0d, 0);
	  break;
	   
	default:
      sprintf(buffer.frame, "$t%c%c", 0x0d, 0);
	  break;
  }
  
  bytes_sent = arm_load_tx(buffer);

  return bytes_sent;
	
}
int arm_init(int index, long kp, long ki, long kl, long kd, long kv, long adt, long vt, long amps)
{
  int i;
  long gear[MOTOR_NUMBER];
  int cursor_position = 0;

  // Load gear parameters
  if(arm_read_path(ARM_GEAR_FILE, gear, &cursor_position) > 0)
  {
    if(index == 0)
    {
      for(i = 0; i < MOTOR_NUMBER; i++)
        arm_link[i].gear = gear[i];
    }
    else
      arm_link[index - 1].gear = gear[index - 1];
  }
  else
    return -1;
  
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
  
  if(arm_set_command(index, "AMPS", amps) <= 0) // set pwm drive signal limit
    return -1;
	
  if(arm_set_command_without_value(index, "F") <= 0) // active settings
    return -1;
	
  return 1;
}

int arm_start(void)
{
  int i;
  
  // Init trajectory status
  for(i = 0; i < MOTOR_NUMBER; i++)
    arm_link[i].trajectory_status = 1;
  
  if(arm_set_command_without_value(0, "ZS") <= 0)	//Clear faults
    return -1;
	
  if(arm_set_command_without_value(0, "BRKSRV") <= 0)	//release brake only with servo active 
	return -1;
	
  if(arm_set_command_without_value(0, "MV") <= 0)	//set velocity mode 
    return -1;
	
  /*  For i = 1 To CommInterface.NoOfMotors
  Braccio.giunto(i).PosDES = Motor(i).GetPosition
  Next i
Dim q(3) As Double
Dim a2 As Double
Dim a3 As Double
  
a2 = Braccio.Length(2)
a3 = Braccio.Length(3)
q(2) = Braccio.giunto(2).PosDES * Braccio.giunto(2).StepToRAD
q(3) = Braccio.giunto(3).PosDES * Braccio.giunto(3).StepToRAD
 
  Braccio.py_des = a2 * Cos(q(2)) + a3 * Cos(q(3) + q(2))
  Braccio.pZ_des = a2 * Sin(q(2)) + a3 * Sin(q(2) + q(3))*/
  
  if(arm_set_command_without_value(0, "SLE") <= 0)  // Enable software limit  
    return -1;
	
  return 1;
}

int arm_stop(int index)
{
  static int last_link_queried = 0;
  static int request_time = 0;
  const int timeout_index = 1;
  int i;
  
  request_time++;

  if(index == 0)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      //printf("send stop command to all: %i %i\n", i, arm_link[i].trajectory_status);
      if(arm_link[i].trajectory_status > 0)
        break;

      if(i == (MOTOR_NUMBER - 1))
      {
        //printf("send stop command to all\n");
        if(arm_set_command_without_value(0, "OFF") <= 0) //stop servoing the motors
          return -1;  

        return 1;
      }
    }
  }
  else
  {
    if(arm_link[index - 1].trajectory_status == 0)
    {
      if(arm_set_command_without_value(index, "OFF") <= 0) //stop servoing the motors
        return -1;  

      return 1;
    }
  }

  // wait for stop
  if(request_time == timeout_index)
  {
    if(query_link == -1)
    {
      if(index == 0)
      {
        last_link_queried++;
        if(last_link_queried > MOTOR_NUMBER)
          last_link_queried = 1;

        query_link = last_link_queried;
      }
      else
        query_link = index;
      
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
        if(arm_set_command_without_value(index, "X") <= 0) //slow motion to stop
          return -1;
  
        arm_set_command_without_value(query_link, "RB(0,2)");
      }
    }
    else
      request_time--;  //enter again here if i can't send the message due query_link != -1
  }
  else if(request_time > (timeout_index + 3))
  {
    // If timeout accurred increment timeout flag up to 100
    if(arm_link[query_link - 1].request_timeout < 100)
    {
      arm_link[query_link - 1].request_timeout++;
      printf("Link %i Timout number %i\n", query_link, arm_link[query_link - 1].request_timeout);
    }
    
    arm_link[query_link - 1].request_trajectory_status = 0;
    
    // if I obtain too much timeout then consider motor off
    if(arm_link[query_link - 1].request_timeout  > 10)
      arm_link[query_link - 1].trajectory_status = 0;
    
    request_time = 0;
    query_link = -1;
  }
  else if(query_link == -1)
    request_time = 0;
  
  return 0;
}

int arm_move(struct wwvi_js_event jse, __u16 joy_max_value)
{
  int bytes_sent = 0;
  char selected_link;
  static int request_time = 0;
  static int last_link_queried = 0;
  const int timeout_index = 1;

  //if no new command then off
  selected_link = jse.button[0] + (jse.button[1] << 1) + (jse.button[2] << 2);

  /*   For i = 1 To CommInterface.NoOfMotors
         Frm_seriale.My_comm.Invia_comando "VT=" & CLng(Braccio.giunto(i).lVel * 10000), CByte(i)  'TESTING
         'Debug.Print i & " " & Braccio.giunto(i).lVel
   Next i*/
  request_time++; // multiple of timeout

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

  if(request_time == timeout_index)
  {
    if(query_link == -1)
    {
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
      request_time--;
  }
  else if(request_time > (timeout_index + 3)) // if the message has been received or timeout accured
  {
    // If timeout accurred increment timeout flag up to 100
    if(arm_link[query_link - 1].request_timeout < 100)
    {
      arm_link[query_link - 1].request_timeout++;
      printf("Link %i Timout number %i\n", query_link, arm_link[query_link - 1].request_timeout);
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
  else if(query_link == -1)
    request_time = 0;
    
  return bytes_sent;
}

int arm_automatic_motion_start(char *motion_file)
{
  int i;
  char return_value;
  long motor_position_target[MOTOR_NUMBER];
  
  link_homing_complete = 0;
  
  if(motion_file != NULL)
    strcpy(current_motion_file, motion_file);

  // Load homing position
  return_value = arm_read_path(current_motion_file, motor_position_target, &motion_file_cursor_position);
  
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

void arm_automatic_motion_abort()
{
  motion_file_cursor_position = 0;
}

int arm_homing_check()
{
  static int last_link_queried = 0;
  static int request_time = 0;
  const int timeout_index = 1;
  int link_count;
  
  if(link_homing_complete == (pow(2, MOTOR_NUMBER) - 1))
  {
    // reset timeout flags
    for(link_count = 0; link_count < MOTOR_NUMBER; link_count++)
      arm_link[link_count].request_timeout = 0;

    // read the next target. If there's not other one, then send homing complete message
    if(arm_automatic_motion_start(NULL) < 1)
      return 1;
  }

  request_time++;
  
  if(request_time == timeout_index)
  {
    // Update link number
    if(query_link == -1)
    {
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
          if(arm_automatic_motion_start(NULL) < 1)
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
      //printf("Query link %i, homing complete: %i\n", last_link_queried, link_homing_complete);
      query_link = last_link_queried;
	  
      if(query_link < MOTOR_NUMBER)
      {
        //Get position (Get_enc_pos with RPA)
        arm_link[query_link - 1].request_actual_position = 1;
        arm_set_command_without_value(query_link, "RPA");
      }
      else
      {
        arm_link[query_link - 1].request_trajectory_status = 1;
        actuator_request_trajectory();
      }
    }
  }
  else if(request_time > (timeout_index + 3)) // if the message has been received or timeout accured
  {
    // If timeout accurred increment timeout flag up to 100
    if(arm_link[query_link - 1].request_timeout < 100)
    {
      arm_link[query_link - 1].request_timeout++;
      printf("Link %i Timout number %i\n", query_link, arm_link[query_link - 1].request_timeout);
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
  else if(query_link == -1)
    request_time = 0;
  
  return 0;
}

int arm_read_path(const char *file_path, long *motor_position_target, int *cursor_position)
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
	return -1;
	  
  if((read = getline(&line, &len, file)) != -1)
  {
    //printf("Line read: %s\n", line);
	arm_message_log("arm_read_path", line);
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
    *cursor_position = 0;
	free(line);
	return 0;
  }
  
  free(line);
  fclose(file);
  return 1;
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