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

#include "arm_udp.h"
#include "joystick.h"

#define ARM_BUFFER_SIZE 1024

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */

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
volatile unsigned char arm_net_down = 1;

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

int arm_init(int index, long kp, long ki, long kd, long adt, long vt, long amps)
{
  int i;

  // Set motor parameter 
  if(arm_set_command(index, "KP", kp) <= 0)
    return -1;
	
  if(arm_set_command(index, "KI", ki) <= 0)
    return -1;
	
  if(arm_set_command(index, "KD", kd) <= 0)
    return -1;
	
  if(arm_set_command(index, "ADT", adt) <= 0)  // set acceleration and deceleration
    return -1;
	
  if(arm_set_command(0, "VT", 0) <= 0) //set velocity target
    return -1;

  if(index == 0)
  {
    for(i = 0; i < MOTOR_NUMBER; i++)
	  arm_link[i].velocity_target_limit = vt;
  }
  else
	arm_link[index].velocity_target_limit = vt;

	
  if(arm_set_command(index, "AMPS", amps) <= 0) // set pwm drive signal limit
    return -1;
	
  if(arm_set_command_without_value(index, "F") <= 0) // active settings
    return -1;
	
  return 1;
}

int arm_start(void)
{
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

int arm_stop(void)
{
  if(arm_set_command_without_value(0, "X") <= 0) //slow motion to stop
    return -1;
  
  // wait for stop
  
  if(arm_set_command_without_value(0, "OFF") <= 0) //stop servoing the motors
    return -1;
	
  return 1;
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
  
  return bytes_sent;
}

int arm_move(struct wwvi_js_event jse, __u16 joy_max_value)
{
  int bytes_sent = 0;
  char selected_link;
  static int request_time = 0;
  static int last_link_queried = 0;
  const int timeout_index = 1;
  
  //if no new command then off

  selected_link = jse.button[0] + (jse.button[1] << 1);
  
  /*   For i = 1 To CommInterface.NoOfMotors
         Frm_seriale.My_comm.Invia_comando "VT=" & CLng(Braccio.giunto(i).lVel * 10000), CByte(i)  'TESTING
         'Debug.Print i & " " & Braccio.giunto(i).lVel
   Next i*/
  request_time++;
	  
  if(request_time <= timeout_index) 
  {
    switch(selected_link)
    {
      case 1:
	  
	    if(query_link == -1)
		{
		  last_link_queried++;
		  if(last_link_queried > 3)
		    last_link_queried = 1;
			
		  query_link = last_link_queried;
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
		  if(last_link_queried > 6)
		    last_link_queried = 4;
			
		  query_link = last_link_queried;
		}
		
		arm_set_command(4, "VT", ((float)jse.stick_x / joy_max_value) * arm_link[3].velocity_target_limit);
	    arm_set_command(5, "VT", ((float)jse.stick_y / joy_max_value) * arm_link[4].velocity_target_limit);
	    arm_set_command(6, "VT", ((float)jse.stick_z / joy_max_value) * arm_link[5].velocity_target_limit);
	    arm_set_command_without_value(4, "G");
	    arm_set_command_without_value(5, "G");
	    arm_set_command_without_value(6, "G");
		arm_set_command(0, "c", 0);
        break;
		
	  default:
	    arm_set_command(0, "VT", 0);
		arm_set_command(0, "c", 0);
	    break;
    }
	
    if(request_time == timeout_index)
    {
      //Get position (Get_enc_pos with RPA)
      arm_set_command_without_value(query_link, "RPA");
    }
  }
  else if((request_time > (timeout_index + 2)) || (query_link == -1)) // if the message has been received or timeout accured
  {
    request_time = 0;
	query_link = -1;
  }
  
  return bytes_sent;
}
