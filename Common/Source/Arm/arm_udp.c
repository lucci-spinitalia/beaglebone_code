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

void arm_calc_xyz(double *wrist_x, double *wrist_y, double *wrist_z, double *x, double *y, double *z)
{
  double c0, c1, c3, c4, s0, s1, s3, s4, c12, s12;
  double tetha0, tetha1, tetha2, tetha3, tetha4;
  const unsigned char arm_encoder_factor = 11; // = (4000/360) each motor has an encorder with 4000 step
  
  // Link position in rad
  tetha0 = arm_link[0].actual_position * 3.14 / (arm_encoder_factor * arm_link[0].gear * 180);
  tetha1 = arm_link[1].actual_position * 3.14 / (arm_encoder_factor * arm_link[1].gear * 180);
  tetha2 = arm_link[2].actual_position * 3.14 / (arm_encoder_factor * arm_link[2].gear * 180);
  tetha3 = arm_link[3].actual_position * 3.14 / (arm_encoder_factor * arm_link[3].gear * 180);
  tetha4 = arm_link[4].actual_position * 3.14 / (arm_encoder_factor * arm_link[4].gear * 180);
  
  /* Spheric wrist 
   *     x = p_x cos(t4) cos(t5) - p_y sen(t4) + p_z cos(t4) sen(t5) + l6 cos(t4) sen(t5) - l5 sen(t4)
   *     y = p_x sen(t4) cos(t5) + p_y cos(t4) + p_z sen(t4) sen(t5) + l6 sen(t4) sen(t5) - l5 cos(t4)
   *     z = -p_x sen(t5) + p_z cos(t5) + l6 cos(t5)
   * 
   * I want the origin, so p = {0, 0, 0}
   */
  c3 = cos(tetha3);
  c4 = cos(tetha4);
  s3 = sin(tetha3);
  s4 = sin(tetha4);
  
  *wrist_x = arm_link[5].link_length * c3 * s4 - arm_link[4].link_length * s3;
  *wrist_y = arm_link[5].link_length * s3 * s4 - arm_link[4].link_length * c3;
  *wrist_z = arm_link[5].link_length * c4;
  
  /* Anthropomorphic arm
   * To calculate current position when p = {0, 0, 0} use this formula:
   *     x = l2 cos(t1) cos(t2) + l3 cos(t1) cos(t2 + t3)
   *     y = l2 sen(t1) cos(t2) + l3 sen(t1) cos(t2 + t3)
   *     z = l2 sen(t2) + l3 sen(t2 + t3) + l1
   * */
  
  /* To calculate current position when p != {0, 0, 0} use this formula:
   *     x = p_x cos(t1) cos(t2 + t3)- p_y cos(t1) sen(t2 + t3) + p_z sen(t1) + l2 cos(t1) cos(t2) + l3 cos(t1) cos(t2 + t3)
   *     y = p_x sen(t1) cos(t2 + t3) - p_y sen(t1) sen(t2 + t3) - p_z cos(t1) + l2 sen(t1) cos(t2) + l3 sen(t1) cos(t2 + t3)
   *     z = p_x sen(t2 + t3) + p_y cos(t2 + t3) + l2 sen(t2) + l3 sen(t2 + t3) + l1
   * */
  
  c0 = cos(tetha0);
  c1 = cos(tetha1);
  s0 = sin(tetha0);
  s1 = sin(tetha1);
  c12 = cos(tetha1 + tetha2);
  s12 = sin(tetha1 + tetha2);
  
  *x = *wrist_x * c0 * c12 - *wrist_y * c0 * s12 - *wrist_z * s1 + arm_link[1].link_length * c0 * c1 + arm_link[2].link_length * c0 * c12;
  *y = *wrist_x * s0 * c12 - *wrist_y * s0 * s12 - *wrist_z * c1 + arm_link[1].link_length * s0 * c1 + arm_link[2].link_length * s0 * c12;
  *z = *wrist_x * s12 + *wrist_y * c12 + arm_link[1].link_length * s1 + arm_link[2].link_length * s12;
}

int arm_calc_tetha(double new_x, double new_y, double new_z, long *tetha0, long *tetha1, long *tetha2, long *tetha3, long *tetha4)
{
  double new_wrist_x, new_wrist_y, new_wrist_z;
  double c0, c2, c3, s0, s2, s3, c12, s12, tetha_cos1, tetha_sin1, tetha_cos2, tetha_sin2, tetha1_den;
  const unsigned char arm_encoder_factor = 11; // = (4000/360) each motor has an encorder with 4000 step
  
  // Link position in rad
  *tetha0 = arm_link[0].actual_position * 3.14 / (arm_encoder_factor * arm_link[0].gear * 180);
  *tetha1 = arm_link[1].actual_position * 3.14 / (arm_encoder_factor * arm_link[1].gear * 180);
  *tetha2 = arm_link[2].actual_position * 3.14 / (arm_encoder_factor * arm_link[2].gear * 180);
  *tetha3 = arm_link[3].actual_position * 3.14 / (arm_encoder_factor * arm_link[3].gear * 180);
  
  // Calculate coordinates in wrist coordinate system with center into the wrist center point
  c0 = cos(*tetha0);
  s0 = sin(*tetha0);
  c2 = cos(*tetha2);
  s2 = sin(*tetha2);
  c3 = cos(*tetha3);
  s3 = sin(*tetha3);
  c12 = cos(*tetha1 + *tetha2);
  s12 = sin(*tetha1 + *tetha2);
  
  new_wrist_x = new_x * c0 * c12 + new_y * s0 * s12 + new_z * s12 - arm_link[1].link_length * c3 - arm_link[2].link_length;
  new_wrist_y = -new_x * c0 * s12 - new_y * s0 * s12 + new_z * c12 - arm_link[1].link_length * s3;
  new_wrist_z = new_x * s0 - new_y * c0;
  
  printf("Wrist X: %f Wrist Y: %f Wrist Z: %f\n", new_wrist_x, new_wrist_y, new_wrist_z);
  
  if(wrist_calc_tetha(new_wrist_x, new_wrist_y, new_wrist_z, tetha3, tetha4) == 0)
  {
    // If the new point is out of the circle with radius equal to the arm's length, then return 0
    if((pow(new_x, 2) + pow(new_y, 2) + pow(new_z, 2) - pow(arm_link[1].link_length + arm_link[2].link_length + arm_link[3].link_length + arm_link[5].link_length, 2)) > 0)
    {
      tetha0 = NULL;
      tetha1 = NULL;
      tetha2 = NULL;
      return 0;
    }
    
    // Calculate inverse cinematic for anthropomorphic arm
    // There are 4 solution, but I want only one configuration
    *tetha0 = atan2(new_x, new_y) * 180 * arm_encoder_factor * arm_link[0].gear / 3.14; // in step;
    printf("Tetha0: %f\n", *tetha0 * 3.14 / (180 * arm_encoder_factor * arm_link[0].gear));
    
    tetha_cos2 = (pow(new_x, 2) + pow(new_y, 2) + pow(new_z, 2) - pow(arm_link[1].link_length, 2) - pow(arm_link[2].link_length, 2)) / (2 * arm_link[1].link_length * arm_link[2].link_length);
    tetha_sin2 = -sqrt(1 - pow(tetha_cos2, 2));
    
    *tetha2 = atan2(tetha_sin2, tetha_cos2) * 180 * arm_encoder_factor * arm_link[0].gear / 3.14; // in step;
    printf("Tetha2: %f\n", *tetha2 * 3.14 / (180 * arm_encoder_factor * arm_link[0].gear));
    
    tetha1_den = (c0 * (pow(arm_link[1].link_length, 2) + pow(arm_link[2].link_length, 2) + 2 * c2 * arm_link[1].link_length * arm_link[2].link_length * c2));
    tetha_cos1 = (sqrt(pow(new_x, 2) + pow(new_y, 2)) * (c2 * arm_link[2].link_length + arm_link[1].link_length) - new_z * s2 * arm_link[2].link_length) / tetha1_den;
    tetha_sin1 = (new_x * (c2 * arm_link[2].link_length + arm_link[1].link_length) - sqrt(pow(new_x, 2) + pow(new_y, 2)) * arm_link[2].link_length * s2) / tetha1_den;
    
    *tetha1 = atan2(tetha_sin1, tetha_cos1) * 180 * arm_encoder_factor * arm_link[0].gear / 3.14; // in step;
    printf("Tetha1: %f\n", *tetha1 * 3.14 / (180 * arm_encoder_factor * arm_link[0].gear));
  }
  
  return 1;
}

int wrist_calc_tetha(double new_wrist_x, double new_wrist_y, double new_wrist_z, long *tetha3, long *tetha4)
{
  const unsigned char arm_encoder_factor = 11; // = (4000/360) each motor has an encorder with 4000 step
  
  static double tetha4_prev_rad = 0;
  static double tetha3_prev_rad = 0;
  
  double wrist_limit, sphere;
  double t, t_2, c3, s3;
  double l4_2 = pow(arm_link[4].link_length, 2);
  double l4_x_sin2;
  
  wrist_limit = pow(new_wrist_x, 2) + pow(new_wrist_y, 2) - l4_2;
  sphere = wrist_limit + pow(new_wrist_z, 2);

  // Check if the wrist can arrive to the point (it must be between two circle with radius' length equal to link5 +- 50 mm
  //if((sqrt(sphere + l4_2) > (arm_link[5].link_length - 50)) && (sqrt(sphere + l4_2) < (arm_link[5].link_length + 50)))
  if((sphere > pow(arm_link[5].link_length - 100, 2)) && (sphere < pow(arm_link[5].link_length, 2)))
  {
    
    /*if((new_wrist_z == 0) && ((tetha3_prev_rad >= 0) && (tetha3_prev_rad < M_PI)))
    {
      tetha3_prev_rad = M_PI_2;
      
      tetha4_prev_rad = atan2(new_wrist_y / arm_link[5].link_length, new_wrist_x / arm_link[5].link_length);
    }
    else if((new_wrist_z == 0) && ((tetha3_prev_rad < 0) && (tetha3_prev_rad > -M_PI)))
    {
      tetha3_prev_rad = -M_PI_2;
      
      tetha4_prev_rad = atan2(new_wrist_y / arm_link[5].link_length, new_wrist_x / arm_link[5].link_length);
    }
    else
    {
      printf("else\n");
      if(wrist_limit != 0)
      {
        tetha3_prev_rad = atan2(new_wrist_y / wrist_limit, new_wrist_x / wrist_limit);
	    printf("tetha3_prev_rad: %f\n", tetha3_prev_rad);
      }
      else
        tetha3_prev_rad = 0;
      
      // Calculate inverse cinematic for wrist
      // The wrist can arrive to the point by two position. Which one we choose depend on tetha4
      if((tetha4_prev_rad >= 0) && (tetha4_prev_rad < M_PI))
      {
        tetha4_prev_rad = atan2(new_wrist_z / sqrt(sphere), sqrt(wrist_limit / sphere));
      }
      else
      {
        //tetha4_prev_rad = atan2(sqrt(-wrist_limit / sphere), new_wrist_z / sqrt(sphere));
        tetha4_prev_rad = atan2(new_wrist_z / sqrt(sphere), sqrt(wrist_limit / sphere));
      }
    }*/

    printf("Tetha3: %f\n", tetha3_prev_rad);
    *tetha3 = tetha3_prev_rad * 180 * arm_encoder_factor * arm_link[3].gear / M_PI; // in step

    printf("tetha4_prev_rad: %f\n", tetha4_prev_rad);
    // if out of limits return 0 (+- 90)
    if((tetha4_prev_rad < -M_PI_2) || (tetha4_prev_rad > M_PI_2))
    {
      tetha4 = NULL;
      return 0;
    }
    
    *tetha4 = tetha4_prev_rad * 180 * arm_encoder_factor * arm_link[4].gear / M_PI; // in step
    printf("Tetha4: %f\n", *tetha4 * M_PI / (180 * arm_encoder_factor * arm_link[4].gear));
  }
  else
  {
    tetha3 = NULL;
    tetha4 = NULL;
    return 0;
  }
  
  return 1;
}

int arm_init(int index, long kp, long ki, long kl, long kd, long kv, long adt, long vt, long amps)
{
  int i;
  long gear[MOTOR_NUMBER];
  long link_length[MOTOR_NUMBER];
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

  cursor_position = 0;
  // Load length parameters
  /*if(arm_read_path(ARM_LINK_LENGTH_FILE, link_length, &cursor_position) > 0)
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
  }*/
  
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
      //printf("Link %i Timout number %i\n", query_link, arm_link[query_link - 1].request_timeout);
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