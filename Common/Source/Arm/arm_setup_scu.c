#include <sys/types.h> 
#include <sys/socket.h> 
#include <sys/ioctl.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <net/if.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h>
#include <limits.h>

#include <sys/time.h> 
#include <time.h> 
#include <errno.h>

#include <locale.h>

#include <string.h>
#include <math.h>

#include <netinet/in.h> 
#include <arpa/inet.h>

#include <signal.h>

#include <termios.h>

#include "arm_udp.h"
#include "arm_rs485.h"
#include "nmea/nmea.h"
#include "rs232.h"


/* Arm */
#define ARM_ADDRESS "192.168.1.28"
#define ARM_PORT 8012
#define ARM_DEVICE_FILE "/dev/ttyUSB0"
#define ARM_MAX_POSITION 0.1 // value in degree under which the link will be stopped

#define SCU_STATUS_PORT 8016
#define SCU_SEGWAY_FROM_JOYSTICK_PORT 8017


#define JOY_MAX_VALUE 32767 

#define TIMEOUT_SEC 0 
#define ARM_TIMEOUT_USEC 10000
#define AUTOMOTION_TIMEOUT_USEC 50000
#define STOP_TIMEOUT_USEC 50000
#define REQUEST_TIMEOUT_USEC 10000
#define ARM_POSITION_TIMEOUT_USEC 100000
#define ARM_BATTERY_TIMEOUT_SEC 1
#define INIT_POSITION_TIMEOUT_USEC 100000

#define LINK_TIMEOUT_LIMIT 100

#define ARM_JOINT_YZ_STEP_M 0.0009

// Standoff Control Unit scu_state
#define SCU_ARM_IDLE 0
#define SCU_ARM_REST 1
#define SCU_ARM_MOVE 2
#define SCU_ARM_STOP 3
#define SCU_ARM_OUT_OF_SERVICE 13
#define SCU_ARM_UNKNOWN 14

/* SCU Request */
#define SCU_RQST_STATE 0

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

#undef min
#define min(x,y) ((x) < (y) ? (x) : (y))

/* Prototype */
int eth_check_connection();
int gpio_export(int pin_number);
int gpio_set_value(int pin_number, int value);
int gpio_set_direction(int pin_number, int value);
int gpio_generic_set_value(char *path, int value);
void arm_status_update(unsigned char *arm_state, unsigned char *arm_next_state, unsigned char *arm_prev_state, struct arm_frame arm_message);

/* Status client interface*/
int scu_state_socket = -1;
unsigned char scu_state_rqst_flag = 0;

/* Robotic Arm Client*/
int arm_udp_device = -1;
int arm_rs485_device = -1;
unsigned char automove_timer_flag = 0;
unsigned char stop_timer_flag = 0;
int arm_query_link = -1;
  
/* Generic */
unsigned char show_arm_state_flag = 0;
unsigned char show_arm_position_flag = 0;
unsigned char show_arm_ayz_init_flag = 0;
unsigned char arm_out_of_service_enable_flag = 1;
unsigned char laser_enable_flag = 1;

float arm_tetha0, arm_y, arm_z;

void signal_handler(int signum)
{
  // Garbage collection
  printf("Terminating program...\n");
  
  close(scu_state_socket);
  close(arm_rs485_device);
  close(arm_udp_device);

  exit(signum);
}

int main(int argc, char **argv) 
{
  int argc_count;
  if(argc > 1)
  {
    for(argc_count = 1; argc_count < argc; argc_count++)
    {
      if(strcmp(argv[argc_count], "--show-state") == 0)
        show_arm_state_flag = 1;
      else if(strcmp(argv[argc_count], "--show-ayz") == 0)
        show_arm_position_flag = 1;
      else if(strcmp(argv[argc_count], "--no-out-of-service") == 0)
        arm_out_of_service_enable_flag = 0;
      else
      {
        printf("Usage:\n\t%s [option]\nOption:\n", argv[0]);
        printf("\t--show-state\tprint the arm state to screen\n");
        printf("\t--show-ayz\tprint current links position\n");
        printf("\t--no-out-of-service\tdisable OUT OF SERVICE state\n");
        exit(0);
      }
    }
  }
  
  /* Standoff Control Unit*/
  unsigned char scu_prev_state = SCU_ARM_IDLE; 
  unsigned char scu_next_state = SCU_ARM_IDLE; // it's used to return to the previouse scu_state after an automatic move is completed
  unsigned char scu_state = SCU_ARM_IDLE;
  
  if(show_arm_state_flag)
    printf("scu state\t[SCU_ARM_REST] as default in main\n");
  
  /* Robotic Arm */
  char arm_buffer[ARM_RS485_BUFFER_SIZE];
  long arm_number_convertion_result;
  char *arm_token_number;
  unsigned char request_timer_flag = 0;
  unsigned char arm_request_position = 0;
  unsigned char arm_request_trajectory = 0;
 
  
  /* Robotic Arm Client*/
  struct sockaddr_in arm_address;
  struct arm_frame arm_command;
  int query_link_count;
  char arm_position_initialized = 0;
  struct timespec request_timer_start, request_timer_stop;
  long request_elapsed_time_ns = 0;

  /* Status client interface*/
  socklen_t scu_state_addr_dest_len = sizeof(struct sockaddr_in);
  struct sockaddr_in scu_state_addr_dest;
  struct sockaddr_in scu_state_addr_src;
  unsigned char scu_state_buffer;

  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read
  int bytes_sent;

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  long stop_timeout_counter = 0;
  long arm_position_timeout_counter = 0;
  long init_position_timeout_counter = 0;
  long current_timeout = 0;
  
  printf("Initializing. . .\n");
    
  while(eth_check_connection() != 1)
    printf("Waiting for network. . .\n");
    
  /* Peripheral initialization */

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  /* Init Arm Client */  
  if(arm_open_server(&arm_udp_device, &arm_address, ARM_PORT)  == -1)
    perror("init arm client");
  else
  {
    arm_command.arm_command_param.crc_uint = 0;
    arm_crc_initialize();
    
    printf("Arm\t[connected]\n");
  }
  
  /* Init rs485 */
  arm_rs485_device = arm_rs485_open(ARM_DEVICE_FILE, 115200, 'N', 8, 1);

  if(arm_rs485_device == -1)
  {
    perror("arm_rs485_open");
    fflush(stdout);
  }
  else
  {
    printf("Init rs485\t[OK]\n");
    fflush(stdout);
    /* Init Robotic Arm */
    arm_init(0, 500, 10, 1500, 200, 1500, 100, 300, 1023);
    
    // tuning motor 2 and 3
    arm_set_max_velocity(2, 700);
    arm_set_max_velocity(3, 700);
    
    // tuning motor 5
    arm_set_max_velocity(5, 1400);
        
    if(arm_set_command(5, "KP", 1000) <= 0)
      return -1;
      
    if(arm_set_command(5, "KL", 32767) <= 0)
      return -1;
      
    if(arm_set_command(5, "KD", 2000) <= 0)
      return -1;
      
    if(arm_set_command_without_value(5, "F") <= 0) // active settings
      return -1;
      
    // tuning motor 6
    arm_set_max_velocity(6, 700);

    if(arm_set_command(6, "KP", 1000) <= 0)
      return -1;
      
    if(arm_set_command(6, "KL", 32767) <= 0)
      return -1;
      
    if(arm_set_command(6, "KD", 2000) <= 0)
      return -1;
      
    if(arm_set_command_without_value(6, "F") <= 0) // active settings
      return -1;
  }
      
  /* Status client interface */
  scu_state_socket = socket(AF_INET, SOCK_DGRAM, 0);

  if(scu_state_socket < 0)
    perror("scu_state_socket");
  else
  {
    bzero(&scu_state_addr_src, sizeof(scu_state_addr_src));
    scu_state_addr_src.sin_family = AF_INET;
    scu_state_addr_src.sin_port = htons(SCU_STATUS_PORT);
    scu_state_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(scu_state_socket, (struct sockaddr *)&scu_state_addr_src, sizeof(scu_state_addr_src)) == -1)
      perror("scu_state_socket");
    else
      printf("Status client\t[bind]\n");
    
    printf("Status client\t[listening]\n");
  }
	
  select_timeout.tv_sec = TIMEOUT_SEC;
  select_timeout.tv_usec = ARM_TIMEOUT_USEC;

  current_timeout = select_timeout.tv_usec;
  
  printf("Run main program. . .\n");
 
  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);
      
    
    if(request_timer_flag == 1)
    {
      clock_gettime(CLOCK_REALTIME, &request_timer_stop);
      request_elapsed_time_ns = (request_timer_stop.tv_sec * 1000000000 + request_timer_stop.tv_nsec) - (request_timer_start.tv_sec * 1000000000 + request_timer_start.tv_nsec);
    }
    
    if((scu_state_socket > 0))
    {
      if(scu_state_rqst_flag == 0)
      {
        FD_SET(scu_state_socket, &rd);
        nfds = max(nfds, scu_state_socket);
      }
      else if(arm_position_initialized == ((2 << (MOTOR_NUMBER - 1)) - 1))
      {
        FD_SET(scu_state_socket, &wr);
        nfds = max(nfds, scu_state_socket);
      }
    }
    
    if(arm_udp_device > 0)
    {
      FD_SET(arm_udp_device, &rd);
      nfds = max(nfds, arm_udp_device); 
    }
    
    if(arm_rs485_device > 0)
    {
      FD_SET(arm_rs485_device, &rd);
      nfds = max(nfds, arm_rs485_device); 
          
      if((arm_rs485_buffer_tx_empty == 0) && (request_timer_flag == 0))
      {
        FD_SET(arm_rs485_device, &wr);
        nfds = max(nfds, arm_rs485_device);
      }
    }
    else
    {  
      arm_rs485_device = arm_rs485_open(ARM_DEVICE_FILE, 115200, 'N', 8, 1);
      
      if(arm_rs485_device > 0)
      {
        printf("Init rs485\t[OK]\n");
        fflush(stdout);
        /* Init Robotic Arm */
        arm_init(0, 500, 10, 1500, 200, 1500, 100, 300, 1023);
    
        // tuning motor 2 and 3
        arm_set_max_velocity(2, 700);
        arm_set_max_velocity(3, 700);
    
        // tuning motor 5
        arm_set_max_velocity(5, 1400);
        
        if(arm_set_command(5, "KP", 1000) <= 0)
          return -1;
      
        if(arm_set_command(5, "KL", 32767) <= 0)
          return -1;
      
        if(arm_set_command(5, "KD", 2000) <= 0)
          return -1;
      
        if(arm_set_command_without_value(5, "F") <= 0) // active settings
          return -1;
      
        // tuning motor 6
        arm_set_max_velocity(6, 700);

        if(arm_set_command(6, "KP", 1000) <= 0)
          return -1;
      
        if(arm_set_command(6, "KL", 32767) <= 0)
          return -1;
        
        if(arm_set_command(6, "KD", 2000) <= 0)
          return -1;
      
        if(arm_set_command_without_value(6, "F") <= 0) // active settings
          return -1;
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

    /* Manage SCU state request */
    if(scu_state_socket > 0)
    {
      if(FD_ISSET(scu_state_socket, &rd))
      {
        bytes_read = recvfrom(scu_state_socket, &scu_state_buffer, sizeof(scu_state_buffer), 0, (struct sockaddr *)&scu_state_addr_dest, &scu_state_addr_dest_len);

        if(bytes_read > 0)
        {  
          switch(scu_state_buffer)
          {
            case SCU_RQST_STATE:
                scu_state_rqst_flag = 1;
              break;
          }
        }
        else    
          perror("status_read");
      }
      
      if(FD_ISSET(scu_state_socket, &wr))
      {
        scu_state_rqst_flag = 0;
        bytes_sent = sendto(scu_state_socket, &scu_state, sizeof(scu_state), 0, (struct sockaddr *)&scu_state_addr_dest, sizeof(scu_state_addr_dest));
      }
    }
    

    if(arm_udp_device > 0)
    {
      if(FD_ISSET(arm_udp_device, &rd))
      {
        bytes_read = recvfrom(arm_udp_device, &arm_command, sizeof(arm_command), 0, NULL, NULL);

        if(bytes_read > 0)
        {
          if(arm_crc_byte_buffer_crc_is_valid((__u8 *)arm_command.arm_command, bytes_read))
          {
            arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
          }          
        }
        else 
          perror("arm_read");
      }
    }
    
    if(arm_rs485_device > 0)
    {
      if(FD_ISSET(arm_rs485_device, &rd))
      {
        bytes_read = arm_rs485_read(arm_rs485_device);

        if(bytes_read == -1)
          perror("rs485 read");

        while((bytes_read = arm_rs485_unload_rx_filtered(arm_buffer, 13)) > 0)
        {
          /**************** Filter message ******************************/
           // I expect at least one character with \r as trailer
          if(bytes_read < 2)
          {
            printf("Message too short\n");
            continue;
          }
          
          arm_buffer[bytes_read] = 0;

          errno = 0;
          // convert sting in a long number
          arm_number_convertion_result = strtol(arm_buffer, &arm_token_number, 10);

          if((errno == ERANGE && (arm_number_convertion_result == LONG_MAX || arm_number_convertion_result == LONG_MIN)) || (errno != 0 && arm_number_convertion_result == 0)) 
          {
            //perror("strtol");
            printf("----------------------------------------> Not a number!!\n");

            //return 0;
            continue;
          }
          else if(arm_token_number == arm_buffer)
          {
            if((strncmp(arm_buffer, "$s0", 3) != 0) && (strncmp(arm_buffer, "$s1", 3) != 0) && (strncmp(arm_buffer, "$s2",3) != 0))
            {
              printf("Not a number at all!\n %s\n", arm_buffer);

              //return 0;
              continue;
            }
          }
          else if(*arm_token_number != '\r')
          {
            printf("----------------------------------------> Not a compleate number!!: \n%s\n", arm_buffer);
                 
            //return 0;
            continue;
          }

          if(arm_request_position == 1)
          {
            arm_link[arm_query_link - 1].actual_position = arm_number_convertion_result;

            if(arm_link[arm_query_link - 1].position_initialized == 0)
              arm_link[arm_query_link - 1].position_initialized = 1;

            arm_position_initialized |= (arm_link[arm_query_link - 1].position_initialized << (arm_query_link - 1));
            
            if(show_arm_position_flag)
            {
              if(show_arm_ayz_init_flag != 0)
              {
                printf("\033[%iA",MOTOR_NUMBER + 1);
              }
              else
                show_arm_ayz_init_flag = 1;
                
              arm_ee_xyz(&arm_tetha0, &arm_y, &arm_z);
              arm_tetha0 = arm_link[0].actual_position * M_PI / (180 * 11 * arm_link[0].gear);
              printf("Actual Positions - x: %f deg y: %f m z: %f m                \n", arm_tetha0, arm_y, arm_z);
              for(query_link_count = 0; query_link_count < MOTOR_NUMBER; query_link_count++)
              {
                printf("Link%i: %ld step %f deg        \n", query_link_count + 1, arm_link[query_link_count].actual_position, (double)arm_link[query_link_count].actual_position / (11 * arm_link[query_link_count].gear));
              }
            }
          }
          else if(arm_request_trajectory == 1)
          {
            arm_link[arm_query_link - 1].trajectory_status = arm_number_convertion_result;
          }
          
          arm_request_position = 0;
          arm_request_trajectory = 0;
          request_timer_flag = 0;
          arm_query_link = -1;
        }
        
        if(bytes_read == -2)
          printf("Frame Error\n");
      }

      if(FD_ISSET(arm_rs485_device, &wr))
      {
        while((arm_rs485_buffer_tx_empty == 0) && (arm_query_link == -1))
        {         
          bytes_sent = arm_rs485_write(arm_rs485_device, &arm_query_link, &arm_request_position, &arm_request_trajectory);
 
          if(bytes_sent > 0)
          {
            if(arm_request_position || arm_request_trajectory)
            {
              request_timer_flag = 1;
              clock_gettime(CLOCK_REALTIME, &request_timer_start);
              clock_gettime(CLOCK_REALTIME, &request_timer_stop);
              request_elapsed_time_ns = 0;
            }
          }
          else
          {
            printf("Error on rs485_write\n");
            return -1;
          }
        }
      }
    }
      
	/* Timeout region */
    if((select_timeout.tv_sec == 0) && (select_timeout.tv_usec == 0))
    {
      if((arm_position_initialized != ((2 << (MOTOR_NUMBER - 1)) - 1)) && (scu_state != SCU_ARM_OUT_OF_SERVICE))
      {
        init_position_timeout_counter++;
        if((init_position_timeout_counter * current_timeout) >= (long)INIT_POSITION_TIMEOUT_USEC)
        {
          init_position_timeout_counter = 0;
          for(query_link_count = 1; query_link_count <= MOTOR_NUMBER; query_link_count++)
          {
            if(((arm_position_initialized >> (query_link_count - 1)) && 0x01) == 0)
              arm_query_position(query_link_count);
          }
        }
      }
      else
        init_position_timeout_counter = 0;
      

      if(request_elapsed_time_ns >= (long)REQUEST_TIMEOUT_USEC * 1000)
      {
        request_timer_flag = 0;
        request_elapsed_time_ns = 0;
        clock_gettime(CLOCK_REALTIME, &request_timer_stop);

        if(scu_state != SCU_ARM_OUT_OF_SERVICE)
        {
          arm_link[arm_query_link - 1].timeout_counter++;

          if(arm_link[arm_query_link - 1].timeout_counter >= LINK_TIMEOUT_LIMIT)
          {
            if(arm_out_of_service_enable_flag)
            {
              printf("Arm\t[out of service link %i]\n", arm_query_link);
              scu_state = SCU_ARM_OUT_OF_SERVICE;

              if(show_arm_state_flag)
                printf("scu state\t[SCU_ARM_OUT_OF_SERVICE] in arm_rs485_device select(rd)\n");
            
              arm_stop(0);
              arm_set_command_without_value(0, "OFF");
              
              scu_state_rqst_flag = 1;
            }
            else
            {
              arm_link[arm_query_link - 1].actual_position = 0;

              if(arm_link[arm_query_link - 1].position_initialized == 0)
                arm_link[arm_query_link - 1].position_initialized = 1;

              arm_position_initialized |= (arm_link[arm_query_link - 1].position_initialized << (arm_query_link - 1));
              
              arm_link[arm_query_link - 1].trajectory_status = 0;
            }
          }
        }
          
        arm_query_link = -1;
      }
 
      if(stop_timer_flag)
      {
        stop_timeout_counter++;
        
        if((stop_timeout_counter * current_timeout) >= (long)STOP_TIMEOUT_USEC)
        {
          stop_timeout_counter = 0;
          
          if(arm_check_trajectory())
          {
            stop_timer_flag = 0;
            arm_rs485_flush_buffer_tx();
            arm_set_command_without_value(0, "OFF");
            arm_start_xyz();
            
            scu_state = scu_next_state;
             
            if(show_arm_state_flag)
              printf("scu state\t[scu_next_state = %i] in stop_timer\n", scu_state);
            
            scu_state_rqst_flag = 1;
          }
          else
            arm_query_trajectory(0);
        }
      }
      else
        stop_timeout_counter = 0;

      if(arm_position_initialized == ((2 << (MOTOR_NUMBER - 1)) - 1))
      {
        arm_position_timeout_counter++;
        if(arm_position_timeout_counter * current_timeout >= (long)ARM_POSITION_TIMEOUT_USEC)
        {
          arm_position_timeout_counter = 0;
        }  
      }
      
      select_timeout.tv_sec = TIMEOUT_SEC;
      select_timeout.tv_usec = ARM_TIMEOUT_USEC;

      current_timeout = select_timeout.tv_usec;
    }
  }  // end while(!= done)

  return 0;
}

void arm_status_update(unsigned char *arm_state, unsigned char *arm_next_state, unsigned char *arm_prev_state, struct arm_frame arm_message) 
{
  static unsigned char arm_stop_flag = 0;
  static unsigned char arm_abort_flag = 0;

  static float x = 0;
  static float y = 0;
  static float z = 0;
  
  switch(*arm_state)
  {
     
    case SCU_ARM_IDLE:
      if(arm_stop_flag)
        arm_stop_flag = 0;
        
      if(arm_abort_flag)
        arm_abort_flag = 0;
      
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
        case ARM_CMD_SECOND_TRIPLET:
        case ARM_CMD_ACTUATOR:
          arm_ee_xyz(&x, &y, &z);
          arm_start_xyz();
          arm_query_position(0);
          *arm_state = SCU_ARM_MOVE;
          *arm_next_state = SCU_ARM_IDLE;
          
          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_MOVE] in arm_status_update after ARM_CMD_MOVEs\n");
          
          scu_state_rqst_flag = 1;
          break;

        case ARM_CMD_STOP:
          break;

          
        case ARM_SET_ORIGIN:
          printf("Warning: predefined point set!\n");
          // store position
          arm_set_command(1, "O", -430870);
          arm_set_command(1, "p", -430870);
          arm_set_command(1, "EPTR", 100);
          arm_set_command_without_value(1, "VST(p,1)");

          arm_set_command(2, "O", 1152139);
          arm_set_command(2, "p", 1152139);
          arm_set_command(2, "EPTR", 100);
          arm_set_command_without_value(2, "VST(p,1)");
    
          arm_set_command(3, "O", -283872);
          arm_set_command(3, "p", -283872);
          arm_set_command(3, "EPTR", 100);
          arm_set_command_without_value(3, "VST(p,1)");
    
          arm_set_command(4, "O", -508);
          arm_set_command(4, "p", -508);
          arm_set_command(4, "EPTR", 100);
          arm_set_command_without_value(4, "VST(p,1)");
    
          arm_set_command(5, "O", 251534);
          arm_set_command(5, "p", 251534);
          arm_set_command(5, "EPTR", 100);
          arm_set_command_without_value(5, "VST(p,1)");
    
          arm_set_command(6, "O", -252424);
          arm_set_command(6, "p", -252424);
          arm_set_command(6, "EPTR", 100);
          arm_set_command_without_value(6, "VST(p,1)");
          break;
      }
      break;
      
    case SCU_ARM_REST:
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
        case ARM_CMD_SECOND_TRIPLET:
        case ARM_CMD_ACTUATOR:
          break;

        case ARM_CMD_STOP:
          break;
                  
        case ARM_RQST_ABORT:
          break;
          
        case ARM_SET_ORIGIN:
          break;
      }
      break;


    case SCU_ARM_MOVE:
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
          y += arm_message.arm_command_param.value2 * ARM_JOINT_YZ_STEP_M;
          z += arm_message.arm_command_param.value3 * ARM_JOINT_YZ_STEP_M;
          arm_move(1, arm_message.arm_command_param.value1, arm_message.arm_command_param.value2, arm_message.arm_command_param.value3);

          arm_query_position(0);
          break;
          
        case ARM_CMD_SECOND_TRIPLET:
          arm_move(2, arm_message.arm_command_param.value1, arm_message.arm_command_param.value2, arm_message.arm_command_param.value3);
          arm_query_position(0);
          break;
          
        case ARM_CMD_ACTUATOR:
          arm_move(3, arm_message.arm_command_param.value1, arm_message.arm_command_param.value2, arm_message.arm_command_param.value3);
          arm_query_position(0);
          break;

        case ARM_CMD_STOP:
          // flush tx buffer
          arm_rs485_flush_buffer_tx();
          
          // load stop command
          arm_stop(0);
          arm_query_trajectory(0);
          *arm_state = SCU_ARM_STOP;
          
          scu_state_rqst_flag = 1;
            
          stop_timer_flag = 1;
         
          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_STOP] in arm_status_update after ARM_CMD_STOP\n");
          break;

        case ARM_RQST_PARK:
          break;
          
        case ARM_RQST_PARK_CLASSA:
          break;
          
        case ARM_RQST_STEADY:
          break;
          
        case ARM_RQST_DINAMIC:
          break;
          
        case ARM_RQST_STEP:
          break;
          
        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
          break;
          
        case ARM_RQST_PUMP:
          break;
          
        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          break;
                    
        case ARM_RQST_ABORT:
          break;
          
        case ARM_SET_ORIGIN:
          break;
      }
      break;

    case SCU_ARM_STOP:
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
          break;
          
        case ARM_CMD_SECOND_TRIPLET:
          break;
          
        case ARM_CMD_ACTUATOR:
          break;

        case ARM_CMD_STOP:
          break;

        case ARM_RQST_PARK:
          break;
          
        case ARM_RQST_PARK_CLASSA:
          break;
          
        case ARM_RQST_STEADY:
          break;
          
        case ARM_RQST_DINAMIC:
          break;
          
        case ARM_RQST_STEP:
          break;
          
        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
          break;
          
        case ARM_RQST_PUMP:
          break;
          
        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          break;
            
        case ARM_RQST_ABORT:
          break;
          
        case ARM_SET_ORIGIN:
          break;
      }
      break;
  }
}

int eth_check_connection()
{
  FILE *file = NULL;
  char *line = NULL;
  size_t len = 0;
  ssize_t read;
  
  file = fopen("/sys/class/net/eth0/operstate", "r");

  if(file == NULL)
    return -1;

  read = getline(&line, &len, file);
  
  if(read != -1)
  {
    if(strncmp(line, "up", strlen("up")) == 0)
    {
      free(line);
      fclose(file);
      return 1;
    }
  }

  free(line);
  fclose(file);

  return 0;
}

int gpio_export(int pin_number)
{  
  FILE *file = NULL;

  file = fopen("/sys/class/gpio/export", "a");
    
  if(file == NULL)
    return -1;
    
  fprintf(file, "%i", pin_number);

  fclose(file);
  return 1;
}

int gpio_set_value(int pin_number, int value)
{
  FILE *file = NULL;
  char file_path[64];

  sprintf(file_path, "/sys/class/gpio/gpio%i/value", pin_number);
  file = fopen(file_path, "a");
      
  if(file == NULL)
    return -1;
    
  fprintf(file, "%i", value);

  fclose(file);
  return 1;
}

int gpio_set_direction(int pin_number, int value)
{
  FILE *file = NULL;
  char file_path[64];

  sprintf(file_path, "/sys/class/gpio/gpio%i/direction", pin_number);
  file = fopen(file_path, "a");
      
  if(file == NULL)
    return -1;
    
  fprintf(file, "%i", value);

  fclose(file);
  return 1;
}

int gpio_generic_set_value(char *path, int value)
{
  FILE *file = NULL;

  file = fopen(path, "a");
      
  if(file == NULL)
    return -1;
    
  fprintf(file, "%i", value);

  fclose(file);
  return 1;
}

