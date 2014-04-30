#include <sys/types.h> 
#include <sys/socket.h> 
#include <sys/ioctl.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <net/if.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h>
 
#include <errno.h>
#include <string.h>
#include <locale.h>
#include <linux/joystick.h>

#include <netinet/in.h> 
#include <arpa/inet.h>


#include "joystick.h" 
#include "socket_udp.h"
#include "arm_udp.h"


#define JOY_ARM_PORT 8013
#define JOY_STATUS_PORT 8014
#define JOY_LOCAL_NAME "/dev/input/js0"
#define JOY_MAX_VALUE 32767 

#define TIMEOUT_SEC 0 
#define TIMEOUT_USEC 100000
#define TIMEOUT_ARM_USEC 10000
#define TIMEOUT_SEC_STATUS 1
#define LOG_FILE "/var/log/stdof_log.txt"

#define ARM_IDLE 0
#define ARM_MOVE 1
#define ARM_STOP 2
#define ARM_HOMING_REQUEST 3
#define ARM_AUTO_MOVE 4
#define ARM_AUTO_MOVE_ABORT 5
#define ARM_DINAMIC_REQUEST 6
#define ARM_REST 7

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))
  
/* Prototype */
void arm_status_update();
int arm_write_path(const char *file_path, long motor_position_target[MOTOR_NUMBER]);

int main(int argc, char**argv) 
{
  /* Joystick interface */
  struct wwvi_js_event jse;
  int joy_local = -1;
  
  /* Robotic Arm */
  int socket_arm = -1;
  struct sockaddr_in arm_address;
  struct arm_frame arm_buffer_temp;
  long motor_position_target[MOTOR_NUMBER];
  int i;
  long write_timeout = 0;
  
  /* Status client interface*/
  int socket_status = -1;
  struct sockaddr_in socket_status_addr_dest;

  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read
    unsigned char init = 0;
	  unsigned char i = 0;

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  
  
  if(argc != 4)  
  {
    printf("usage: %s <Source Address> <Source Port> <File name>\n", argv[0]);
    exit(1);
  }
  
  printf("Initializing stdof. . .\n");

  /* Peripheral initialization */
  
  /* Init Arm Client */  
  if(arm_open(&socket_arm, &arm_address, JOY_ARM_PORT, argv[1], atoi(argv[2])) == -1)
    perror("init arm client");
  else
  {
    if(arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 500, 500) < 0)
    {
      socket_arm = -1;
      perror("init arm");	  
    }
	
    //int arm_init(int index, long kp, long ki, long kd, long kv, long adt, long vt, long amps)
    arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);
    
    if(arm_set_command_without_value(0, "ZS") <= 0)	//Clear faults
      return -1;

    if(arm_set_command_without_value(0, "BRKSRV") <= 0)	//release brake only with servo active 
      return -1;

    if(arm_set_command_without_value(0, "MT") <= 0)	//set velocity mode 
      return -1;
    
    if(arm_set_command(index, "T", 0) <= 0) // set pwm drive signal limit
      return -1;
    
    if(arm_set_command_without_value(index, "F") <= 0) // active settings
      return -1;
  }

  
  /* Init Joystick */
  memset(&jse, 0, sizeof(struct wwvi_js_event));
  joy_local = open_joystick(JOY_LOCAL_NAME);

  if(joy_local < 0)
    perror("open_joystick(local)");
  else
    printf("Find Local Joystick\t[OK]\n");
  
  // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
  // the timeout on udev have to be < 2 secs.
  select_timeout.tv_sec = TIMEOUT_SEC;
  select_timeout.tv_usec = TIMEOUT_ARM_USEC;

  printf("Run main program. . .\n");
  
  while(!done)
  { 
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);
    
    if(socket_arm > 0)
    {
      FD_SET(socket_arm, &rd);
      nfds = max(nfds, socket_arm);  
	  
      if(arm_buffer_tx_empty == 0)
      {
        FD_SET(socket_arm, &wr);
        nfds = max(nfds, socket_arm);  
      }
    }
    
    if(joy_local > 0)
    {
      FD_SET(joy_local, &rd);
      nfds = max(nfds, joy_local);
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

    /* Manage arm message */
    if(socket_arm > 0)
    {
      if(FD_ISSET(socket_arm, &rd))
      {
        // the message would be an information such position or warning
        bytes_read = recvfrom(socket_arm, &arm_buffer_temp, sizeof(struct arm_frame), 0, NULL, NULL);

        if(bytes_read <= 0)
          perror("arm_read");
        else
        {
          if(query_link > -1)
          {
            if(arm_link[query_link - 1].request_actual_position == 1)
            {
              arm_link[query_link - 1].request_actual_position = 0;
              arm_link[query_link - 1].actual_position = atol(arm_buffer_temp.param.arm_command);
 
              if(init != 0)
              {
                printf("\033[%iA",MOTOR_NUMBER + 1);
              }
              else
                init = 1;
    
              printf("Actual Positions:\n");
              for(i = 0; i < MOTOR_NUMBER; i++)
              {
                printf("Link%i: %ld step %f deg        \n", i + 1, arm_link[i].actual_position, (double)arm_link[i].actual_position / (11 * arm_link[i].gear));
              }
   
              query_link = -1;
            }
            else if(arm_link[query_link - 1].request_trajectory_status == 1)
            {
              arm_link[query_link - 1].request_trajectory_status = 0;
  
              arm_link[query_link - 1].trajectory_status = atoi(arm_buffer_temp.param.arm_command);
              query_link = -1;
            }
          }
        }

        continue;
      }

      if(FD_ISSET(socket_arm, &wr))
      {
        arm_send(socket_arm, &arm_address);
        continue;
      }
    }
 
    if(joy_local > 0)
    {
      if(FD_ISSET(joy_local, &rd))
      {
        bytes_read = get_joystick_status(&joy_local, &jse);

        if(bytes_read <= 0)
          perror("get_joystick_status");
        else
        {
          // update at the end of the while
          continue;
        }
      }//end if(joy_selected == null)
    }

    arm_status_update();
    
    for(i = 0; i < MOTOR_NUMBER; i++)
      motor_position_target[i] = arm_link[i].actual_position;
    
    arm_write_path(argv[3], motor_position_target);
    
    select_timeout.tv_sec = TIMEOUT_SEC;
    select_timeout.tv_usec = TIMEOUT_ARM_USEC;

  }  // end while(!= done)

  return 0;
}


void arm_status_update() 
{
  int bytes_sent = 0;
  char selected_link;
  static int request_time = 0;
  static int last_link_queried = 0;
  
  const int timeout_index = 1;

  request_time++; // multiple of timeout

  if(query_link == -1)
  {
    last_link_queried++;
    if(last_link_queried > 6)
      last_link_queried = 1;
  }

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
  else if(query_link == -1)
    request_time = 0;
}

int arm_write_path(const char *file_path, long motor_position_target[MOTOR_NUMBER])
{
  FILE *file = NULL;
  int count = 0;
  
  // Init Log File
  file = fopen(file_path, "a");
  
  if(file == NULL)
    return -1;

  for(count = 0; count < (MOTOR_NUMBER - 1); count++)
  {
    fprintf(file, "%ld,", motor_position_target[count]);
  }
  
  fprintf(file, "%ld\n", motor_position_target[MOTOR_NUMBER - 1]);
  
  fclose(file);
  return 1;
}