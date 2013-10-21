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
void arm_status_update(char *file, int socket, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value) ;
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

  /* Status client interface*/
  int socket_status = -1;
  struct sockaddr_in socket_status_addr_dest;
  struct sockaddr_in socket_status_addr_src;
  unsigned char status_buffer = -1;

  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read
    unsigned char init = 0;
	  unsigned char i = 0;

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  long timeout_scale = 1;
  long time = 0;
  
  
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
	            printf("Link%i: %ld                               \n", i + 1, arm_link[i].actual_position);
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
    
    arm_status_update(argv[3], socket_status, &socket_status_addr_dest, &jse, JOY_MAX_VALUE);

    select_timeout.tv_sec = TIMEOUT_SEC;
    select_timeout.tv_usec = TIMEOUT_ARM_USEC;

  }  // end while(!= done)

  return 0;
}


void arm_status_update(char *file, int socket_status, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value) 
{
  char buffer[256];
  char status_buffer;
  int i;
  int bytes_sent = -1;
  static unsigned char arm_state = ARM_IDLE;  // arm's state
  static unsigned char append_position_on_file = 0;
  static unsigned char set_origin = 0;
  long motor_position_target[MOTOR_NUMBER];
      
  // Save position on file when button 4 is pressed
  if(jse->button[3] && !jse->button[0] && !jse->button[1] && !jse->button[2])
    append_position_on_file = 1;
	
  if((!jse->button[3]) && append_position_on_file)  //button released
  {
//	printf("ARM_HOMING_REQUEST\n");
    // store position
    for(i = 0; i < MOTOR_NUMBER; i++)
      motor_position_target[i] = arm_link[i].actual_position;
    
    arm_write_path(file, motor_position_target);
    append_position_on_file = 0;
  }

  if(jse->button[2] && !jse->button[0] && !jse->button[1] && !jse->button[3])
    set_origin = 1;

  if((!jse->button[2]) && set_origin)  //button released
  {
    // printf("ARM_HOMING_REQUEST\n");
    // store position
    for(i = 0; i < MOTOR_NUMBER; i++)
	{
      arm_set_command(i, "O", 0);
	  arm_set_command(i, "p", 0);
	  arm_set_command(i, "EPTR", 100);
	  arm_set_command_without_value(i, "VST(p,1)");
    }
	
    set_origin = 0;
  }
  
  if((jse->button[0] || jse->button[1]) && !append_position_on_file)
  {
    if(arm_state == ARM_IDLE)
    {
//	  printf("ARM_MOVE\n");
      arm_start();
      arm_state = ARM_MOVE;
    }
  }
  else if(!jse->button[0] && !jse->button[1])
  {
    if(arm_state == ARM_MOVE)
	{
//	  printf("ARM_STOP\n");
      arm_state = ARM_STOP;
	}
  }
  
  switch(arm_state)
  {
    case ARM_IDLE:
      break;

    case ARM_MOVE:
      bytes_sent = arm_move(*jse, JOY_MAX_VALUE);
       
      if(bytes_sent < 0)
        perror("arm_load_tx");
      break;
      
    case ARM_STOP:
      if(arm_stop(0))
      {
//	    printf("ARM_IDLE\n");
        arm_state = ARM_IDLE;
      }
      break;
  }
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