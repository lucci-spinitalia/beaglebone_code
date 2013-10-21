/* Segway server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <error.h>
#include <errno.h>
#include <stdlib.h>

#include "arm_udp.h"

#define SAMPLE_RATE_US 125

#define MODE_POSITION 1
#define MODE_VELOCITY 3


/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

unsigned int gear[] = {220, 880, 220, 70, 256, 256};

void print_status(long *position, long *velocity_target, char *command, unsigned char *brake_status, unsigned char *timeout_status, 
		  unsigned char *mode)
{
  static unsigned char init = 0;
  int i;
  
  if(init != 0)
  {
    printf("\033[%iA",MOTOR_NUMBER);
  }
  else 
    init = 1;
  
  for(i = 0; i < MOTOR_NUMBER; i++)
  {
    printf("Link %i Position:%15ld Velocity:%15ld Command: %s      ", i + 1, position[i], velocity_target[i], command);
//    printf("Link %i Position:%15ld Velocity:%15ld Command: %s      \n", 3, position[2], velocity_target[2], command);
    
    if(mode[i] != MODE_VELOCITY)
      printf("<<-- Mode Unsupported   \n");
    else
    {
      if(brake_status[i] == 1)
      {
        printf("<<-- Brake");
      }
      else
        printf("          ");
    
      if(timeout_status[i] == 1)
        printf(" Timeout      \n");
      else
        printf("              \n");
    }
  }
}

int main(int argc, char**argv)
{
  int sockfd, n, i;
  struct sockaddr_in servaddr, cliaddr;
  socklen_t len = sizeof(cliaddr);
  struct arm_frame mesg;
  
  struct arm_frame tx_buffer;

  /* Select */
  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  int timeout_check_c = 0;
  
  /* Arm Param */
  double motor_velocity_rev_sec[MOTOR_NUMBER];
  long position[MOTOR_NUMBER];
  double position_step;
  double position_temp[MOTOR_NUMBER];
  long velocity_target[MOTOR_NUMBER];
  char last_command[MOTOR_NUMBER][256];
  int c[MOTOR_NUMBER];
  unsigned char brake_status[MOTOR_NUMBER];
  unsigned char timeout_status[MOTOR_NUMBER];
  unsigned char go[MOTOR_NUMBER];
  unsigned char brksrv[MOTOR_NUMBER];
  unsigned char mode[MOTOR_NUMBER];
  unsigned char trajectory_status[MOTOR_NUMBER];
  
  if(argc != 3)  
  {
    printf("usage: %s <IP address> <Port>\n", argv[0]);
    return 1;
  }
  
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  
  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(argv[1]);
  servaddr.sin_port = htons(atoi(argv[2]));

  if(bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
    perror("bind");

  bzero(position, sizeof(position));
  bzero(velocity_target, sizeof(velocity_target));
  bzero(c, sizeof(c));
  bzero(brake_status, sizeof(brake_status));
  bzero(timeout_status, sizeof(timeout_status));
  bzero(trajectory_status, sizeof(trajectory_status));
  bzero(mode, sizeof(mode));
  
  printf("Arm server started. . .\n");

  select_timeout.tv_sec = 0;
  select_timeout.tv_usec = SAMPLE_RATE_US;
      
  for(;;)
  {
    fflush(stdout);
    
    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);
    
    if(sockfd > 0)
    {
      FD_SET(sockfd, &rd);
      nfds = max(nfds, sockfd);
    }
    
    select_result = select(nfds + 1, &rd, NULL, NULL, &select_timeout);

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
    
    if(sockfd > 0)
    {
      if(FD_ISSET(sockfd, &rd))
      {
        n = recvfrom(sockfd, &mesg, sizeof(struct arm_frame), 0, (struct sockaddr *)&cliaddr, &len);
    
        if(n <= 0)
        {
          perror("recvfrom");
          continue;
        }

        mesg.param.arm_command_param.index -= 128;
    
        if(memcmp(mesg.param.arm_command_param.command, "RPA", sizeof("RPA") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
            continue;
 
          memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("RPA") - 1);
	  sprintf(tx_buffer.frame, "%ld%c%c", position[(unsigned char)mesg.param.arm_command_param.index - 1], 13, 0);

          if(sendto(sockfd, tx_buffer.frame, strlen(tx_buffer.frame), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
            perror("sendto");
        }
        else if(memcmp(mesg.param.arm_command_param.command, "VT=", sizeof("VT=") - 1) == 0)
        {
  
          if(mesg.param.arm_command_param.index == 0)
          {
            for(i = 0; i < MOTOR_NUMBER; i++)
	    {
	      memcpy(&(last_command[i][0]), mesg.param.arm_command_param.command, sizeof("VT=") - 1);
              velocity_target[i] = atol(&mesg.param.arm_command_param.command[sizeof("VT=") - 1]);
	    }
          }
          else
	  {
	    memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("VT=") - 1);
            velocity_target[(unsigned char)mesg.param.arm_command_param.index - 1] = atol(&mesg.param.arm_command_param.command[sizeof("VT=") - 1]);
	  }
        }
        else if(memcmp(mesg.param.arm_command_param.command, "c=", sizeof("c=") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
          {
            for(i = 0; i < MOTOR_NUMBER; i++)
	    {
	      memcpy(&(last_command[i][0]), mesg.param.arm_command_param.command, sizeof("c=") - 1);
              c[i] = atoi(&mesg.param.arm_command_param.command[sizeof("c=") - 1]);
              timeout_status[i] = 0;
	    }
          }
          else
	  {
	    memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("c=") - 1);
            c[(unsigned char)mesg.param.arm_command_param.index - 1] = atol(&mesg.param.arm_command_param.command[sizeof("c=") - 1]);
            timeout_status[(unsigned char)mesg.param.arm_command_param.index - 1] = 0;
	  }
        }
        else if(memcmp(mesg.param.arm_command_param.command, "G", sizeof("G") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
          {
            for(i = 0; i < MOTOR_NUMBER; i++)
	    {
	      memcpy(&(last_command[i][0]), mesg.param.arm_command_param.command, sizeof("G") - 1);
              go[i] = 1;
	    }
          }
          else
	  {
	    memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("G") - 1);
            go[(unsigned char)mesg.param.arm_command_param.index - 1] = 1;
	  }
        }
        else if(memcmp(mesg.param.arm_command_param.command, "BRKSRV", sizeof("BRKSRV") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
          {
            for(i = 0; i < MOTOR_NUMBER; i++)
	    {
	      memcpy(&(last_command[i][0]), mesg.param.arm_command_param.command, sizeof("BRKSRV") - 1);
              brksrv[i] = 1;
	    }
          }
          else
	  {
	    memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("BRKSRV") - 1);
            brksrv[(unsigned char)mesg.param.arm_command_param.index - 1] = 1;
	  }
        }
        else if(memcmp(mesg.param.arm_command_param.command, "MV", sizeof("MV") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
          {
            for(i = 0; i < MOTOR_NUMBER; i++)
	    {
	      memcpy(&(last_command[i][0]), mesg.param.arm_command_param.command, sizeof("MV") - 1);
              mode[i] = MODE_VELOCITY;
	    }
          }
          else
	  {
	    memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("MV") - 1);
            mode[(unsigned char)mesg.param.arm_command_param.index - 1] = MODE_VELOCITY;
	  }
        }
        else if(memcmp(mesg.param.arm_command_param.command, "MP", sizeof("MP") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
          {
            for(i = 0; i < MOTOR_NUMBER; i++)
            {
              memcpy(&(last_command[i][0]), mesg.param.arm_command_param.command, sizeof("MP") - 1);
              mode[i] = MODE_POSITION;
            }
          }
          else
          {
            memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("MP") - 1);
            mode[(unsigned char)mesg.param.arm_command_param.index - 1] = MODE_POSITION;
          }
        }
        else if(memcmp(mesg.param.arm_command_param.command, "OFF", sizeof("OFF") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
          {
            for(i = 0; i < MOTOR_NUMBER; i++)
            {
              memcpy(&(last_command[i][0]), mesg.param.arm_command_param.command, sizeof("OFF") - 1);

              if(brksrv[i] == 1)
                brake_status[i] = 1;
            }
          }
          else
          {
            memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("OFF") - 1);
     
            if(brksrv[(unsigned char)mesg.param.arm_command_param.index - 1] == 1)
              brake_status[(unsigned char)mesg.param.arm_command_param.index - 1] = 1;
          }
        }
        else if(memcmp(mesg.param.arm_command_param.command, "RB(0,2)", sizeof("RB(0,2)") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
            continue;
  
          memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("RB(0,2)") - 1);
          sprintf(tx_buffer.frame, "%u%c%c", trajectory_status[(unsigned char)mesg.param.arm_command_param.index - 1], 13, 0);

          //printf("Motor%i trajectory status: %u\n", (unsigned char)mesg.param.arm_command_param.index - 1,  trajectory_status[(unsigned char)mesg.param.arm_command_param.index - 1]);
          if(sendto(sockfd, tx_buffer.frame, strlen(tx_buffer.frame), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
            perror("sendto");
        }
        else if(memcmp(mesg.param.arm_command_param.command, "X", sizeof("X") - 1) == 0)
        {
          if(mesg.param.arm_command_param.index == 0)
          {
            for(i = 0; i < MOTOR_NUMBER; i++)
            {
              memcpy(&(last_command[i][0]), mesg.param.arm_command_param.command, sizeof("X") - 1);
       
              trajectory_status[i] = 0; //trajectory done
              velocity_target[i] = 0;
              go[i] = 0;
            }
          }
          else
          {     
            memcpy(&(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), mesg.param.arm_command_param.command, sizeof("X") - 1);
            trajectory_status[(unsigned char)mesg.param.arm_command_param.index - 1] = 0; //trajectory done
            velocity_target[(unsigned char)mesg.param.arm_command_param.index - 1] = 0;
            go[(unsigned char)mesg.param.arm_command_param.index - 1] = 0;
          }
        }
        //else 
        //  printf("Received %s message\n", mesg.param.arm_command_param.command);
        continue;
      }
    }
    
    /* Timeout */
    for(i = 0; i < MOTOR_NUMBER; i++)
    {
      if((go[i] == 1) && (mode[i] == MODE_VELOCITY))
      {
        trajectory_status[i] = 1; //trajectory in progress
        brake_status[i] = 0;  // realease brake
        motor_velocity_rev_sec[i] = (double)velocity_target[i] / 32768;

        //printf("Motor Velocity %i %f\n", i, motor_velocity_rev_sec[i]);
        //n_step =  SAMPLE_RATE / t_step = SAMPLE_RATE / ((1/4000)/vel)
        position_step = (SAMPLE_RATE_US * motor_velocity_rev_sec[i]) / 250;
        position_temp[i] += position_step;
        position[i] = (long)position_temp[i];
//        if(i == 2)
//          printf("Position calculated: %ld\n", position[i]);

      }
    }
    
    /* Timeout check c var */
    timeout_check_c++;
    
    if(timeout_check_c >= (int)(100000/SAMPLE_RATE_US)) // 100 ms
    {
      timeout_check_c = 0;
      for(i = 0; i < MOTOR_NUMBER; i++)
      {
	c[i]++;
	
        if(c[i] >= 5)
        {
	  c[i] = 0;
	  timeout_status[i] = 1;
	  if(mode[i] == MODE_VELOCITY)
            velocity_target[i] = 0;

          trajectory_status[i] = 0; //trajectory done
        }
      }
      
      print_status(position, velocity_target, &(last_command[(unsigned char)mesg.param.arm_command_param.index - 1][0]), brake_status, timeout_status,
                     mode);
    }
    
    
    select_timeout.tv_sec = 0;
    select_timeout.tv_usec = SAMPLE_RATE_US;
  }
}
