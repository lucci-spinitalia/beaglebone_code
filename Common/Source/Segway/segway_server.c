/* Segway server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <error.h>
#include <stdlib.h>
#include <errno.h>

#include "segway_udp_v2.h"

#define SEGWAY_TIMEOUT_USEC 100000
#define MAX_VEL_MPS 3
#define MAX_YAW_RPS 3

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

void print_status(__u32 operational_state, float linear_velocity, float yaw_rate)
{
  static unsigned char init = 0;
  char operational_state_buffer[256];
  
  if(init != 0)
  {
    printf("\033[2A");
  }
  else 
    init = 1;
  
  switch(operational_state)
  {
    case CCU_INIT:
      sprintf(operational_state_buffer, "CCU_INIT");
      break;
    case PROPULSION_INIT:
      sprintf(operational_state_buffer, "PROPULSION_INIT");
      break;
    case CHECK_STARTUP:
      sprintf(operational_state_buffer, "CHECK_STARTUP");
      break;
    case SEGWAY_STANDBY:
      sprintf(operational_state_buffer, "SEGWAY_STANDBY");
      break; 
    case SEGWAY_TRACTOR:
      sprintf(operational_state_buffer, "SEGWAY_TRACTOR");
      break;
    case DISABLE_POWER:
      sprintf(operational_state_buffer, "DISABLE_POWER");
      break;
    case UNKNOWN:
      sprintf(operational_state_buffer, "UNKNOWN");
      break;
    default:
      sprintf(operational_state_buffer, "UNKNOWN");
      break;
  }

  printf("Operational State: %s\n", operational_state_buffer); 
  printf("Linear Velocity(m/s):%2.3f Yaw Rate(rad/s):%2.3f          \n", linear_velocity, yaw_rate);
}

void send_param_message(int sockfd, struct sockaddr_in *cliaddr, __u32 bitmap1_mask, __u32 bitmap2_mask, __u32 bitmap3_mask, union segway_union segway_param)
{
  union segway_union bitmap;
  __u32 bitmap_count = 0;
  int bytes_sent = 0;
  int i;

  bzero(&bitmap, sizeof(bitmap));

  for(i = 0; i < 32; i++)
  {
    if(((bitmap1_mask >> i) & 0x01) == 1)
    {
      bitmap.segway_feedback[bitmap_count] = __builtin_bswap32(segway_param.segway_feedback[i]);
      //printf("bitmap1[%i][%i] = %08lx\n", bitmap_count, i, bitmap.segway_feedback[bitmap_count]);
      bitmap_count++;
    }
  }

  for(i = 0; i < 32; i++)
  {
    if(((bitmap2_mask >> i) & 0x01) == 1)
    { 
      bitmap.segway_feedback[bitmap_count] = __builtin_bswap32(segway_param.segway_feedback[i + 32]);
      //printf("bitmap2[%i][%i] = %lu\n", bitmap_count, i, bitmap.segway_feedback[bitmap_count]);
      bitmap_count++;
    }
  }

  for(i = 0; i < 32; i++)
  {
    if(((bitmap3_mask >> i) & 0x01) == 1)
    { 
       bitmap.segway_feedback[bitmap_count] = __builtin_bswap32(segway_param.segway_feedback[i + 64]);
       //printf("bitmap3[%i][%i] = %lu\n", bitmap_count, i, bitmap.segway_feedback[bitmap_count]);
       bitmap_count++;
    }
  }

  tk_crc_compute_byte_buffer_crc(bitmap.segway_feedback_u8, ((bitmap_count + 1) * 4));
  //printf("crc[%i] = %lu, [%i] = %lu\n", bitmap_count, bitmap.segway_feedback[bitmap_count], bitmap_count + 1, bitmap.segway_feedback[bitmap_count + 1]);

  //printf("Vel: %f\n", convert_to_float(segway_param.list.linear_vel_mps) );
  
  bytes_sent = sendto(sockfd, bitmap.segway_feedback, (bitmap_count + 1) * sizeof(__u32), 0, (struct sockaddr *)cliaddr, sizeof(*cliaddr));
  if(bytes_sent == -1)
    perror("sendto"); 
  
}

int main(int argc, char**argv)
{


  int sockfd, n;
  struct sockaddr_in servaddr, cliaddr;
  socklen_t len = sizeof(cliaddr);
  struct udp_frame mesg;

  /* Select */
  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  int timeout_check = 0;
  int disable_check = 0;
  int battery_check = 0;
  int battery_direction = 1;
  float battery_value = 100;
  long disable_timeout = 0;

  __u32 bitmap1_mask = 0xFFFFFFFF;
  __u32 bitmap2_mask = 0xFFFFFFFF;
  __u32 bitmap3_mask = 0x0000FFFF;
  
  union segway_union segway_param;

  __u8 message[(SEGWAY_PARAM*4) + 3];
  __u32 uvel = 0;
  __u32 uyaw = 0;
  __u32 umax_vel = 0;
  __u32 umax_yaw_rate = 0;
  
  float vel = 0;
  float yaw = 0;
  float max_vel = MAX_VEL_MPS;
  float max_yaw_rate = MAX_YAW_RPS;

  if(argc < 3)
  {
    printf("usage: %s <IP address> <Port> <disable timer> \n", argv[0]);
    return 1;
  }

  if(argc > 3)
  {
    disable_timeout = atol(argv[3]);
  }

  bzero(segway_param.segway_feedback, sizeof(segway_param.segway_feedback));
  segway_param.list.operational_state = 0x3;
  segway_param.list.front_base_batt_1_soc = convert_to_ieee754(100);
  segway_param.list.front_base_batt_2_soc = convert_to_ieee754(100);
  segway_param.list.rear_base_batt_1_soc = convert_to_ieee754(100);
  segway_param.list.rear_base_batt_2_soc = convert_to_ieee754(100);

  tk_crc_compute_byte_buffer_crc(message, sizeof(message));

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(argv[1]);
  servaddr.sin_port = htons(atoi(argv[2]));

  if(bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
    perror("bind");

  printf("Segway server started. . .\n");

  tk_crc_initialize();

  select_timeout.tv_sec = 0;
  select_timeout.tv_usec = SEGWAY_TIMEOUT_USEC;
  
  for(;;)
  {
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
        n = recvfrom(sockfd, &mesg, sizeof(struct udp_frame), 0, (struct sockaddr *)&cliaddr, &len);

        if(n <= 0)
        {
          perror("recvfrom");
          continue;
        }

        if(tk_crc_byte_buffer_crc_is_valid(mesg.frame, sizeof(mesg.frame)) == 0)
        {
          //printf("Bad crc\n");
          continue;
        }
     
        switch(__builtin_bswap32(mesg.param.udp_id << 16))
        {
          case 0x500:    
            timeout_check = 0;

            uvel = (mesg.param.data[0] << 24) | (mesg.param.data[1] << 16) | (mesg.param.data[2] << 8) | (mesg.param.data[3]);
            vel = convert_to_float(uvel);
            uyaw = (mesg.param.data[4] << 24) |(mesg.param.data[5] << 16) |(mesg.param.data[6] << 8) |(mesg.param.data[7]);
            yaw = convert_to_float(uyaw);
            break;

          case 0x501:
 
            switch(mesg.param.data[3])
            {
              case 0x00:  // None
                break;

              case 0x01: // Max Velocity
                memcpy(&umax_vel, &mesg.param.data[4], sizeof(__u32));
                segway_param.list.vel_limit_mps = __builtin_bswap32(umax_vel);
                max_vel = convert_to_float(segway_param.list.vel_limit_mps);
                //printf("Vel limit: %f\n", convert_to_float(segway_param.list.vel_limit_mps));
                break;

              case 0x06:  // Max Yaw Rate
                memcpy(&umax_yaw_rate, &mesg.param.data[4], sizeof(__u32));
                segway_param.list.yaw_rate_limit_rps = __builtin_bswap32(umax_yaw_rate);
                max_yaw_rate = convert_to_float(segway_param.list.yaw_rate_limit_rps);
                //printf("Yaw limit: %f\n", convert_to_float(segway_param.list.yaw_rate_limit_rps));
                break;   

              case 0x10: //configure bitmap 1
                memcpy(&bitmap1_mask, &mesg.param.data[4], sizeof(__u32));
                bitmap1_mask = __builtin_bswap32(bitmap1_mask);
                //printf("Bitmap1 configuration received: %lu\n", bitmap1_mask);
                break;

              case 0x11: //configure bitmap 2
                memcpy(&bitmap2_mask, &mesg.param.data[4], sizeof(__u32));
                bitmap2_mask = __builtin_bswap32(bitmap2_mask);
                //printf("Bitmap2 configuration received: %lu\n", bitmap2_mask);
                break;

              case 0x12: //configure bitmap 3
                memcpy(&bitmap3_mask, &mesg.param.data[4], sizeof(__u32));
                bitmap3_mask = __builtin_bswap32(bitmap3_mask);
                //printf("Bitmap3 configuration received: %lu\n", bitmap3_mask);
                break;

              case 0x15:
                switch(mesg.param.data[7])
                {
                  case 0x04:
                    //printf("---->Requested Standby mode \n");
                    segway_param.list.operational_state = 0x03;
                    vel = 0;
                    yaw = 0;
                    if(sendto(sockfd, message, sizeof(message), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
                      perror("sendto");
                    break;

                  case 0x05:
                    //printf("---->Requested Tractor mode \n");
                    if(sendto(sockfd, message, sizeof(message), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
                      perror("sendto");

                    segway_param.list.operational_state =  0x04;
                    break;          
                }
                break;

              default:
                if(sendto(sockfd, message, sizeof(message), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
                  perror("sendto");

                //printf("Configuration message for segway:"); 
                /*for(i = 0; i < sizeof(mesg.frame); i++)
                  printf("[%x]", mesg.frame[i]);
  
                printf("\n");
                fflush(stdout);*/
                break;
            }
        
            break;

          default:
            //printf("Unknown Message\n");
            break;
        }  //end switch

        segway_param.list.linear_vel_mps = convert_to_ieee754(vel * max_vel);
        segway_param.list.inertial_z_rate_rps = convert_to_ieee754(yaw * max_yaw_rate);
        send_param_message(sockfd, &cliaddr, bitmap1_mask, bitmap2_mask, bitmap3_mask, segway_param);
        print_status(segway_param.list.operational_state, convert_to_float(segway_param.list.linear_vel_mps), convert_to_float(segway_param.list.inertial_z_rate_rps));
        continue;
      }
    }

    timeout_check++;
    if(timeout_check > (2000000 / SEGWAY_TIMEOUT_USEC))
    {
      vel = 0;
      yaw = 0;
      timeout_check = 0;
      segway_param.list.linear_vel_mps = convert_to_ieee754(vel * max_vel);
      segway_param.list.inertial_z_rate_rps = convert_to_ieee754(yaw * max_yaw_rate);

      print_status(segway_param.list.operational_state, convert_to_float(segway_param.list.linear_vel_mps), convert_to_float(segway_param.list.inertial_z_rate_rps));
    }

    if(disable_timeout != 0)
    {
      disable_check++;
      if(disable_check > (disable_timeout / SEGWAY_TIMEOUT_USEC))
      {
        disable_check = 0;
        segway_param.list.operational_state = 0x05;
        vel = 0;
        yaw = 0;
      }
    }

    battery_check++;
    if(battery_check > (1000000 / SEGWAY_TIMEOUT_USEC))
    {
      battery_check = 0;
      if(battery_direction == 1)
        battery_value++;
      else
        battery_value--;

      if(battery_value >= 100)
        battery_direction = 0;
      else if(battery_value <= 0)
        battery_direction = 1;

      segway_param.list.front_base_batt_1_soc = convert_to_ieee754(battery_value);
    }

    select_timeout.tv_sec = 0;
    select_timeout.tv_usec = SEGWAY_TIMEOUT_USEC;
  } // end while
}
