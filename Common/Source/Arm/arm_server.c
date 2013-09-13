/* Sample UDP server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <linux/types.h>
#include <stdlib.h>

#include "arm_udp.h"

int main(int argc, char**argv)
{
  int sockfd, n,i;
  struct sockaddr_in servaddr, cliaddr;
  socklen_t len;
  struct arm_frame mesg;
  char *buffer;

  if(argc != 2)  
  {
    printf("usage: %s <Source Port>\n", argv[0]);
    exit(1);
  }
  
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(atoi(argv[1]));
  bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

  for(;;)
  {
    len = sizeof(cliaddr);
    //n = recvfrom(sockfd, &mesg, sizeof(struct wwvi_js_event), 0, (struct sockaddr *)&cliaddr, &len);
	n = recvfrom(sockfd, &mesg, sizeof(struct arm_frame), 0, (struct sockaddr *)&cliaddr, &len);
	
	buffer = strstr(mesg.param.command, "VT=");
	if(buffer != NULL)
	{
	  mesg.param.command[strlen(mesg.param.command) - 1] = 0;
      printf("Link: %i\tVel: %i\n", mesg.param.index - 128, atoi(&mesg.param.command[3]));
	}
	
    buffer = strstr(mesg.param.command, "X");
	if(buffer != NULL)
	{
	  mesg.param.command[strlen(mesg.param.command) - 1] = 0;
      printf("Slow motor motion to stop\n");
	}
    /*printf("------------------------------------------------------\n");
	printf("Received message from port %i, lenght: %i\n", ntohs(cliaddr.sin_port), strlen(mesg.frame));
    printf("Received the following:\n [%i]%s", mesg.frame[0], mesg.frame);*/
    /*printf("X: %i\tY: %i\tZ: %i\n", mesg.stick_x, mesg.stick_y, mesg.stick_z);
	printf("But1: %i\tBut2: %i\tBut3: %i\tBut4: %i\n", mesg.button[0], mesg.button[1], mesg.button[2], mesg.button[3]);*/
	
	/*printf("\n");
    printf("------------------------------------------------------\n");*/
	
	//sendto(sockfd, mesg, n, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
  }
}
