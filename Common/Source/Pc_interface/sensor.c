/* Sample UDP server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <linux/types.h>
#include <stdlib.h>

#include "pc_interface_udp.h"

int main(int argc, char**argv)
{
  int sockfd, n,i;
  struct sockaddr_in servaddr, cliaddr;
  socklen_t len;
  struct udp_frame mesg;

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
    n = recvfrom(sockfd, mesg.frame, sizeof(struct udp_frame), 0, (struct sockaddr *)&cliaddr, &len);
    
    printf("------------------------------------------------------\n");
	printf("Received message from port %i\n", ntohs(cliaddr.sin_port));
    printf("Received the following:\n");
    printf("Id: [%x]\nLength: [%x]\nData:", (((__u32)(mesg.param.length & 0x07) << 8) | mesg.param.id), mesg.param.length >> 3);
	
	for(i = 0; i < (mesg.param.length >> 3); i++)
	{
	  printf("[%x]", mesg.param.data[i]);
	}
	
	printf("\n");
    printf("------------------------------------------------------\n");
	
	sendto(sockfd, mesg.frame, n, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
  }
}
