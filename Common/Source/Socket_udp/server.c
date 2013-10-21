/* Sample UDP server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


int main(int argc, char**argv)
{
  int sockfd, n;
  struct sockaddr_in servaddr, cliaddr;
  socklen_t len;
  char mesg[1000];

  if(argc < 3)
  {
    printf("Usage: %s <Source Address> <Source Port>\n", argv[0]);
    return 1;	
  }
  
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(argv[1]);
  servaddr.sin_port = htons(atoi(argv[2]));
  bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

  for(;;)
  {
    len = sizeof(cliaddr);
    n = recvfrom(sockfd, mesg, 1000, 0, (struct sockaddr *)&cliaddr, &len);
    sendto(sockfd, mesg, n, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
    printf("------------------------------------------------------\n");
	printf("Received message from port %i\n", ntohs(cliaddr.sin_port));
    mesg[n] = 0;
    printf("Received the following:\n");
    printf("%s", mesg);
    printf("------------------------------------------------------\n");
  }
}
