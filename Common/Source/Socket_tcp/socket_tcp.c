#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

int init_tcp_client(int *net_socket, const char *dest_addr, int dest_portnumber)
{
  struct sockaddr_in serv_addr;
  struct hostent *server;

  *net_socket = socket(AF_INET, SOCK_STREAM, 0);

  if(net_socket < 0)
    return -1;

  server = gethostbyname(dest_addr);

  if(server == NULL)
    return -1;

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);

  serv_addr.sin_port = htons(dest_portnumber);

  if(connect(*net_socket, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    return -1;
	
  return 0;
}
/*void dostuff(int);

void error(const char *msg)
{
  perror(msg);
  exit(1);
}

int main(int argc, char *argv[])
{
  int sockfd, newsockfd, portno, pid;
  socklen_t clilen;
  struct sockaddr_in serv_addr, cli_addr;
 
  if(argc < 2) 
  {
    fprintf(stderr, "ERROR, no port provided\n");
    exit(1);
  }

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  
  if(sockfd < 0)
    error("ERROR opening socket");

  bzero((char *) &serv_addr, sizeof(serv_addr));
  portno = atoi(argv[1]);
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(portno);
  
  if(bind(sockfd, (struct sockaddr *) &serv_addr,
     sizeof(serv_addr)) < 0)
    error("ERROR on binding");

  listen(sockfd, 5);
  clilen = sizeof(cli_addr);

  while(1)
  {
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);

    if(newsockfd < 0)
      error("ERROR on accept");

    pid = fork();

    if(pid < 0)
      error("ERROR on fork");

    if(pid == 0)
    {
      close(sockfd);
      dostuff(newsockfd);
      exit(0);
    }
    else
      close(newsockfd);
 
  }

  return 0;
}

void dostuff(int sock)
{
  int n;
  char buffer[256];

  bzero(buffer, 256);

  n = read(sock, buffer, 255);

  if(n < 0)
    error("ERROR reading from socket");

  printf("Here is the message: %s\n", buffer);

  n = write(sock, "I got your message", 18);

  if(n < 0)
    error("ERROR writing to socket");
}*/
