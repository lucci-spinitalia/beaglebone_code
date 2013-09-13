#ifndef _SOCKET_UDP_H
#define _SOCKET_UDP_H

#include <netinet/in.h>

/* Protype */
extern int init_server(int *, struct sockaddr_in *, int);
extern int init_client(int *, struct sockaddr_in *, const char *, int);
extern int init_client2(int *net_socket, struct sockaddr_in *socket_addr, int src_portnumber, const char *dest_addr, int dest_portnumber);
#endif
