#ifndef _SOCKET_UDP_H
#define _SOCKET_UDP_H

#include <netinet/in.h>

/* Protype */
extern int init_tcp_client(int *net_socket, const char *dest_addr, int dest_portnumber);

#endif
