#ifndef ARM_H_
#define ARM_H_

#include "socket_udp.h"

/* Prototpe */
int arm_open(int *socket, struct sockaddr_in *address, char *ip_address, int port);
int arm_buffer_tx_get_space(void);
int arm_load_tx(char *data, int data_length);
int arm_send(int device, struct sockaddr_in *dest_address);


#endif