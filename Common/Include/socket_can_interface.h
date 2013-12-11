#ifndef _SOCKET_CAN_INTERFACE_H_
#define _SOCKET_CAN_INTERFACE_H_

/* Prototype */
int can_init(int *, struct sockaddr_can *, struct ifreq *, int, int);
int can_restart(void);

#endif
