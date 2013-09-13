#ifndef _PC_INTERFACE_UDP_H
#define _PC_INTERFACE_UDP_H

#include <netinet/in.h>
#include "socket_udp.h"

struct pc_interface_udp_frame
{
  union
  {
    __u8 frame[34];
  
    struct
    {
	  __u8 header;
      __u8 length;
      __u8 id;
      __u8 data[31];
    } param;
  };
};

extern unsigned char pc_interface_buffer_tx_empty;
extern unsigned char pc_interface_buffer_tx_full;
extern unsigned char pc_interface_buffer_tx_overrun;
extern unsigned char pc_interface_buffer_rx_empty;
extern unsigned char pc_interface_buffer_rx_full;
extern unsigned char pc_interface_buffer_rx_overrun;

int pc_interface_connect(int *socket, struct sockaddr_in *address, int src_port, char *ip_address, int dest_port);
int pc_interface_buffer_tx_get_space(void);
int pc_interface_buffer_rx_get_space(void);
int pc_interface_load_tx(struct pc_interface_udp_frame data);
int pc_interface_unload_rx(struct pc_interface_udp_frame *data);
int pc_interface_send(int device, struct sockaddr_in *dest_address);
int pc_interface_read(int device);
#endif
