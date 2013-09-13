#ifndef ARM_H_
#define ARM_H_

#include "joystick.h"
#include "socket_udp.h"

#define MOTOR_NUMBER 6

struct arm_frame
{
  union
  {
    char frame[64];
  
    struct
    {
	  char index;
      char command[63];
    } param;
  };
};

struct arm_info
{
  __u32 velocity_target_limit;
  __u32 actual_position;
  __u8 request_actual_position;
};

extern unsigned char arm_buffer_tx_empty;
extern unsigned char arm_buffer_tx_full;
extern unsigned char arm_buffer_tx_overrun;
extern struct arm_info arm_link[MOTOR_NUMBER];
extern volatile int query_link;
extern volatile unsigned char arm_net_down;

/* Prototype */
int arm_open(int *socket, struct sockaddr_in *address, int src_port, char *ip_address, int dest_port);
int arm_buffer_tx_get_space(void);
int arm_load_tx(struct arm_frame data);
//int arm_load_tx(struct wwvi_js_event jse);
int arm_send(int device, struct sockaddr_in *dest_address);

int arm_init(int index, long kp, long ki, long kd, long adt, long vt, long amps);
int arm_start(void);
int arm_stop(void);
int arm_move(struct wwvi_js_event jse, __u16 joy_max_value);
int arm_set_command(int index, char *command, long value);
int arm_set_command_without_value(int index, char *command);

#endif