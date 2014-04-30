#ifndef ARM_H_
#define ARM_H_

#include "socket_udp.h"

#define ARM_CMD_FIRST_TRIPLET  0x0001
#define ARM_CMD_SECOND_TRIPLET 0x0002
#define ARM_CMD_ACTUATOR       0x0003
#define ARM_CMD_STOP           0x0004

#define ARM_RQST_PARK          0x0101
#define ARM_RQST_PARK_CLASSA   0x0102
#define ARM_RQST_STEADY        0x0103
#define ARM_RQST_DINAMIC       0x0104
#define ARM_RQST_STEP          0x0105
#define ARM_RQST_GET_TOOL_1    0x0106
#define ARM_RQST_GET_TOOL_2    0x0107
#define ARM_RQST_GET_TOOL_3    0x0108
#define ARM_RQST_GET_TOOL_4    0x0109
#define ARM_RQST_GET_TOOL_5    0x010a
#define ARM_RQST_GET_TOOL_6    0x010b
#define ARM_RQST_GET_TOOL_7    0x010c
#define ARM_RQST_PUMP          0x010d
#define ARM_RQST_PUT_TOOL_1    0x010e
#define ARM_RQST_PUT_TOOL_2    0x010f
#define ARM_RQST_PUT_TOOL_3    0x0110
#define ARM_RQST_PUT_TOOL_4    0x0111
#define ARM_RQST_PUT_TOOL_5    0x0112
#define ARM_RQST_PUT_TOOL_6    0x0113
#define ARM_RQST_PUT_TOOL_7    0x0114
#define ARM_RQST_ABORT         0x0115
#define ARM_SET_ORIGIN         0x0116

#define CRC_ADJUSTMENT 0xA001
#define CRC_TABLE_SIZE 256
#define INITIAL_CRC (0)

/*struct arm_frame
{
  union
  {
    struct
    {
      unsigned char index;
      char command[58];
      char count;
      __u8 request_position;
      __u8 request_trajectory_status;
      char crc[2];  //NOTE: crc should be out of arm_command_param struct
    } arm_command_param;

    char arm_command[64];
  };
};*/

struct arm_frame
{
  union
  {
    struct
    {
      __u16 header_uint;
  
      float value1 __attribute__((packed));
      float value2 __attribute__((packed));
      float value3 __attribute__((packed));
  
      __u16 crc_uint;
      
    } arm_command_param;
  
    char arm_command[16];
  };
};

extern unsigned char arm_buffer_tx_empty;
extern unsigned char arm_buffer_tx_full;
extern unsigned char arm_buffer_tx_overrun;

extern volatile unsigned char arm_net_down;

__u16 crc_table[CRC_TABLE_SIZE];

/* Prototype */
int arm_open(int *socket, struct sockaddr_in *address, char *ip_address, int dest_port);
int arm_open_server(int *socket, struct sockaddr_in *address, int src_port);
int arm_buffer_tx_get_space(void);
int arm_load_tx(struct arm_frame data);
int arm_send(int device, struct sockaddr_in *dest_address);
int arm_unload_rx(struct arm_frame *data);
int arm_read(int device, struct sockaddr_in *arm_client_address_dest);

void arm_crc_initialize(void);
__u16 arm_compute_crc_table_value(__u16);
__u16 arm_crc_calculate_crc_16(__u16, __u8);
unsigned char arm_crc_byte_buffer_crc_is_valid(__u8 *, __u32);
void arm_crc_compute_byte_buffer_crc(__u8 *, __u32);
#endif