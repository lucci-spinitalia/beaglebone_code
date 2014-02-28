#ifndef ARM_H_
#define ARM_H_

#include <time.h> //for debug
#include "joystick.h"
#include "socket_udp.h"

#define MOTOR_NUMBER 7

#define ARM_HOME_FILE "static_position"
#define ARM_DINAMIC_FILE "dinamic_position"
#define ARM_GET_BOX1_FILE "box1_get_position"
#define ARM_GET_BOX2_FILE "box2_get_position"
#define ARM_GET_BOX3_FILE "box3_get_position"
#define ARM_GET_BOX4_FILE "box4_get_position"
#define ARM_GET_BOX5_FILE "box5_get_position"
#define ARM_GET_BOX6_FILE "box6_get_position"
#define ARM_GET_BOX7_FILE "box7_get_position"
#define ARM_PUT_BOX1_FILE "box1_put_position"
#define ARM_PUT_BOX2_FILE "box2_put_position"
#define ARM_PUT_BOX3_FILE "box3_put_position"
#define ARM_PUT_BOX4_FILE "box4_put_position"
#define ARM_PUT_BOX5_FILE "box5_put_position"
#define ARM_PUT_BOX6_FILE "box6_put_position"
#define ARM_PUT_BOX7_FILE "box7_put_position"
#define ARM_PARK_FILE "park_position"
#define ARM_STEP_FILE "step_position"
#define ARM_GEAR_FILE "gear"
#define ARM_LINK_LENGTH_FILE "link_length"

#define CRC_ADJUSTMENT 0xA001
#define CRC_TABLE_SIZE 256
#define INITIAL_CRC (0)

struct arm_frame
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
};

struct arm_info
{
  __u32 velocity_target_limit;
  long velocity_target;
  long actual_position;
  __u8 position_initialized; //0: not initialize  1: initialize
  __u8 trajectory_status; //0: done 1: in progress
  
  long position_target;  // target position in motor step
  
  // this flag tell which request has been sent to the arm
  __u8 request_actual_position;
  __u8 request_trajectory_status;
  __u8 request_timeout;
  
  // Fix param
  unsigned int link_length;  //link's length in mm
  long link_offset_x;  //offset along x axis
  long link_offset_y;   //offset along y axis
  long link_offset_z;   //offset along z axis
  long gear;
};

extern unsigned char arm_buffer_tx_empty;
extern unsigned char arm_buffer_tx_full;
extern unsigned char arm_buffer_tx_overrun;
extern struct arm_info arm_link[MOTOR_NUMBER];
extern volatile int query_link;
extern volatile unsigned char arm_net_down;
extern unsigned char link_homing_complete;

__u16 crc_table[CRC_TABLE_SIZE];

// Debug timer
extern struct timespec debug_timer_start, debug_timer_stop;
extern unsigned char debug_timer;
extern long debug_elapsed_time;
extern long max_debug_elapsed_time;
extern long min_debug_elapsed_time;

/* Prototype */
int arm_open(int *socket, struct sockaddr_in *address, int src_port, char *ip_address, int dest_port);
int arm_open_server(int *socket, struct sockaddr_in *address, int src_port);
int arm_buffer_tx_get_space(void);
int arm_load_tx(struct arm_frame data);
//int arm_load_tx(struct wwvi_js_event jse);
int arm_send(int device, struct sockaddr_in *dest_address);
int arm_unload_rx(struct arm_frame *data);
int arm_read(int device, struct sockaddr_in *arm_client_address_dest);

int arm_init(int index, long kp, long ki, long kl, long kd, long kv, long adt, long vt, long amps);
int arm_start(void);
int arm_start_xyz(void);
int arm_stop(int index);
int arm_move(struct wwvi_js_event jse, __u16 joy_max_value);
int arm_move_xyz(double *x, double *y, double *z, struct wwvi_js_event jse, __u16 joy_max_value);
int arm_query_position(int link_to_query);
int arm_set_command(int index, char *command, long value);
int arm_set_command_without_value(int index, char *command);
int actuator_set_command(long command);
int actuator_request_trajectory();

int arm_automatic_motion_velocity_start(char *motion_file);
int arm_automatic_motion_xyz_start(char *motion_file);
int arm_automatic_motion_velocity_update();
int arm_automatic_motion_xyz_update();
void arm_automatic_motion_abort();
int arm_read_path_step(const char *file_path, long *motor_position_target, int *cursor_position);
int arm_read_path_xyz(const char *file_path, double *motor_position_target, int *cursor_position);

void arm_ik_ang(double pw_x, double pw_y, double pw_z, double *Teta1, double *Teta2, double *Teta3);
void arm_ee_xyz(double *pw_x, double *pw_y, double *pw_z);

void arm_crc_initialize(void);
__u16 arm_compute_crc_table_value(__u16);
__u16 arm_crc_calculate_crc_16(__u16, __u8);
unsigned char arm_crc_byte_buffer_crc_is_valid(__u8 *, __u32);
void arm_crc_compute_byte_buffer_crc(__u8 *, __u32);
#endif