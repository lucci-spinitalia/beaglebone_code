#ifndef ARM_H_
#define ARM_H_

#include <time.h> //for debug
#include "joystick.h"
#include "socket_udp.h"

#define MOTOR_NUMBER 7
#define ARM_HOME_FILE "static_position"
#define ARM_DINAMIC_FILE "dinamic_position"
#define ARM_BOX_FILE "box_position"
#define ARM_PARK_FILE "park_position"
#define ARM_STEP_FILE "step_position"
#define ARM_GEAR_FILE "gear"

struct arm_frame
{
  union
  {
    char frame[67];
  
    struct
    {
      union
      {
        struct
        {
          unsigned char index;
          char command[63];
        } arm_command_param;

        char arm_command[64];
      };
  
      char homing_request_flag;
      char homing_complete_flag;
      char homing_abort_flag;
    } param;
  };
};

struct arm_info
{
  __u32 velocity_target_limit;
  long velocity_target;
  long actual_position;
  __u8 trajectory_status; //0: done 1: in progress
  
  long position_target;
  long gear;
  
  // this flag tell which request has been sent to the arm
  __u8 request_actual_position;
  __u8 request_trajectory_status;
  __u8 request_timeout;
};

extern unsigned char arm_buffer_tx_empty;
extern unsigned char arm_buffer_tx_full;
extern unsigned char arm_buffer_tx_overrun;
extern struct arm_info arm_link[MOTOR_NUMBER];
extern volatile int query_link;
extern volatile unsigned char arm_net_down;
extern unsigned char link_homing_complete;

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

int arm_init(int index, long kp, long ki, long kl, long kd, long kv, long adt, long vt, long amps);
int arm_start(void);
int arm_stop(int index);
int arm_move(struct wwvi_js_event jse, __u16 joy_max_value);
int arm_set_command(int index, char *command, long value);
int arm_set_command_without_value(int index, char *command);
int actuator_set_command(long command);
int actuator_request_trajectory();

int arm_automatic_motion_start(char *motion_file);
int arm_homing_check();
void arm_automatic_motion_abort();
int arm_read_path(const char *file_path, long *motor_position_target, int *cursor_position);

#endif