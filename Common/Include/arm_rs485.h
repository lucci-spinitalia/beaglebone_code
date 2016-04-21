#ifndef ARM_RS485_H_
#define ARM_RS485_H_

#include <linux/types.h>

#define MOTOR_NUMBER 7
#define SMART_MOTOR_NUMBER 6
#define SMART_MOTOR_SYNC_INDEX 1
#define LINK5_SLN -253440
#define LINK5_SLP 298496
#define LINK6_SLN -844800
#define LINK6_SLP 844800

#define X12 0.068 // � il disassamento con il giunto 3 e non solo con il 2
#define Y12 0.155 // distanza tra l'asse del giunto 1 e quello del giunto 2
#define Z12 0.098 // distanza tra l'asse del giunto 1 e quello del giunto 2

#define A2 0.5 //
#define A3 0.512 // distanza tra il gunto 3 ed il 5
#define WRIST_SIZE 0.37677 // distanza tra il giunto 5 e la punta della pinza

#define TETHA_OFFSET atan2(X12, Y12)
#define LENGTH_OFFSET sqrt(pow(X12, 2) + pow(Y12, 2))

#define ARM_RS485_BUFFER_SIZE 1024

#define ARM_PARK_FILE "/opt/spinitalia/default_position/static_position"
#define ARM_DINAMIC_FILE "/opt/spinitalia/default_position/dinamic_position"
#define ARM_READY_FILE "/opt/spinitalia/default_position/ready_position"
#define ARM_GET_BOX1_FILE "/opt/spinitalia/default_position/box1_get_position"
#define ARM_GET_BOX2_FILE "/opt/spinitalia/default_position/box2_get_position"
#define ARM_GET_BOX3_FILE "/opt/spinitalia/default_position/box3_get_position"
#define ARM_GET_BOX4_FILE "/opt/spinitalia/default_position/box4_get_position"
#define ARM_GET_BOX5_FILE "/opt/spinitalia/default_position/box5_get_position"
#define ARM_GET_BOX6_FILE "/opt/spinitalia/default_position/box6_get_position"
#define ARM_GET_BOX7_FILE "/opt/spinitalia/default_position/box7_get_position"
#define ARM_PUT_BOX1_FILE "/opt/spinitalia/default_position/box1_put_position"
#define ARM_PUT_BOX2_FILE "/opt/spinitalia/default_position/box2_put_position"
#define ARM_PUT_BOX3_FILE "/opt/spinitalia/default_position/box3_put_position"
#define ARM_PUT_BOX4_FILE "/opt/spinitalia/default_position/box4_put_position"
#define ARM_PUT_BOX5_FILE "/opt/spinitalia/default_position/box5_put_position"
#define ARM_PUT_BOX6_FILE "/opt/spinitalia/default_position/box6_put_position"
#define ARM_PUT_BOX7_FILE "/opt/spinitalia/default_position/box7_put_position"
#define ARM_PARK_CLASSA_FILE "/opt/spinitalia/default_position/park_position"
#define ARM_STEP_FILE "/opt/spinitalia/default_position/step_position"
#define ARM_GEAR_FILE "/opt/spinitalia/config/gear"
#define ARM_LINK_LENGTH_FILE "/opt/spinitalia/config/link_length"
#define ARM_GET_OUT_FROM_REST_FILE "/opt/spinitalia/default_position/get_out_from_rest"

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

#define LINK_TIMEOUT_LIMIT 100

struct arm_rs485_frame
  {
    union
      {
        struct
          {
            unsigned char index;
            char command[58];

            __u8 request_position;
            __u8 request_trajectory_status;
            __u8 request_interpolation_status;
            __u8 request_error_status;
            __u8 request_mode;

          //__u16 interpolation_status;
          } arm_command_param;

        char arm_command[64];
      };
  };

struct arm_info
  {
    __u32 velocity_target_limit;
    float angle_upper_limit;
    float angle_lower_limit;
    long velocity_target;
    long actual_position;
    __u8 position_initialized; //0: not initialize  1: initialize
    __u8 trajectory_status; //0: done 1: in progress

    long position_target;  // target position in motor step
    unsigned int timeout_counter;

    // Interpolation param
    __u8 slots;
    __u8 pending_g;
    __u8 ip_ready;
    __u8 invalid_time_delta;
    __u8 invalid_position_delta;
    __u8 md_ready;
    __u8 data_buffer_overflow;
    __u8 data_buffer_underflow;
    __u8 md_running;

    // Fix param
    unsigned int link_length;  //link's length in mm
    long link_offset_x;  //offset along x axis
    long link_offset_y;   //offset along y axis
    long link_offset_z;   //offset along z axis
    long gear;
  };

extern const float arm_encoder_factor;

extern unsigned char arm_rs485_buffer_tx_empty;
extern unsigned char arm_rs485_buffer_tx_full;
extern unsigned char arm_rs485_buffer_tx_overrun;
extern struct arm_info arm_link[MOTOR_NUMBER];
extern volatile int query_link;

__u16 crc_table[CRC_TABLE_SIZE];

// Debug timer
extern struct timespec debug_timer_start, debug_timer_stop;
extern unsigned char debug_timer;
extern long debug_elapsed_time;
extern long max_debug_elapsed_time;
extern long min_debug_elapsed_time;

/* Prototype */
int arm_rs485_open(char *device_name, __u32 rate, char parity, int data_bits, int stop_bits);
void flush_device_input(int *device);
void flush_device_output(int *device);
void arm_rs485_flush_buffer_tx(void);
int arm_rs485_buffer_tx_get_space(void);
int arm_rs485_buffer_rx_get_space(void);
int arm_rs485_load_tx(struct arm_rs485_frame data);
int arm_rs485_write(int device, int *query_link, unsigned char *request_position,
    unsigned char *request_trajectory_status, unsigned char *request_interpolation_status,
    unsigned char *request_error_status);
int arm_rs485_get_last_message_write(struct arm_rs485_frame *arm_rs485_buffer);
int arm_rs485_unload_interpolation(unsigned char *data);
int arm_rs485_unload_rx(unsigned char *data);
int arm_rs485_unload_rx_filtered(char *data, char token);
int arm_rs485_unload_rx_multifiltered(char *data, char *token, char token_number);
int arm_rs485_read(int rs232_device);

int arm_init(int index, long kp, long ki, long kl, long kd, long kv, long adt, long vt, long amps,
    float upper_limit[], float lower_limit[]);
void arm_set_max_velocity(int index, long velocity);
//int arm_start(void);
int arm_start_xyz(void);
int arm_stop(int index);
int arm_home_start(int index);

//int arm_move(struct wwvi_js_event jse, __u16 joy_max_value);
int arm_move(unsigned char triplet_selected, float value1, float value2, float value3);
int arm_move_xyz(unsigned char triplet_selected, float value1, float value2, float value3);
int arm_query_position(int link_to_query);
int arm_query_trajectory(int link_to_query);
int arm_check_trajectory();
int arm_set_command(int index, char *command, long value);
int arm_set_command_without_value(int index, char *command);
int actuator_set_command(long command);
void actuator_get_status(struct arm_info *arm_link);
int actuator_request_trajectory();
int actuator_request_position();

int arm_automatic_motion_velocity_start(char *motion_file);
int arm_automatic_motion_xyz_start(char *motion_file);
void arm_automatic_motion_xyz_update_cursor();
int arm_automatic_motion_velocity_update();
int arm_automatic_motion_xyz_update();
void arm_automatic_motion_abort();
int arm_read_path_step(const char *file_path, long *motor_position_target, int *cursor_position);
int arm_read_path_xyz(const char *file_path, float *motor_position_target, int *cursor_position);

void arm_ik_ang(float pw_x, float pw_y, float pw_z, float *Teta1, float *Teta2, float *Teta3);
void arm_ee_xyz(long motor_step[], float *pw_x, float *pw_y, float *pw_z);

void arm_crc_initialize(void);
__u16 arm_compute_crc_table_value(__u16);
__u16 arm_crc_calculate_crc_16(__u16, __u8);
unsigned char arm_crc_byte_buffer_crc_is_valid(__u8 *, __u32);
void arm_crc_compute_byte_buffer_crc(__u8 *, __u32);

int arm_give_a_shit(void);
#endif
