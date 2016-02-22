#ifndef LMS511_H_
#define LMS511_H_

#include "socket_tcp.h"

/* MACRO */
#define lms511_config_request()         (lms511_send_command("sRN LMPscancfg"))
#define lms511_login_as_manteinance()   (lms511_send_command("sMN SetAccessMode 02 B21ACE26"))
#define lms511_login_as_auth_client()   (lms511_send_command("sMN SetAccessMode 03 F4724744"))
#define lms511_logout()                 (lms511_send_command("sMN Run"))
#define lms511_start_measure()          (lms511_send_command("LMCstartmeas"))
#define lms511_query_status()           (lms511_send_command("sRN STlms"))
#define lms511_scan_request()           (lms511_send_command("sRN LMDscandata"))
#define lms511_scan_auto()		          (lms511_send_command("sEN LMDscandata 1"))

/* lms511 state */
#define LMS511_UNDEFINED 0
#define LMS511_INIT      1
#define LMS511_CONFIG    2
#define LMS511_IDLE      3
#define LMS511_ROTATING  4
#define LMS511_PREP      5
#define LMS511_READY     6
#define LMS511_MEASURE   7

extern unsigned char lms511_buffer_tx_empty;
extern unsigned char lms511_buffer_tx_full;
extern unsigned char lms511_buffer_tx_overrun;
extern struct LMS511_INFO lms511_info;

struct LMS511_INFO
{
  /*config*/
  int error;
  unsigned long scanning_frequency;
  unsigned long angle_resolution;
  long starting_angle;
  long stopping_angle;

  /*data*/
  unsigned char state;
  unsigned char temperature_range_met;
  unsigned long scaling_factor;
  int angular_step;
  unsigned int spot_number;

  struct
  {
    unsigned int spot_number;
    unsigned int *spot;
  } data;
};

/* Prototype */
int lms511_open(int *socket, struct sockaddr_in *address, char *ip_address, int dest_port);
int lms511_buffer_tx_get_space(void);
int lms511_buffer_rx_get_space(void);
int lms511_load_tx(char *data);
int lms511_unload_rx(unsigned char *data);
int lms511_send(int device, struct sockaddr_in *dest_address);
int lms511_read(int device);
int lms511_init();
void lms511_dispose();
int lms511_parse(int socket_lms511);
int lms511_send_command(char *command);
int lms511_config_set(long scanning_freq, long angle_resolution, long starting_angle,
    long stopping_angle);

#endif
