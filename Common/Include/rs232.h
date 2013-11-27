#ifndef __RS232_H_
#define __RS232_H_

#include <linux/types.h>

#define RS232_BUFFER_SIZE 65536
#define RS232_MAX_TX_LENGTH 1024

extern unsigned char rs232_buffer_tx_empty;
extern unsigned char rs232_buffer_tx_full;
extern unsigned char rs232_buffer_rx_empty;
extern unsigned char rs232_buffer_rx_full;

int com_open(char *device_name, __u32 rate, char parity, int data_bits, int stop_bits);

void flush_device_input(int *);
void flush_device_output(int *);

int rs232_load_tx(unsigned char *data, unsigned int data_length);
int rs232_unload_rx(unsigned char *data);
int rs232_unload_rx_filtered(char *data, char token);
int rs232_write(int rs232_device);
int rs232_read(int rs232_device);
int rs232_buffer_tx_get_space(void);
int rs232_buffer_rx_get_space(void);
int rs232_search_in_buffer(char *keyword);
int rs232_check_last_char(char keyword);
#endif
