#ifndef __RS232_H_
#define __RS232_H_

#include <linux/types.h>

#define RS232_BUFFER_SIZE 65536
#define RS232_MAX_TX_LENGTH 1024
#define RS232_DEVICE_NUM 5

extern unsigned char rs232_buffer_tx_empty[RS232_DEVICE_NUM];
extern unsigned char rs232_buffer_tx_full[RS232_DEVICE_NUM];
extern unsigned char rs232_buffer_tx_overrun[RS232_DEVICE_NUM];
extern unsigned char rs232_buffer_rx_empty[RS232_DEVICE_NUM];
extern unsigned char rs232_buffer_rx_full[RS232_DEVICE_NUM];
extern unsigned char rs232_buffer_rx_overrun[RS232_DEVICE_NUM];

int rs232_open(char *device_name, __u32 rate, char parity,
             int data_bits, int stop_bits, char mark_error, char translate_cr, int device_index);
int rs232_close(int *rs232_device);

void flush_device_input(int *);
void flush_device_output(int *);

int rs232_load_tx(unsigned char *data, unsigned int data_length, int device_index);
int rs232_unload_rx(unsigned char *data, int device_index);
int rs232_unload_rx_multifiltered(char *data, char *token, char token_number, int device_index);
int rs232_write(int rs232_device, int device_index);
int rs232_read(int rs232_device, int device_index);
int rs232_read_filter(int rs232_device, char * token, int device_index);
int rs232_buffer_tx_get_space(int device_index);
int rs232_buffer_rx_get_space(int device_index);
int rs232_search_in_buffer(char *keyword, int device_index);
int rs232_check_last_char(char keyword, int device_index);
#endif
