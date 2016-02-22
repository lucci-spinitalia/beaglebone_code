#include <sys/types.h>
#include <linux/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <linux/serial.h>

#include "rs232.h"

#define _POSIX_SOURCE 1
#define FALSE 0
#define TRUE 1

/* Prototype */
int rs232_open(char *, __u32 , char ,
             int , int , char , char , int );

void flush_device_input(int *);
void flush_device_output(int *);

int rs232_load_tx(unsigned char *data, unsigned int data_length, int device_index);
int rs232_unload_rx(unsigned char *data, int device_index);
int rs232_unload_rx_multifiltered(char *data, char *token, char token_number, int device_index);
int rs232_write(int rs232_device, int device_index);
int rs232_read(int rs232_device, int device_index);
int rs232_buffer_tx_get_space(int device_index);
int rs232_buffer_rx_get_space(int device_index);

char rs232_buffer_tx[RS232_DEVICE_NUM][RS232_BUFFER_SIZE];
unsigned int rs232_buffer_tx_ptr_wr[RS232_DEVICE_NUM]; // write position in tx buffer
unsigned int rs232_buffer_tx_ptr_rd[RS232_DEVICE_NUM]; // read position to place data from
unsigned char rs232_buffer_tx_empty[RS232_DEVICE_NUM];
unsigned char rs232_buffer_tx_full[RS232_DEVICE_NUM];
unsigned char rs232_buffer_tx_overrun[RS232_DEVICE_NUM];
unsigned int rs232_buffer_tx_data_count[RS232_DEVICE_NUM];  // number of byte to transmit

char rs232_buffer_rx[RS232_DEVICE_NUM][RS232_BUFFER_SIZE];
unsigned int rs232_buffer_rx_ptr_wr[RS232_DEVICE_NUM]; // write position in rx buffer
unsigned int rs232_buffer_rx_ptr_rd[RS232_DEVICE_NUM];
unsigned char rs232_buffer_rx_empty[RS232_DEVICE_NUM];
unsigned char rs232_buffer_rx_full[RS232_DEVICE_NUM];
unsigned char rs232_buffer_rx_overrun[RS232_DEVICE_NUM];
unsigned int rs232_buffer_rx_data_count[RS232_DEVICE_NUM];  // number of byte received

int rs232_buffer_rx_bookmark[RS232_DEVICE_NUM]; /**< segna a che punto sono arrivato nella ricerca di un carattere nel buffer di ricezione */

int device_number = 0;

/***
 * Apre la seriale con le caratteristiche richieste.
 *
 * Input:
 *  device_name: stringa che indica il nome del dispositivo da aprire
 *  rate: intero senza segno a 32 bit che indica il badurate. I valori ammessi sono:
 *        50, 75, 110, 134, 150 200, 300, 600, 1200, 1800, 2400, 4800
 *        9600, 19200, 38400, 57600, 115200
 *  parity: carattere che indica l'uso del controllo di parità. I valori ammessi sono:
 *        N, O, E
 *  data_bits: intero che indica il numero di bit per frame. Sono ammessi numeri da 5 a 8
 *  stop_bits: numero di bit di stop frame. Può essere 1 o 2
 *  mark_error: se viene passato un valore maggiore di 0, imposta la segnalazione di un
 *              frame error o parity error (vedi PARMRK).
 *  translate_cr: se maggiore di zero, converte il carattere CarriegeReturn in NewLine
 *  device_index: indice da assegnare al dispositivo. Deve essere unico, maggiore di zero
 *                 progressivo e minore di RS232_DEVICE_NUM
 *
 * Output: Numero del descrittore del file rappresentante la seriale
 *
 */
int rs232_open(char *device_name, __u32 rate, char parity,
             int data_bits, int stop_bits, char mark_error, char translate_cr, int device_index)
{
  int fd;
  int local_rate = 0;
  int local_databits = 0;
  int local_stopbits = 0;
  int local_parity = 0;
  char upper_parity;
  struct termios oldtio, newtio;

  if((device_index < 0) || (device_index > device_number) || (device_index >= RS232_DEVICE_NUM))
    return -1;

  // Init circular buffer
  rs232_buffer_tx_ptr_wr[device_index] = 0;
  rs232_buffer_tx_ptr_rd[device_index] = 0;
  rs232_buffer_tx_empty[device_index] = 1;
  rs232_buffer_tx_full[device_index] = 0;
  rs232_buffer_tx_overrun[device_index] = 0;
  rs232_buffer_tx_data_count[device_index] = 0;

  rs232_buffer_rx_ptr_wr[device_index] = 0;
  rs232_buffer_rx_ptr_rd[device_index] = 0;
  rs232_buffer_rx_empty[device_index] = 1;
  rs232_buffer_rx_full[device_index] = 0;
  rs232_buffer_rx_overrun[device_index] = 0;
  rs232_buffer_rx_data_count[device_index] = 0;
  
  fd = open(device_name, O_RDWR | O_NOCTTY);

  if(fd < 0)
    return fd;

  if(!isatty(fd))
    printf("Not a serial device\n");
    
  // Check fo valid values
  upper_parity = toupper(parity);

  if(((data_bits == 5) || (data_bits == 6) || (data_bits == 7) || (data_bits == 8)) &&
     ((stop_bits == 2) || (stop_bits == 1)) &&
     ((upper_parity == 'N') || (upper_parity == 'O') || (upper_parity == 'E')) &&
     ((rate == 50) || (rate == 75) || (rate == 110) || (rate == 134) || (rate == 150) ||
      (rate == 200) || (rate == 300) || (rate == 600) || (rate == 1200) || (rate == 2400) || 
      (rate == 4800) || (rate == 9600) || (rate == 19200) || (rate == 38400) ||
      (rate == 57600) || (rate == 115200)))
  {
    switch(rate)
    {
      case 50: local_rate = B50; break;
      case 75: local_rate = B75; break;
      case 110: local_rate = B110; break;
      case 134: local_rate = B134; break;
      case 150: local_rate = B150; break;
      case 200: local_rate = B200; break;
      case 300: local_rate = B300; break;
      case 600: local_rate = B600; break;
      case 1200: local_rate = B1200; break;
      case 1800: local_rate = B1800; break;
      case 2400: local_rate = B2400; break;
      case 4800: local_rate = B4800; break;
      case 9600: local_rate = B9600; break;
      case 19200: local_rate = B19200; break;
      case 38400: local_rate = B38400; break;
      case 57600: local_rate = B57600; break;
      case 115200: local_rate = B115200; break;
    }

    switch(data_bits)
    {
      case 5: local_databits = CS5; break;
      case 6: local_databits = CS6; break;
      case 7: local_databits = CS7; break;
      case 8: local_databits = CS8; break;
    }

    if(stop_bits == 2)
      local_stopbits = CSTOPB;
    else
      local_stopbits = ~CSTOPB;

    switch(upper_parity)
    {
      case 'E': local_parity = PARENB; break;
      case 'O': local_parity |= PARODD; break;
    } 


    if(tcgetattr(fd, &oldtio) < 0)  //save current port settings
      return -1;

    bzero(&newtio, sizeof(newtio));
    
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //

    newtio.c_cflag |= (local_rate | local_databits | local_stopbits | local_parity | CREAD | CLOCAL); 
    newtio.c_cflag &= ~(PARODD | PARENB | CRTSCTS);
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    /*newtio.c_iflag &= ~(IGNPAR | IGNBRK | ICRNL | INLCR |
                        ISTRIP | IXON | IXOFF | IXANY | IGNCR);*/
    newtio.c_iflag &= ~(IGNPAR | IGNBRK | INLCR |
                            ISTRIP | IXON | IXOFF | IXANY | IGNCR);
    

    /*newtio.c_iflag |= (BRKINT | PARMRK | INPCK);*/
    newtio.c_iflag |= (BRKINT | INPCK);
    
    if(mark_error > 0)
      newtio.c_iflag |= (PARMRK);
    else
      newtio.c_iflag &= ~(PARMRK);

    if(translate_cr > 0)
      newtio.c_iflag |= (ICRNL);
    else
      newtio.c_iflag &= ~(ICRNL);

    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    // 
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    //newtio.c_oflag &= ~OPOST;
    newtio.c_oflag &= ~OPOST;
    
    //
    // No line processing:
    // echo off, echo newline off, canonical mode off, 
    // extended input processing off, signal chars off
    //
    newtio.c_lflag = ~(ECHO | ECHOE | ECHOK | ECHONL | ECHOPRT | ECHOCTL | ECHOKE | ICANON | ISIG | IEXTEN | NOFLSH | XCASE | TOSTOP);
    
    
    //newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; //inter-character timer unused
    newtio.c_cc[VMIN] = 1; //blocking read

    tcflush(fd, TCIFLUSH);
    
    if(cfsetispeed(&newtio, local_rate) < 0 || cfsetospeed(&newtio, local_rate) < 0) 
      return -1;

    if(tcsetattr(fd, TCSANOW, &newtio) < 0)
      return -1;

    device_number++;

    return fd;
  }
  else
  {
    close(fd);
    return -1;
  }
}

int rs232_close(int *rs232_device)
{
  if(*rs232_device > 0)
  {
    if(close(*rs232_device) < 0)
      return -1;

    *rs232_device = -1;
  }

  return 0;
}

void flush_device_input(int *device)
{
  tcflush(*device, TCIFLUSH);
}

void flush_device_output(int *device)
{
  tcflush(*device, TCOFLUSH);
}

int rs232_buffer_tx_get_space(int device_index)
{
  if((device_index < 0) || (device_index > device_number))
    return 0;

  return (RS232_BUFFER_SIZE - rs232_buffer_tx_data_count[device_index]);
}

int rs232_buffer_rx_get_space(int device_index)
{
  if((device_index < 0) || (device_index > device_number))
    return 0;

  return (RS232_BUFFER_SIZE - rs232_buffer_rx_data_count[device_index]);
}

int rs232_load_tx(unsigned char *data, unsigned int data_length, int device_index)
{
  int i;

  if((device_index < 0) || (device_index > device_number))
    return -1;

  if(rs232_buffer_tx_full[device_index])
  {
    rs232_buffer_tx_overrun[device_index] = 1;
    return -1;
  }

  if(rs232_buffer_tx_get_space(device_index) < data_length)
  {
    rs232_buffer_tx_full[device_index] = 1;
    rs232_buffer_tx_overrun[device_index] = 1;
    return -1;
  }

  for(i = 0; i < data_length; i++)
  {
    rs232_buffer_tx[device_index][rs232_buffer_tx_ptr_wr[device_index]] = data[i];

    rs232_buffer_tx_data_count[device_index] ++;
    rs232_buffer_tx_ptr_wr[device_index] ++;

    if(rs232_buffer_tx_ptr_wr[device_index] == RS232_BUFFER_SIZE)
      rs232_buffer_tx_ptr_wr[device_index] = 0;
  }

  rs232_buffer_tx_empty[device_index] = 0;

  if(rs232_buffer_tx_data_count[device_index] == RS232_BUFFER_SIZE)
    rs232_buffer_tx_full[device_index] = 1;

  return 1;
}

/***
 * Scarica i dati dal buffer di ricezione e li memorizza nella variabile passata.
 *
 * Input:
 *  data: buffer dove memorizzare le informazioni
 *  device_index: indice della seriale da cui prelevare le informazioni
 *
 *  Output:
 *    numero di byte scaricati dal buffer di ricezione.
 */
int rs232_unload_rx(unsigned char *data, int device_index)
{
  int length_to_write = 0;

  if((device_index < 0) || (device_index > device_number))
    return -1;

  if(rs232_buffer_rx_empty[device_index])
    return 0;

  rs232_buffer_rx_full[device_index] = 0;
 
  if(rs232_buffer_rx_ptr_rd[device_index] < rs232_buffer_rx_ptr_wr[device_index])
  {
    // se non sono tornato all'inizio del buffer circolare, allora posso
    // copiare tutti i dati in sequenza
    length_to_write = (rs232_buffer_rx_ptr_wr[device_index] - rs232_buffer_rx_ptr_rd[device_index]);
    memcpy(data, &rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_rd[device_index]], length_to_write);
  }
  else
  {
    // se sono tornato all'inizio del buffer circolare, devo eseguire la
    // copia in due momenti
    length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_rd[device_index]);
    memcpy(data, &rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_rd[device_index]], length_to_write);
    memcpy(&data[length_to_write], &rs232_buffer_rx[device_index], rs232_buffer_rx_ptr_wr[device_index] + 1);
    length_to_write = length_to_write + rs232_buffer_rx_ptr_wr[device_index];
  }
	
  rs232_buffer_rx_data_count[device_index] -= length_to_write;

  if(rs232_buffer_rx_data_count[device_index] == 0)
    rs232_buffer_rx_empty[device_index] = 1;

  rs232_buffer_rx_ptr_rd[device_index] += length_to_write;

  if(rs232_buffer_rx_ptr_rd[device_index] >= RS232_BUFFER_SIZE)
    rs232_buffer_rx_ptr_rd[device_index] -= RS232_BUFFER_SIZE;

  //if(rs232_buffer_rx_ptr_rd == RS232_BUFFER_SIZE)
  // rs232_buffer_rx_ptr_rd = 0;

//  printf("\nempty: %i, data_count: %i\n", rs232_buffer_rx_empty,rs232_buffer_rx_data_count);
//  printf("full: %i, rd pointer: %i\n", rs232_buffer_rx_full, rs232_buffer_rx_ptr_rd);
//  printf("\n");

  return length_to_write;
}

int rs232_unload_rx_multifiltered(char *data, char *token, char token_number, int device_index)
{
  int length_to_write = 0;
  char rs232_buffer_rx_temp[RS232_BUFFER_SIZE];

  char *token_ptr[10];
  char *token_winner = NULL;
  char *null_character = NULL;
  int token_count = 0;

  if((device_index < 0) || (device_index > device_number))
    return -1;

  if(rs232_buffer_rx_empty[device_index])
    return 0;

  if(token_number > 10)
    return -1;

  // it checks if bookmark arrives to the end and if it rolled up

  // if the bookmark pass the buffer limit, then it must
  // starts from the begginning
  if(rs232_buffer_rx_bookmark[device_index] >= RS232_BUFFER_SIZE)
    rs232_buffer_rx_bookmark[device_index] = rs232_buffer_rx_bookmark[device_index] - RS232_BUFFER_SIZE;

  // the bookmark must be less than write pointer
  if(rs232_buffer_rx_bookmark[device_index] == rs232_buffer_rx_ptr_wr[device_index])
    return 0;

  // if it doesn't roll up then it copy message into temp buffer
  // else it copy the last part of the buffer and the first one until data
  // length
  if(rs232_buffer_rx_bookmark[device_index] <= rs232_buffer_rx_ptr_wr[device_index])
  {
    length_to_write = rs232_buffer_rx_ptr_wr[device_index] - rs232_buffer_rx_bookmark[device_index];
    memcpy(rs232_buffer_rx_temp, &rs232_buffer_rx[device_index][rs232_buffer_rx_bookmark[device_index]],
        length_to_write);
  }
  else
  {
    length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_rx_bookmark[device_index]);

    memcpy(rs232_buffer_rx_temp, &rs232_buffer_rx[device_index][rs232_buffer_rx_bookmark[device_index]],
        length_to_write);
    memcpy(&rs232_buffer_rx_temp[length_to_write], rs232_buffer_rx[device_index],
        rs232_buffer_rx_ptr_wr[device_index] + 1);

    length_to_write = length_to_write + rs232_buffer_rx_ptr_wr[device_index] + 1;
  }

  // imposto il limite per la ricerca del token
  rs232_buffer_rx_temp[length_to_write] = '\0';

  // it checks for null character into the string
  // and replace it with 0x01 character.
  null_character = strchr(rs232_buffer_rx_temp, '\0');
  while(null_character != NULL)
  {
    if(null_character < &rs232_buffer_rx_temp[length_to_write])
      *null_character = 1;
    else
      break;

    null_character = strchr(rs232_buffer_rx_temp, '\0');
  }

  // it search for token
  for(token_count = 0; token_count < token_number; token_count++)
  {
    token_ptr[token_count] = strchr(rs232_buffer_rx_temp, token[token_count]);

    if(token_ptr[token_count] > token_winner)
      token_winner = token_ptr[token_count];
  }

  if(token_winner == NULL)
  {
    rs232_buffer_rx_bookmark[device_index] = rs232_buffer_rx_ptr_wr[device_index];
    rs232_buffer_rx_empty[device_index] = 1;

    return 0;
  }

  rs232_buffer_rx_full[device_index] = 0;

  // Questa volta il controllo deve essere fatto sul token, perchè, anche se il
  // puntatore di scrittura ha passato il limite, il token potrebbe essere ancora
  // nella coda del buffer
  if((rs232_buffer_rx_bookmark[device_index] >= rs232_buffer_rx_ptr_rd[device_index])
      && ((rs232_buffer_rx_ptr_rd[device_index] + (token_winner - rs232_buffer_rx_temp + 1))
          < RS232_BUFFER_SIZE))
  {
    length_to_write = (token_winner - rs232_buffer_rx_temp + 1)
        + (rs232_buffer_rx_bookmark[device_index] - rs232_buffer_rx_ptr_rd[device_index]);
    memcpy(data, &rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_rd[device_index]], length_to_write);
  }
  else
  {
    length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_rd[device_index]);

    memcpy(data, &rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_rd[device_index]], length_to_write);

    if(rs232_buffer_rx_bookmark[device_index] >= rs232_buffer_rx_ptr_rd[device_index])
      length_to_write = (token_winner - rs232_buffer_rx_temp + 1)
          + (rs232_buffer_rx_bookmark[device_index] - rs232_buffer_rx_ptr_rd[device_index]) - length_to_write;
    else
      length_to_write = (token_winner - rs232_buffer_rx_temp + 1) + rs232_buffer_rx_bookmark[device_index];

    memcpy(&data[RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_rd[device_index]], rs232_buffer_rx[device_index],
        length_to_write);

    length_to_write += (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_rd[device_index]);
  }

  data[length_to_write - 1] = 0;

  token_ptr[0] = strchr(data, '\377');

  if(token_ptr[0] != NULL)
  {
    rs232_buffer_rx_temp[token_ptr[0] - rs232_buffer_rx_temp] = '\0';
    strcat(rs232_buffer_rx_temp, &rs232_buffer_rx_temp[token_ptr[0] - rs232_buffer_rx_temp + 1]);

    memcpy(data, rs232_buffer_rx_temp, strlen(rs232_buffer_rx_temp));

    printf("Rs232 Frame error: %s\nat %i and long %i\n", rs232_buffer_rx_temp,
        token_ptr[0] - rs232_buffer_rx_temp, strlen(rs232_buffer_rx_temp));
  }

  rs232_buffer_rx_data_count[device_index] -= length_to_write;

  if((rs232_buffer_rx_data_count[device_index] < 0)
      || (rs232_buffer_rx_data_count[device_index] > RS232_BUFFER_SIZE))
    rs232_buffer_rx_empty[device_index] = 1;

  if(rs232_buffer_rx_data_count[device_index] == 0)
    rs232_buffer_rx_empty[device_index] = 1;

  rs232_buffer_rx_ptr_rd[device_index] += length_to_write;

  if(rs232_buffer_rx_ptr_rd[device_index] >= RS232_BUFFER_SIZE)
    rs232_buffer_rx_ptr_rd[device_index] -= RS232_BUFFER_SIZE;

  rs232_buffer_rx_bookmark[device_index] = rs232_buffer_rx_ptr_rd[device_index];

  if((token_ptr[0] != NULL) && ((token_ptr[0] - rs232_buffer_rx_temp) <= length_to_write))
    //return -2;
    return strlen(rs232_buffer_rx_temp);
  else
    return length_to_write;
}

int rs232_read(int rs232_device, int device_index)
{
  int bytes_read = -1;
  char rs232_buffer_rx_temp[RS232_BUFFER_SIZE];

  if((device_index < 0) || (device_index > device_number))
    return -1;

  if(rs232_buffer_rx_full[device_index])
  {
    rs232_buffer_rx_overrun[device_index] = 1;
    return -1;
  }

  if(rs232_device > 0)
  {
    // I want to read as many bytes as possible, but I have to manage the
    // circular buffer. So I can read only the data before restart the
    // buffer.
    //bytes_read = read(rs232_device, &rs232_buffer_rx[rs232_buffer_rx_ptr_wr], (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr));
    bytes_read = read(rs232_device, rs232_buffer_rx_temp, rs232_buffer_rx_get_space(device_index));
  
    if((RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr[device_index]) >= bytes_read)
      memcpy(&rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_wr[device_index]], rs232_buffer_rx_temp, bytes_read);
    else
    {
      //printf("Buffer reset-------------------------------------------------------------->\n");
      memcpy(&rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_wr[device_index]], rs232_buffer_rx_temp, (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr[device_index]));
      //printf("Copy first part...wr: %i bytes: %i\n",rs232_buffer_rx_ptr_wr, (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr));
      memcpy(rs232_buffer_rx[device_index], &rs232_buffer_rx_temp[RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr[device_index]], bytes_read - (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr[device_index]));
      //printf("Copy second part...buffer: %i bytes: %i\n", RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr, bytes_read - (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr));
    }
    
    //rs232_buffer_rx_temp[bytes_read] = '\0';
    //printf("Rs232 read: %s \nWrite pointer: %i Bytes read: %i\n", rs232_buffer_rx_temp, rs232_buffer_rx_ptr_wr, bytes_read);
    
    if(bytes_read > 0)
    {
      rs232_buffer_rx_empty[device_index] = 0;
      rs232_buffer_rx_data_count[device_index] += bytes_read;
 
      if(rs232_buffer_rx_data_count[device_index] == RS232_BUFFER_SIZE)
        rs232_buffer_rx_full[device_index] = 1;

      rs232_buffer_rx_ptr_wr[device_index] += bytes_read;

      if(rs232_buffer_rx_ptr_wr[device_index] >= RS232_BUFFER_SIZE)
      {
        //printf("Buffer rx ptr wr: %i of %i\tBytes read: %i\n", rs232_buffer_rx_ptr_wr, RS232_BUFFER_SIZE, bytes_read);
        rs232_buffer_rx_ptr_wr[device_index] -= RS232_BUFFER_SIZE;
        //printf("Buffer rx ptr wr After: %i of %i\n", rs232_buffer_rx_ptr_wr, RS232_BUFFER_SIZE);
      }
    }
  }

  return bytes_read;
}

int rs232_read_filter(int rs232_device, char * token, int device_index)
{
  int bytes_read = -1;
  char rs232_buffer_rx_temp[RS232_BUFFER_SIZE];

  if((device_index < 0) || (device_index > device_number))
    return -1;

  if(rs232_buffer_rx_full[device_index])
  {
    rs232_buffer_rx_overrun[device_index] = 1;
    return -1;
  }

  if(rs232_device > 0)
  {
    // I want to read as many bytes as possible, but I have to manage the
    // circular buffer. So I can read only the data before restart the
    // buffer.
    //bytes_read = read(rs232_device, &rs232_buffer_rx[rs232_buffer_rx_ptr_wr], (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr));
    bytes_read = read(rs232_device, rs232_buffer_rx_temp, rs232_buffer_rx_get_space(device_index));

    if((RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr[device_index]) >= bytes_read)
      memcpy(&rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_wr[device_index]], rs232_buffer_rx_temp, bytes_read);
    else
    {
      //printf("Buffer reset-------------------------------------------------------------->\n");
      memcpy(&rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_wr[device_index]], rs232_buffer_rx_temp, (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr[device_index]));
      //printf("Copy first part...wr: %i bytes: %i\n",rs232_buffer_rx_ptr_wr, (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr));
      memcpy(rs232_buffer_rx[device_index], &rs232_buffer_rx_temp[RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr[device_index]], bytes_read - (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr[device_index]));
      //printf("Copy second part...buffer: %i bytes: %i\n", RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr, bytes_read - (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr));
    }

    //rs232_buffer_rx_temp[bytes_read] = '\0';
    //printf("Rs232 read: %s \nWrite pointer: %i Bytes read: %i\n", rs232_buffer_rx_temp, rs232_buffer_rx_ptr_wr, bytes_read);

    if(bytes_read > 0)
    {
      rs232_buffer_rx_empty[device_index] = 0;
      rs232_buffer_rx_data_count[device_index] += bytes_read;

      if(rs232_buffer_rx_data_count[device_index] == RS232_BUFFER_SIZE)
        rs232_buffer_rx_full[device_index] = 1;

      rs232_buffer_rx_ptr_wr[device_index] += bytes_read;

      if(rs232_buffer_rx_ptr_wr[device_index] >= RS232_BUFFER_SIZE)
      {
        //printf("Buffer rx ptr wr: %i of %i\tBytes read: %i\n", rs232_buffer_rx_ptr_wr, RS232_BUFFER_SIZE, bytes_read);
        rs232_buffer_rx_ptr_wr[device_index] -= RS232_BUFFER_SIZE;
        //printf("Buffer rx ptr wr After: %i of %i\n", rs232_buffer_rx_ptr_wr, RS232_BUFFER_SIZE);
      }
    }
  }

  rs232_buffer_rx_temp[bytes_read] = '\0';
  if(strchr(rs232_buffer_rx_temp, *token) != NULL)
    return 1;
  else
    return 0;
}

int rs232_write(int rs232_device, int device_index)
{
  int bytes_sent = 0;
  int length_to_write = 0;

  if((device_index < 0) || (device_index > device_number))
    return -1;

  if(rs232_device > 0)
  {
    if(rs232_buffer_tx_empty[device_index])
      return 0;

    // I want to send as many bytes as possible, but I have to manage the
    // circular buffer. So I can send only the data before restart the
    // buffer.
    if(rs232_buffer_tx_ptr_rd[device_index] < rs232_buffer_tx_ptr_wr[device_index])
    {
      length_to_write = (rs232_buffer_tx_ptr_wr[device_index] - rs232_buffer_tx_ptr_rd[device_index]);
      
      if(length_to_write > RS232_MAX_TX_LENGTH)
        length_to_write = RS232_MAX_TX_LENGTH;
      
      bytes_sent = write(rs232_device, &rs232_buffer_tx[device_index][rs232_buffer_tx_ptr_rd[device_index]], length_to_write);
    }
    else
    {
      if((RS232_BUFFER_SIZE - rs232_buffer_tx_ptr_rd[device_index] + rs232_buffer_tx_ptr_wr[device_index]) > RS232_MAX_TX_LENGTH)
      {
        // devo capire quale parte del buffer supera RS232_MAX_TX_LENGTH
        if((RS232_BUFFER_SIZE - rs232_buffer_tx_ptr_rd[device_index]) > RS232_MAX_TX_LENGTH)
          bytes_sent = write(rs232_device, &rs232_buffer_tx[device_index][rs232_buffer_tx_ptr_rd[device_index]], RS232_MAX_TX_LENGTH);
        else
        {
          length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_tx_ptr_rd[device_index]);
          bytes_sent = write(rs232_device, &rs232_buffer_tx[device_index][rs232_buffer_tx_ptr_rd[device_index]], length_to_write);
          
          if(bytes_sent > 0)
          {
            //printf("bytes_sent: %i\n", bytes_sent);
            /*for(i = 0; i < bytes_sent; i++)
              printf("[%x]", rs232_buffer_tx[rs232_buffer_tx_ptr_rd + i]);
            printf("\n");*/ 
            rs232_buffer_tx_full[device_index] = 0;
            rs232_buffer_tx_data_count[device_index] -= bytes_sent;
            rs232_buffer_tx_ptr_rd[device_index] += bytes_sent;

            if(rs232_buffer_tx_ptr_rd[device_index] == RS232_BUFFER_SIZE)
              rs232_buffer_tx_ptr_rd[device_index] = 0;
            else if(rs232_buffer_tx_ptr_rd[device_index] > RS232_BUFFER_SIZE)
              printf("Circular buffer critical error\n");
          }
          
          length_to_write = RS232_MAX_TX_LENGTH - (RS232_BUFFER_SIZE - rs232_buffer_tx_ptr_rd[device_index]);
          bytes_sent = write(rs232_device, rs232_buffer_tx[device_index], length_to_write);
        }
      }
      else
      {
        // posso trasmettere tutti i dati nel buffer, ma sempre in due momenti
        length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_tx_ptr_rd[device_index]);
        bytes_sent = write(rs232_device, &rs232_buffer_tx[device_index][rs232_buffer_tx_ptr_rd[device_index]], length_to_write);
        
        if(bytes_sent > 0)
        {
          //printf("bytes_sent: %i\n", bytes_sent);
          /*for(i = 0; i < bytes_sent; i++)
            printf("[%x]", rs232_buffer_tx[rs232_buffer_tx_ptr_rd + i]);
          printf("\n");*/ 
          rs232_buffer_tx_full[device_index] = 0;
          rs232_buffer_tx_data_count[device_index] -= bytes_sent;
          rs232_buffer_tx_ptr_rd[device_index] += bytes_sent;

          if(rs232_buffer_tx_ptr_rd[device_index] == RS232_BUFFER_SIZE)
            rs232_buffer_tx_ptr_rd[device_index] = 0;
          else if(rs232_buffer_tx_ptr_rd[device_index] > RS232_BUFFER_SIZE)
            printf("Circular buffer critical error\n");
        }
          
        bytes_sent = write(rs232_device, rs232_buffer_tx[device_index], rs232_buffer_tx_ptr_wr[device_index]);
      }
    }

    if(bytes_sent > 0)
    {
      //printf("bytes_sent: %i\n", bytes_sent);
      /*for(i = 0; i < bytes_sent; i++)
        printf("[%x]", rs232_buffer_tx[rs232_buffer_tx_ptr_rd + i]);
      printf("\n");*/ 
      rs232_buffer_tx_full[device_index] = 0;
      rs232_buffer_tx_data_count[device_index] -= bytes_sent;
      rs232_buffer_tx_ptr_rd[device_index] += bytes_sent;

      if(rs232_buffer_tx_ptr_rd[device_index] == RS232_BUFFER_SIZE)
        rs232_buffer_tx_ptr_rd[device_index] = 0;
      else if(rs232_buffer_tx_ptr_rd[device_index] > RS232_BUFFER_SIZE)
        printf("Circular buffer critical error\n");
    }
  }

  if(rs232_buffer_tx_data_count[device_index] == 0)
    rs232_buffer_tx_empty[device_index] = 1;

  return bytes_sent;
}

int rs232_search_in_buffer(char *keyword, int device_index)
{
  if((device_index < 0) || (device_index > device_number))
    return 0;

  if(strstr(rs232_buffer_rx[device_index], keyword) != NULL)
    return 1;
  else
    return 0;
}

int rs232_check_last_char(char keyword, int device_index)
{
  if((device_index < 0) || (device_index > device_number))
    return -1;

  if(rs232_buffer_rx[device_index][rs232_buffer_rx_ptr_wr[device_index] - 1] == keyword)
    return 1;
  else
    return 0;
}
/******************* Example without using circular buffer***********************/
/*int main()
{
  fd_set rset;
  int status = 0;

  int rs232_device, res;
  char buf[255];

  rs232_device = com_open(MODEMDEVICE, 19200, 'N', 8, 1);

  while(STOP == FALSE)
  {
    FD_ZERO(&rset);

    if(rs232_device > 0)
      FD_SET(rs232_device, &rset);

    status = select(rs232_device + 1, &rset, NULL, NULL, NULL);

    if(status > 0)
    {
      if(FD_ISSET(rs232_device, &rset))
      { 
        res = read(rs232_device, buf, 255);
        buf[res] = 0;
        printf(":%s:%d\n", buf, res);
        write(rs232_device, buf, res);
  
        if(buf[0] == 'z')
          STOP = TRUE;
      }
    }
  }

//  tcsetattr(rs232_device, TCSANOW, &oldtio); 

  return 0;
}
*/

/******************* Example using circular buffer ***********************/
/*int main()
{
  fd_set rset, wset;
  int status = 0;

  int rs232_device, res;
  char buf[255];
  int bytes_read;

  rs232_device = com_open(MODEMDEVICE, 19200, 'N', 8, 1);

  while(STOP == FALSE)
  {
    FD_ZERO(&rset);
	FD_ZERO(&wset);

    if(rs232_device > 0)
	{
      FD_SET(rs232_device, &rset);
	  
	  if(rs232_buffer_tx_empty == 0)
	    FD_SET(rs232_device, &wset);
	}
    status = select(rs232_device + 1, &rset, &wset, NULL, NULL);

    if(status > 0)
    {
      if(FD_ISSET(rs232_device, &rset))
      { 
        res = rs232_read(rs232_device);
		
	if(res < 0)
	  perror("rs232_read");
      }
	  
	  if(FD_ISSET(rs232_device, &wset))
      { 
        res = rs232_write(rs232_device);
		
	if(res < 0)
	  perror("rs232_read");
      }
    }
	
    if(rs232_buffer_rx_empty == 0)
    {
      bytes_read = rs232_unload_rx(buf);
      buf[bytes_read] = 0;
  
      printf(":%s:%d\n", buf, res);
      rs232_load_tx(buf, bytes_read);
	  
      if(buf[0] == 'z')
        STOP = TRUE;
    }
  }

//  tcsetattr(rs232_device, TCSANOW, &oldtio); 

  return 0;
}*/

