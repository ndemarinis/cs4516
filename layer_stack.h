/*
 * layer_stack.h
 * Nicholas DeMarinis
 * 29 March 2012
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#ifndef LAYER_STACK_H
#define LAYER_STACK_H


#define PIPE_BUFFER_SIZE 2048
#define MAX_CLIENTS 5
#define PACKET_OVERHEAD 6

#define PACKET_PAYLOAD_SIZE 256
#define FRAME_PAYLOAD_SIZE  150

#define MAX_FRAME_SIZE 158
#define MAX_PACKET_SIZE 260

#define MAX_SEQ 1

#define FRAME_TYPE_FRAME 0xBE
#define FRAME_TYPE_ACK   0xEF

#define FRAME_NOT_EOP 0xDE
#define FRAME_IS_EOP 0xAD

#define FRAME_TERMINATOR 0x03

#define FRAME_TIMEOUT_MS 100

// Happy macros for getting the read and write components of a pipe's fd array
#define pipe_read(x) (x[0])
#define pipe_write(x) (x[1])

enum frame_event { PHYSICAL_FRAME_READY, NETWORK_FRAME_READY, CHECKSUM_ERROR, TIME_OUT, NOP };

struct layer_info
{
  // FDs for the thread's input/output pipes
  int in;
  int out;
};

struct bidirectional_layer_info // So the DLL can't run as a send and recv thread
{
  int top_in, top_out;       // So we need to handle communication in both directions
  int bottom_in, bottom_out; 
};

struct layer_stack
{
  struct layer_info net_send_info, net_recv_info; // FDs for each end of the pipe for each layer
  struct layer_info dl_send_info, dl_recv_info;
  struct layer_info phys_send_info, phys_recv_info;
  pthread_mutex_t wire_lock; // Mutex for controlling which piece of the physical layer has the wire
};

struct frame_window {
  uint8_t end_of_pkt;       // Whether or not this payload is the end of the packet
  uint8_t length;
  struct timeval time_sent; // Time this packet was sent
  char payload[FRAME_PAYLOAD_SIZE];
};
  

struct packet {
  uint16_t seq_num; //First two bytes are the sequence number
  uint8_t opcode; //The opcode for this packet
  uint8_t length; //6th byte is the length of this packets data field 
  char payload[PACKET_PAYLOAD_SIZE]; //reserve space for the maximum amount of data the payload could contain
};

struct packet_segment {
  uint8_t end_of_pkt;
  uint8_t length;
  char payload[FRAME_PAYLOAD_SIZE];
};

struct frame {
  uint8_t  type;       // Frame type (Data or ACK), one byte
  uint16_t seq;        // Sequence number for this frame
  uint8_t  end_of_pkt; // Whether or not this is the end of the backet
  uint8_t length;      // Length of the payload
  uint16_t checksum;   // Checksum, determined with folding XOR
  char payload[FRAME_PAYLOAD_SIZE]; // Payload buffer, max of 150 bytes
};

// Prototypes
int init_layer_stack(int clnt_sock, int *app_layer_pipes);
void die_with_error(char *msg);
#endif
