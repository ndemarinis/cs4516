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
#include <signal.h>
#include <pthread.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#ifndef LAYER_STACK_H
#define LAYER_STACK_H


#define PIPE_BUFFER_SIZE 2048
#define MAX_CLIENTS 5


#define PACKET_PAYLOAD_SIZE 256
#define FRAME_PAYLOAD_SIZE  150

#define MAX_FRAME_SIZE 158
#define MAX_PACKET_SIZE 260

#define MAX_SEQ 255
#define SLIDING_WINDOW_SIZE 4

#define FRAME_TYPE_FRAME 0xBE
#define FRAME_TYPE_ACK   0xEF

#define FRAME_NOT_EOP 0xDE
#define FRAME_IS_EOP 0xAD

#define FRAME_TERMINATOR 0x03

#define FRAME_TIMEOUT_MS 250

#define FRAME_KILL_EVERY_N_FRAMES 6
#define FRAME_KILL_EVERY_N_ACKS 8
#define FRAME_KILL_MAGIC 0x0800

// Possibly dangerous APP layer constants
#define MAX_PACKET 256
#define PACKET_OVERHEAD 4
#define MAX_PAYLOAD 252

#ifdef DID_DEBUG_MODE
#define DID_DEBUG_LEVEL 5
#else
#define DID_DEBUG_LEVEL 0
#endif

// Various debugging levels
#define DID_CRIT 0
#define DID_INFO 4
#define DID_WARN 2
#define DID_DLL_INFO 3

// Happy macros for getting the read and write components of a pipe's fd array
#define pipe_read(x) (x[0])
#define pipe_write(x) (x[1])

#define dprintf(level, format, ...) if(level <= DID_DEBUG_LEVEL) printf(format, ##__VA_ARGS__)

enum frame_event { PHYSICAL_FRAME_READY, NETWORK_FRAME_READY, CHECKSUM_ERROR, TIME_OUT, NOP };


struct response {
    int recordID;
    char *firstName;
    char *lastName;
    char *location;
};


// Info we pass to the thread that creates/watches the layer stack
struct stack_create_info
{
  int clnt_sock;
  int *app_layer_pipes;
  struct layer_stack *stack;
};


struct layer_info
{
  // FDs for the thread's input/output pipes
  int in;
  int out;
  struct layer_stack *stack; // Pointer to the whole stack so each layer can update the statistics
};

struct bidirectional_layer_info // So the DLL can't run as a send and recv thread
{
  int top_in, top_out;       // So we need to handle communication in both directions
  int bottom_in, bottom_out; 
  struct layer_stack *stack;
};

struct layer_stack
{
  struct layer_info net_send_info, net_recv_info; // FDs for each end of the pipe for each layer
  struct bidirectional_layer_info dl_info;
  struct layer_info phys_send_info, phys_recv_info;

  pthread_mutex_t net_dl_wire_lock, phys_dl_wire_lock; // Mutexes for our "nonblocking" DL pipes
  uint32_t net_to_dl_frame_size, phys_to_dl_frame_size;

  int total_frames_sent, total_acks_sent;
  int total_good_frames_sent, total_good_frames_recvd; // Statistics, as specified
  int total_good_acks_sent, total_good_acks_recvd;
  int total_bad_acks_recvd, total_dup_frames_recvd, total_bad_frames_recvd;
  int completed; // Whether or not we're done & can print statistics
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
struct layer_stack *create_layer_stack(int clnt_sock, int *app_layer_pipes);
void die_with_error(char *msg);
#endif
