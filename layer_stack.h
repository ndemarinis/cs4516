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

#include <sys/socket.h>
#include <arpa/inet.h>

#ifndef LAYER_STACK_H
#define LAYER_STACK_H


#define PIPE_BUFFER_SIZE 128
#define MAX_CLIENTS 5

// Happy macros for getting the read and write components of a pipe's fd array
#define pipe_read(x) (x[0])
#define pipe_write(x) (x[1])

struct layer_info
{
  // FDs for the thread's input/output pipes
  int in;
  int out;
};

struct layer_stack
{
  struct layer_info net_send_info, net_recv_info; // FDs for each end of the pipe for each layer
  struct layer_info dl_send_info, dl_recv_info;
  struct layer_info phys_send_info, phys_recv_info;
  pthread_mutex_t wire_lock; // Mutex for controlling which piece of the physical layer has the wire
};

// Prototypes
int init_layer_stack(int clnt_sock, int *app_layer_pipes);
void die_with_error(char *msg);
#endif