/*
 * did_server.c
 * Nicholas DeMarinis
 * 29 March 2012
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <sys/socket.h>

#include "layer_stack.h"

#define DID_DEFAULT_PORT 4516
#define MAX_PENDING 2
#define MAX_CLIENTS 5

// Struct for data we send to the client handler
struct client_handler_data
{
  int sock; // Just the client's fd, for now
  pthread_t client_handler;
  struct layer_stack *stack;
};


// Prototypes
void *handle_client(void *data);

// Globals
struct client_handler_data client_data[MAX_CLIENTS];

int main(int argc, char *argv[])
{
  int srv_sock, clnt_sock, curr_clients = 0;
  unsigned int clnt_len;
  struct sockaddr_in srv_addr, clnt_addr;

  pthread_t threads[MAX_CLIENTS];

  // Create our listen socket
  if((srv_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    die_with_error("socket() failed!");

  // Create the address structure
  memset(&srv_addr, 0, sizeof(srv_addr));
  srv_addr.sin_family = AF_INET;
  srv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  srv_addr.sin_port = htons(DID_DEFAULT_PORT);

  setvbuf(stdout, NULL, _IONBF, 0);

  // Bind to the address we just specified
  if((bind(srv_sock, (struct sockaddr *)(&srv_addr), sizeof(srv_addr)) < 0))
    die_with_error("bind() failed!\n");

  // Listen for connections
  if((listen(srv_sock, MAX_PENDING) < 0))
    die_with_error("listen() failed!");

  for(;;)
    {
      // When we receive a connection from a a client, make a new thread to handle them
      clnt_len = sizeof(clnt_addr);
      
      if((clnt_sock = accept(srv_sock, (struct sockaddr *)(&clnt_addr), &clnt_len)) < 0)
	die_with_error("accept() failed!");
      
      client_data[curr_clients].sock = clnt_sock;

      if(curr_clients < MAX_CLIENTS)
	pthread_create(&(client_data[curr_clients].client_handler), NULL, handle_client, 
		       (void *)(&(client_data[curr_clients])));
      else
	die_with_error("Too many clients!");

      curr_clients++;
    }
  
  // We should never reach this.  Yet.
  pthread_exit(NULL);
  exit(0);
}


/**
 * handle_client:  Main application layer thread for each client
 * @author ndemarinis (Basic implementation)
 */
void *handle_client(void *data)
{
  struct client_handler_data *clnt = (struct client_handler_data *)data;
  struct layer_stack *stack; // Pointer to statistics and such from the layer stack
  int pipes[2]; // Make a pipe to connect to the layer stack

  int to_read, bytes_written;
  char read_buffer[PIPE_BUFFER_SIZE];
  struct packet* pkt_in;

  memset(read_buffer, 0, PIPE_BUFFER_SIZE);
  
  create_layer_stack(clnt->sock, pipes); // Initialize all of our layer threads
  sleep(1); // Wait for the layer stack creation to settle

  for(;;)
    {
      // Just try and echo a message for now.
      printf("APP:  Starting a test read.\n\n");

      // Grab a string
      if((to_read = read(pipe_read(pipes), read_buffer, PIPE_BUFFER_SIZE)) <= 0)
	{
	  printf("APP:  Read 0 bytes from socket.  Terminating!\n");
	  break;
	}

      pkt_in = (struct packet *)read_buffer;

      printf("APP:  Read packet of %d bytes with payload of %d bytes\n", 
	     to_read, pkt_in->length);

      // Send it straight back
      printf("APP:  Sending packet of %d bytes back to client\n", to_read);
      if((bytes_written = write(pipe_write(pipes), read_buffer, to_read)) <= 0)
	{
	  printf("APP:  Wrote %d bytes, socket must have closed.  Terminating!\n", bytes_written);
	  break;
	}
    }
  
  printf("Client successfully terminated!\n");
  pthread_exit(NULL);
}
