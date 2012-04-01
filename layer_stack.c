/*
 * layer_stack.c
 * Nicholas DeMarinis
 * 30 March 2012
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <sys/socket.h>
#include <arpa/inet.h>

#define PIPE_BUFFER_SIZE 128

// Happy macros for getting the read and write components of a pipe's fd array
#define pipe_read(x) (x[0])
#define pipe_write(x) (x[1])

struct layer_info
{
  // FDs for the thread's input/output pipes
  int in;
  int out;
};

// Globals
int fds[2];
char* str = "Hello world!\n";

struct layer_info net_send_info, net_recv_info;
struct layer_info dl_send_info, dl_recv_info;
struct layer_info phys_send_info, phys_recv_info;

// Prototypes
void *init_network_layer_send(void *info);
void *init_network_layer_recv(void *info);
void *init_data_link_layer_send(void *info);
void *init_data_link_layer_recv(void *info);
void *init_physical_layer_send(void *info);
void *init_physical_layer_recv(void *info);

int main(int argc, char *argv[])
{
  int i, rv;
  long thread_ret;

  pthread_t t_net_send, t_net_recv, t_dl_send, t_dl_recv, t_phys_send, t_phys_recv;

  // Declare pipes for communication to/from each layer
  int app_to_net[2], net_to_app[2];
  int net_to_dl[2], dl_to_net[2];
  int dl_to_phys[2], phys_to_dl[2];

  int num_pipes = 6;
  int *pipes[6] = {app_to_net, net_to_app, 
		net_to_dl, dl_to_net, 
		dl_to_phys, phys_to_dl};

  // Make our pipes
  for(i = 0; i < num_pipes; i++)
    if(pipe(pipes[i]))
      printf("Error creating pipe:  %s\n", strerror(errno));
  
  // Populate a layer info struct for each layer with their corresponding pipes
  net_send_info.in  = pipe_read(app_to_net);
  net_send_info.out = pipe_write(net_to_dl);
  net_recv_info.in  = pipe_read(dl_to_net);
  net_recv_info.out = pipe_write(net_to_app);
  
  dl_send_info.in  = pipe_read(net_to_dl);
  dl_send_info.out = pipe_write(dl_to_phys);
  dl_recv_info.in  = pipe_read(phys_to_dl);
  dl_recv_info.out = pipe_write(dl_to_net);

  phys_send_info.in = pipe_read(dl_to_phys);
  phys_recv_info.out = pipe_write(phys_to_dl);

  printf("Creating layer threads...\n");
  if((rv = pthread_create(&t_net_send, NULL, init_network_layer_send, (void*)(&net_send_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_net_recv, NULL, init_network_layer_recv, (void*)(&net_recv_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_dl_send, NULL, init_data_link_layer_send, (void*)(&dl_send_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_dl_recv, NULL, init_data_link_layer_recv, (void*)(&dl_recv_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_phys_send, NULL, init_physical_layer_send, (void*)(&phys_send_info))))
   printf("Error creating thread!\n");

  if((rv = pthread_create(&t_phys_recv, NULL, init_physical_layer_recv, (void*)(&phys_recv_info))))
    printf("Error creating thread!\n");

  // Send something down our pipe
  write(pipe_write(dl_to_phys), str, strlen(str) + 1);
  
  pthread_exit(NULL);
  exit(0);
}

void *init_physical_layer_send(void *info)
{
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;;
  char read_buffer[PIPE_BUFFER_SIZE];

  // Grab something to process
  printf("PHY:  Thread created!\n");
  bytes_read = read(fds->in, read_buffer, 128);
  printf("PHY:  Received string of length %d bytes:  %s", n, read_buffer);

  // Send it down to the next pipe
  write(fds->out, read_buffer, n);

  pthread_exit(NULL);
}

void *init_network_layer_send(void *info)
{
  pthread_exit(NULL);
}

void *init_network_layer_recv(void *info)
{
  pthread_exit(NULL);
}

void *init_data_link_layer_send(void *info)
{
  pthread_exit(NULL);
}

void *init_data_link_layer_recv(void *info)
{
  pthread_exit(NULL);
}

void *init_physical_layer_recv(void *info)
{
  pthread_exit(NULL);
}

