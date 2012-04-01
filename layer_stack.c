/*
 * layer_stack.c
 * Nicholas DeMarinis
 * 30 March 2012
 */

#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/socket.h>

#include "layer_stack.h"

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

int init_layer_stack(int clnt_sock, int *app_layer_pipes)
{
  int i, rv, bytes_read;
  long thread_ret;

  char read_buffer[PIPE_BUFFER_SIZE];
  
  pthread_t t_net_send, t_net_recv, t_dl_send, t_dl_recv, t_phys_send, t_phys_recv;

  // Declare pipes for communication to/from each layer
  int app_to_net[2], net_to_app[2];
  int net_to_dl[2], dl_to_net[2];
  int dl_to_phys[2], phys_to_dl[2];

  int num_pipes = 6;
  int *pipes[6] = {app_to_net, net_to_app, 
		net_to_dl, dl_to_net, 
		dl_to_phys, phys_to_dl};
  //int phys[2];

  // Make our pipes
  for(i = 0; i < num_pipes; i++)
    if(pipe(pipes[i]))
      printf("Error creating pipe:  %s\n", strerror(errno));
    else
      printf("Created pipe:  %d -> %d\n", pipes[i][0], pipes[i][1]);
  
  pipe_read(app_layer_pipes) = pipe_read(net_to_app);
  pipe_write(app_layer_pipes) = pipe_write(app_to_net);

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

  // Connect the socket to the appropriate physical layer FDs.  
  phys_recv_info.in = clnt_sock;
  phys_send_info.out = clnt_sock;

  // For now, cheat and connect the physical layers together
  //pipe(phys);
  //phys_send_info.out = pipe_write(phys);
  //phys_recv_info.in = pipe_read(phys);

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
  //printf("APP:  Sending string of length %d bytes:  %s\n", strlen(str) + 1, str);
  //write(pipe_write(app_to_net), str, strlen(str) + 1);

  // Read it back, after it passes through all of the pipes
  //bytes_read = read(pipe_read(net_to_app), read_buffer, PIPE_BUFFER_SIZE);
  //printf("APP:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);
  
  return 0;
  //pthread_exit(NULL);

}

void *init_physical_layer_send(void *info)
{
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  // Grab something to process
  printf("PHY:  Thread created!\n");
  printf("Got sockets: %d -> %d\n", fds->in, fds->out);
  bytes_read = read(fds->in, read_buffer, 128);
  printf("PHY:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);

  // Send it down to the next pipe
  //write(fds->out, read_buffer, bytes_read);
  send(fds->out, read_buffer, bytes_read, 0);

  pthread_exit(NULL);
}

void *init_network_layer_send(void *info)
{
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  // Grab something to process
  printf("NET:  Thread created!\n");
  printf("Got sockets: %d -> %d\n", fds->in, fds->out);
  bytes_read = read(fds->in, read_buffer, 128);
  printf("NET:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);

  // Send it down to the next pipe
  write(fds->out, read_buffer, bytes_read);

  pthread_exit(NULL);
}

void *init_network_layer_recv(void *info)
{ 
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  // Grab something to process
  printf("NET:  Thread created!\n");
  printf("Got sockets: %d -> %d\n", fds->in, fds->out);
  bytes_read = read(fds->in, read_buffer, 128);
  printf("NET:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);

  // Send it down to the next pipe
  write(fds->out, read_buffer, bytes_read);

  pthread_exit(NULL);
}

void *init_data_link_layer_send(void *info)
{
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  // Grab something to process
  printf("DLL:  Thread created!\n");
  printf("Got sockets: %d -> %d\n", fds->in, fds->out);
  bytes_read = read(fds->in, read_buffer, 128);
  printf("DLL:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);

  // Send it down to the next pipe
  write(fds->out, read_buffer, bytes_read);

  pthread_exit(NULL);
}

void *init_data_link_layer_recv(void *info)
{
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  // Grab something to process
  printf("DLL:  Thread created!\n");
  printf("Got sockets: %d -> %d\n", fds->in, fds->out);
  bytes_read = read(fds->in, read_buffer, PIPE_BUFFER_SIZE);
  printf("DLL:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);

  // Send it down to the next pipe
  write(fds->out, read_buffer, bytes_read);

  pthread_exit(NULL);
}

void *init_physical_layer_recv(void *info)
{
  int bytes_read, to_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];
  
  memset(read_buffer, 0, PIPE_BUFFER_SIZE);

  // Grab something to process
  printf("PHY:  Thread created!  Socket should be:  %d\n", fds->in);
  printf("Got sockets: %d -> %d\n", fds->in, fds->out);
  
  recv(fds->in, &to_read, sizeof(int), 0);
  printf("Read:  %d\n", to_read);
  
  bytes_read = recv(fds->in, read_buffer, to_read, 0);
  if(bytes_read <= 0)
    printf("PHY:  %s\n", strerror(errno));
  else
    printf("PHY:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);
    
  // Send it down to the next pipe
  write(fds->out, read_buffer, bytes_read);

  pthread_exit(NULL);
}

