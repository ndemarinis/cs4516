/*
 * layer_stack.c
 * Nicholas DeMarinis
 * 30 March 2012
 */

#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/socket.h>

#include "layer_stack.h"

// Prototypes
void *init_network_layer_send(void *info);
void *init_network_layer_recv(void *info);
void *init_data_link_layer_send(void *info);
void *init_data_link_layer_recv(void *info);
void *init_physical_layer_send(void *info);
void *init_physical_layer_recv(void *info);

// Globals
int fds[2];
char* str = "Hello world!\n";

// Create space for all of the info we need to send to each thread in each stack
struct layer_stack stack_info[MAX_CLIENTS];

struct layer_info net_send_info, net_recv_info; // FDs for each end of the pipe for each layer
struct layer_info dl_send_info, dl_recv_info;
struct layer_info phys_send_info, phys_recv_info;

pthread_mutex_t wire_lock;
pthread_cond_t wire_lock_cv;

/**
 * init_layer_stack:  Creates all layer threads and pipes to communicate between them.
 * This is accomplished by creating a thread for the sending and receiving portion of 
 * each layer--network, DL, and physical.  Pipes are created for communication 
 * between each layer.  
 * @author ndemarinis
 * @param clnt_num Client ID, provided by server
 * @param clnt_sock Socket descriptor for the accepted client
 * @param app_layer_pipes Pointer to array of two integers for pipes to application layer
 * 
 */
int init_layer_stack(int clnt_sock, int *app_layer_pipes)
{
  int i, rv;

  pthread_t t_net_send, t_net_recv, t_dl_send, t_dl_recv, t_phys_send, t_phys_recv;

  // Declare pipes for communication to/from each layer
  int app_to_net[2], net_to_app[2];
  int net_to_dl[2], dl_to_net[2];
  int dl_to_phys[2], phys_to_dl[2];

  int num_pipes = 6;
  int *pipes[6] = {app_to_net, net_to_app, 
		net_to_dl, dl_to_net, 
		dl_to_phys, phys_to_dl};
  //int phys[2]; // Hacky pipe to connect physical layers together for demonstration purposes

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

  // Initialize our mutex and condition variable
  pthread_mutex_init(&wire_lock, NULL);
  pthread_cond_init(&wire_lock_cv, NULL);

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

  return 0;
  //pthread_exit(NULL);

}

void *init_physical_layer_send(void *info)
{
  int bytes_read, bytes_sent;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  // Grab something to process
  printf("PHY:  Thread created!\n");

  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      // Get something to send, block if nothing.  
      bytes_read = read(fds->in, read_buffer, PIPE_BUFFER_SIZE);
      printf("PHY:  Sending string of length %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Do any necessary processing here

      //      pthread_mutex_lock(&wire_lock); // Grab the wire

      // Send it down to the next pipe, don't block
      if((bytes_sent = send(fds->out, read_buffer, bytes_read, 0)) <= 0)
	{
	  printf("PHY:  Read %d bytes: %s.  Socket was probably closed.  Terminating!\n", 
		 bytes_read, strerror(errno));
	  exit(2);
	}
	
      //pthread_cond_signal(&wire_lock_cv); // Let the recv thread know we're done, blocking or not

      //pthread_mutex_unlock(&wire_lock); // Let the receiver keep checking.  
    }

  pthread_exit(NULL);
}

void *init_network_layer_send(void *info)
{
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  printf("NET:  Thread created!\n");
  
  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      // Grab something to process
      bytes_read = read(fds->in, read_buffer, PIPE_BUFFER_SIZE);
      printf("NET:  Sending string of length %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Send it down to the next pipe
      write(fds->out, read_buffer, bytes_read);
    }
  
  pthread_exit(NULL);
}

void *init_network_layer_recv(void *info)
{ 
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  printf("NET:  Thread created!\n");
  
  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      // Grab something to process  
      bytes_read = read(fds->in, read_buffer, PIPE_BUFFER_SIZE);
      printf("NET:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Send it down to the next pipe
      write(fds->out, read_buffer, bytes_read);
    }
  
  pthread_exit(NULL);
}

void *init_data_link_layer_send(void *info)
{
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];


  printf("DLL:  Thread created!\n");

  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      // Grab something to process
      bytes_read = read(fds->in, read_buffer, PIPE_BUFFER_SIZE);
      printf("DLL:  Sending string of length %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Send it down to the next pipe
      write(fds->out, read_buffer, bytes_read);
    }

  pthread_exit(NULL);
}

void *init_data_link_layer_recv(void *info)
{
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  printf("DLL:  Thread created!\n");

  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      // Grab something to process
      bytes_read = read(fds->in, read_buffer, PIPE_BUFFER_SIZE);
      printf("DLL:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Send it down to the next pipe
      write(fds->out, read_buffer, bytes_read);
    }

  pthread_exit(NULL);
}

void *init_physical_layer_recv(void *info)
{
  int bytes_read, to_read;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  printf("PHY:  Thread created!\n");

  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      pthread_mutex_lock(&wire_lock); // Grab the wire while we need to receive

      // Try to receive without blocking.  If we get nothing, release the lock and
      // let the sending thread go ahead and check
      //      while(((bytes_read = recv(fds->in, read_buffer, 128, MSG_DONTWAIT)) == -1) &&
      //	    (errno == EWOULDBLOCK || errno == EAGAIN))
      //	pthread_cond_wait(&wire_lock_cv, &wire_lock);

      // Try to receive, block if necessary
      // TODO:  Handle errors/terminating more gracefully.  
      if((bytes_read = recv(fds->in, read_buffer, PIPE_BUFFER_SIZE, 0)) <= 0) 
	{
	  printf("PHY:  Read %d bytes: %s.  Socket was probably closed.  Terminating!\n", 
		 bytes_read, strerror(errno));
	  exit(2);
	}
      else
	printf("PHY:  Received string of length %d bytes:  %s\n", bytes_read, read_buffer);

      pthread_mutex_unlock(&wire_lock); // When we're done receiving, release the wire.
      
      // Send it down to the next pipe
      write(fds->out, read_buffer, bytes_read);
    }

  pthread_exit(NULL);
}

/**
 * die_with_error:  Print out a string and to stderr and then exit
 * @author ndemarinis
 */
void die_with_error(char *msg)
{
  fprintf(stderr, "%s\n", msg);
  exit(2);
}

