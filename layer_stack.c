/*
 * layer_stack.c
 * Nicholas DeMarinis
 * 30 March 2012
 */

#include <fcntl.h>

#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/socket.h>

#include "layer_stack.h"

// Prototypes
void *init_network_layer_send(void *info);
void *init_network_layer_recv(void *info);
void *init_data_link_layer(void *info);
void *init_data_link_layer_send(void *info);
void *init_data_link_layer_recv(void *info);
void *init_physical_layer_send(void *info);
void *init_physical_layer_recv(void *info);

enum frame_event wait_for_event(int net_fd, int phys_fd, char *buffer, int *bytes_read);

// Create space for all of the info we need to send to each thread in each stack
struct layer_stack stack_info[MAX_CLIENTS];

struct layer_info net_send_info, net_recv_info; // FDs for each end of the pipe for each layer
struct layer_info phys_send_info, phys_recv_info;
struct bidirectional_layer_info dl_info;

pthread_mutex_t net_dl_wire_lock;
pthread_mutex_t phys_dl_wire_lock;
int net_to_dl_frame_size = 0;
int phys_to_dl_frame_size = 0;

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

  pthread_t t_net_send, t_net_recv, t_dl, t_phys_send, t_phys_recv;

  // Declare pipes for communication to/from each layer
  // Since these are just integers, we literally copy them into the thread-specific
  // structs later
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
    else
      printf("Created pipe:  %d -> %d\n", pipes[i][0], pipes[i][1]);
  
  pipe_read(app_layer_pipes) = pipe_read(net_to_app);
  pipe_write(app_layer_pipes) = pipe_write(app_to_net);

  // Populate a layer info struct for each layer with their corresponding pipes
  net_send_info.in  = pipe_read(app_to_net);
  net_send_info.out = pipe_write(net_to_dl);
  net_recv_info.in  = pipe_read(dl_to_net);
  net_recv_info.out = pipe_write(net_to_app);
  
  dl_info.top_in = pipe_read(net_to_dl);
  dl_info.top_out = pipe_write(dl_to_net);
  dl_info.bottom_in = pipe_read(phys_to_dl);
  dl_info.bottom_out = pipe_write(dl_to_phys);

#if 0
  dl_send_info.in  = pipe_read(net_to_dl);
  dl_send_info.out = pipe_write(dl_to_phys);
  dl_recv_info.in  = pipe_read(phys_to_dl);
  dl_recv_info.out = pipe_write(dl_to_net);
#endif

  phys_send_info.in = pipe_read(dl_to_phys);
  phys_recv_info.out = pipe_write(phys_to_dl);

  // Connect the socket to the appropriate physical layer FDs.  
  phys_recv_info.in = clnt_sock;
  phys_send_info.out = clnt_sock;

  pthread_mutex_init(&net_dl_wire_lock, NULL);
  pthread_mutex_init(&phys_dl_wire_lock, NULL);

  printf("Creating layer threads...\n");
  if((rv = pthread_create(&t_net_send, NULL, init_network_layer_send, (void*)(&net_send_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_net_recv, NULL, init_network_layer_recv, (void*)(&net_recv_info))))
    printf("Error creating thread!\n");

#if 0
  if((rv = pthread_create(&t_dl_send, NULL, init_data_link_layer_send, (void*)(&dl_send_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_dl_recv, NULL, init_data_link_layer_recv, (void*)(&dl_recv_info))))
    printf("Error creating thread!\n");
#endif

  if((rv = pthread_create(&t_dl, NULL, init_data_link_layer, (void*)(&dl_info))))
    printf("Error creating thread!\n");
  
  if((rv = pthread_create(&t_phys_send, NULL, init_physical_layer_send, (void*)(&phys_send_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_phys_recv, NULL, init_physical_layer_recv, (void*)(&phys_recv_info))))
    printf("Error creating thread!\n");

  // TODO:  Handle thread sync/termination gracefully.  
  return 0;
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
      printf("PHY:  Sending %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Do any necessary processing here

      // Send it down to the next pipe, don't block
      if((bytes_sent = send(fds->out, read_buffer, bytes_read, 0)) <= 0)
	{
	  printf("PHY:  Read %d bytes: %s.  Socket was probably closed.  Terminating!\n", 
		 bytes_read, strerror(errno));
	  exit(2);
	}
    }

  pthread_exit(NULL);
}

void *init_network_layer_send(void *info)
{
  int bytes_read, bytes_written;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  printf("NET:  Thread created!\n");
  
  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      // Grab something to process
      bytes_read = read(fds->in, read_buffer, PIPE_BUFFER_SIZE);
      printf("NET:  Sending %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Send it down to the next pipe
      pthread_mutex_lock(&net_dl_wire_lock);
      bytes_written = write(fds->out, read_buffer, bytes_read);
      net_to_dl_frame_size += bytes_written;
      pthread_mutex_unlock(&net_dl_wire_lock);
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
      printf("NET:  Received %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Send it down to the next pipe
      write(fds->out, read_buffer, bytes_read);
    }
  
  pthread_exit(NULL);
}

#if 0
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
      printf("DLL:  Sending %d bytes:  %s\n", bytes_read, read_buffer);

#if 0
      // Build the frame
      f.type = FRAME_TYPE_FRAME;
      f.seq  = next_seq;
      f.checksum = 0xFFFF; // Just set this to something for now.
      f.eop = 0xFF; // We're only sending one frame for now.
      f.term = 0xFF;
  
      // Copy in the payload
      memcpy(f.payload, read_buffer, 
	     bytes_read > FRAME_PAYLOAD_SIZE ? FRAME_PAYLOAD_SIZE : bytes_read); 
 #endif
      // Send it down to the next pipe
      write(fds->out, read_buffer, bytes_read);
      //      next_seq++;
    }

  pthread_exit(NULL);
}
#endif 

void *init_data_link_layer(void *info)
{
  int bytes_read;
  enum frame_event event;
  struct bidirectional_layer_info *fds = (struct bidirectional_layer_info *)info;

  char read_buffer[MAX_FRAME_SIZE];

  // Make the incoming pipes nonblocking so we can continue if we don't send

  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      // Wait for one of our layers to tell us to do something.  
      while((event = wait_for_event(fds->top_in, fds->bottom_in, read_buffer, &bytes_read)) != NOP)

      switch(event)
	{
	case PHYSICAL_FRAME_READY: // Frame is going up, to the network layer
	  printf("DLL:  Got a frame from PHY of %d bytes.\n", bytes_read);
	  write(fds->top_out, read_buffer, bytes_read);
	  break;
	case NETWORK_FRAME_READY: // Frame is going down, to the physical layer
	  printf("DLL:  Got a frame from NET of %d bytes.\n", bytes_read);
	  write(fds->bottom_out, read_buffer, bytes_read);
	  break;
	default:
	  printf("DLL:  I don't know what's going on here!\n");
	  break;
	}
    }
}

#if 0
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
      printf("DLL:  Received %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Send it down to the next pipe
      write(fds->out, read_buffer, bytes_read);
    }

  pthread_exit(NULL);
}
#endif

void *init_physical_layer_recv(void *info)
{
  int bytes_read, bytes_written;
  struct layer_info *fds = (struct layer_info *)info;
  char read_buffer[PIPE_BUFFER_SIZE];

  printf("PHY:  Thread created!\n");

  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);

      // Try to receive, block if necessary
      // TODO:  Handle errors/terminating more gracefully.  
      if((bytes_read = recv(fds->in, read_buffer, PIPE_BUFFER_SIZE, 0)) <= 0) 
	{
	  printf("PHY:  Read %d bytes: %s.  Socket was probably closed.  Terminating!\n", 
		 bytes_read, strerror(errno));
	  exit(2);
	}
      else
	printf("PHY:  Received %d bytes:  %s\n", bytes_read, read_buffer);
      
      // Send it down to the next pipe
      pthread_mutex_lock(&phys_dl_wire_lock);
      bytes_written = write(fds->out, read_buffer, bytes_read);
      phys_to_dl_frame_size += bytes_written;
      pthread_mutex_unlock(&phys_dl_wire_lock);
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

enum frame_event wait_for_event(int net_fd, int phys_fd, char *buffer, int *b_read)
{
  int bytes_read;
  enum frame_event rv = NOP;
  
  // Try to read from our input pipe, but don't block if nothing's there
  pthread_mutex_lock(&net_dl_wire_lock);
  if(net_to_dl_frame_size)
    {
      bytes_read = read(net_fd, buffer, MAX_FRAME_SIZE);
      printf("EVENT:  Got into NET, read %d bytes.\n", bytes_read);
      net_to_dl_frame_size -= bytes_read;
      rv = NETWORK_FRAME_READY;
    }
  pthread_mutex_unlock(&net_dl_wire_lock);
  
  if(rv == NOP)
    {
      pthread_mutex_lock(&phys_dl_wire_lock);
      if(phys_to_dl_frame_size)
	{
	  bytes_read = read(phys_fd, buffer, MAX_FRAME_SIZE);
	  printf("EVENT:  Got into PHYS, read %d bytes.\n", bytes_read);
	  phys_to_dl_frame_size -= bytes_read;
	  rv = PHYSICAL_FRAME_READY;
	}
      pthread_mutex_unlock(&phys_dl_wire_lock);
    }

  // Write out how much we just wrote to the buffer
  *b_read = bytes_read;

  return rv;
}

