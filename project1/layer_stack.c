/*
 * layer_stack.c
 * Nicholas DeMarinis
 * 30 March 2012
 */

#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/socket.h>

#include "layer_stack.h"


#define get_frame_type(x) (x.type == FRAME_TYPE_FRAME ? "FRAME" : "ACK")

// Prototypes
void *init_network_layer_send(void *info);
void *init_network_layer_recv(void *info);
void *init_data_link_layer(void *info);
void *init_data_link_layer_send(void *info);
void *init_data_link_layer_recv(void *info);
void *init_physical_layer_send(void *info);
void *init_physical_layer_recv(void *info);

void *init_layer_stack(void *info);
enum frame_event wait_for_event(struct layer_stack *stack, int net_fd, int phys_fd, 
				struct frame_window *window, int frames_buffered, int expected, 
				char *buffer, int *bytes_read);
void send_frame(int fd, pid_t id, 
		struct frame_window *pkt_buffer, uint8_t frame_type, uint8_t seq_num);
uint16_t compute_checksum(struct frame *frame);
void print_layer_stack_statistics(struct layer_stack *stack);


// Define a timeval for the maximum timeout
struct timeval max_wait_time;

struct layer_stack *create_layer_stack(int clnt_sock, pid_t id, int *app_layer_pipes)
{
  struct stack_create_info *info = (struct stack_create_info *)malloc(sizeof(struct stack_create_info));
  struct layer_stack *s_info = (struct layer_stack *)malloc(sizeof(struct layer_stack));
  pthread_t th;

  // Populate the struct we're sending to the stack creation thread
  info->clnt_sock = clnt_sock;
  info->app_layer_pipes = app_layer_pipes;
  info->stack = s_info;
  info->id = id;
  
  // Create the thread, which creates the layer threads
  pthread_create(&th, NULL, init_layer_stack, (void*)info);

  return s_info;
}

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
void *init_layer_stack(void *info)
{
  int i, rv;

  struct stack_create_info *in = (struct stack_create_info *)info;

  pthread_t t_net_send, t_net_recv, t_dl, t_phys_send, t_phys_recv;
  pthread_attr_t attr;

  // Make a new set of statistics vars for this stack instance
  struct layer_stack *s_info = in->stack;

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

  // Fill in the client's ID
  s_info->id = in->id;
  
  dprintf(DID_INFO, "Initializing layer stack for client %d\n", s_info->id);

  // Make our pipes
  for(i = 0; i < num_pipes; i++)
    if(pipe(pipes[i]))
      {
	dprintf(DID_WARN, "%d:  Error creating pipe:  %s\n", s_info->id, strerror(errno));
      }
    else
      dprintf(DID_INFO, "%d:  Created pipe:  %d -> %d\n", s_info->id, pipes[i][0], pipes[i][1]);
  
  pipe_read(in->app_layer_pipes) = pipe_read(net_to_app);
  pipe_write(in->app_layer_pipes) = pipe_write(app_to_net);

  // Populate a layer info struct for each layer with their corresponding pipes
  s_info->net_send_info.in  = pipe_read(app_to_net);
  s_info->net_send_info.out = pipe_write(net_to_dl);
  s_info->net_recv_info.in  = pipe_read(dl_to_net);
  s_info->net_recv_info.out = pipe_write(net_to_app);
  
  s_info->dl_info.top_in = pipe_read(net_to_dl);
  s_info->dl_info.top_out = pipe_write(dl_to_net);
  s_info->dl_info.bottom_in = pipe_read(phys_to_dl);
  s_info->dl_info.bottom_out = pipe_write(dl_to_phys);

  s_info->phys_send_info.in = pipe_read(dl_to_phys);
  s_info->phys_recv_info.out = pipe_write(phys_to_dl);

  // Connect the socket to the appropriate physical layer FDs.  
  s_info->phys_recv_info.in = in->clnt_sock;
  s_info->phys_send_info.out = in->clnt_sock;

  // Define our max timeout
  max_wait_time.tv_sec = 0;
  max_wait_time.tv_usec = FRAME_TIMEOUT_MS * 1000;

  // Create mutexes for locking each end of the pipes to the DLL
  // We need this to maintain counters for the number of bytes in the pipe
  // so the calls to them can be "nonblocking"
  pthread_mutex_init(&(s_info->net_dl_wire_lock), NULL);
  pthread_mutex_init(&(s_info->phys_dl_wire_lock), NULL);

  s_info->net_to_dl_frame_size = 0;
  s_info->phys_to_dl_frame_size = 0;

  // Give each thread a pointer to the layer stack so it can update the statistice
  // This is possibly the ugliest thing I have ever done.  
  s_info->net_send_info.stack = s_info;
  s_info->net_recv_info.stack = s_info;
  s_info->dl_info.stack = s_info;
  s_info->phys_send_info.stack = s_info;
  s_info->phys_recv_info.stack = s_info;

  // Zero out all of the statistics
  s_info->total_frames_sent = 0;
  s_info->total_acks_sent = 0;
  s_info->total_good_frames_sent = 0;
  s_info->total_good_frames_recvd = 0;
  s_info->total_good_acks_sent = 0;
  s_info->total_good_acks_recvd = 0;
  s_info->total_bad_acks_recvd = 0;
  s_info->total_dup_frames_recvd = 0;
  s_info->total_bad_frames_recvd = 0;

  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  dprintf(DID_INFO, "%d:  Creating layer threads...\n", s_info->id);
  if((rv = pthread_create(&t_net_send, NULL, init_network_layer_send, 
			  (void*)&(s_info->net_send_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_net_recv, NULL, init_network_layer_recv, 
			  (void*)&(s_info->net_recv_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_dl, NULL, init_data_link_layer, 
			  (void*)&(s_info->dl_info))))
    printf("Error creating thread!\n");
  
  if((rv = pthread_create(&t_phys_send, NULL, init_physical_layer_send, 
			  (void*)&(s_info->phys_send_info))))
    printf("Error creating thread!\n");

  if((rv = pthread_create(&t_phys_recv, &attr, init_physical_layer_recv, 
			  (void*)&(s_info->phys_recv_info))))
    printf("Error creating thread!\n");

  // Just wait for the thread holding the socket.  When the app layer closes it, we're done.  
  pthread_join(t_phys_recv, NULL);

  print_layer_stack_statistics(s_info);

  printf("Client %d Successfully terminated!\n", s_info->id);  


  pthread_exit(NULL);
}

void *init_physical_layer_send(void *info)
{
  int bytes_read, bytes_sent;
  struct layer_info *fds = (struct layer_info *)info;

  char *buffer_to_send;
  int len_to_send = 0;
  
  struct frame frame_out;
  struct ack ack_out;
  // Grab something to process
  printf("%d:  PHY:  Thread created!\n", (fds->stack)->id);

  for(;;)
    {
      memset(&frame_out, 0, sizeof(struct frame));
      memset(&ack_out, 0, sizeof(struct ack));

      // Get something to send, block if nothing.  
      if((bytes_read = read(fds->in, &frame_out, sizeof(struct frame))) <= 0)
	{
	  printf("%d:  PHY:  Read %d bytes from APP.  Pipe was probably closed.  Terminating!\n", 
		 (fds->stack)->id, bytes_read);
	  break;
	}

      if(frame_out.type == FRAME_TYPE_FRAME) 
	{
	  if(!((++((fds->stack)->total_frames_sent)) % FRAME_KILL_EVERY_N_FRAMES))
	    {
	      dprintf(DID_DLL_INFO, "%d:  PHY:  Injecting error in frame %d, %d sent\n", 
		      (fds->stack)->id, frame_out.seq, (fds->stack)->total_frames_sent);
	      frame_out.checksum ^= FRAME_KILL_MAGIC; // Flip a single bit of the checksum
	    }
	  else
	    (fds->stack)->total_good_frames_sent++;
	  
	  // We're sending a whole frame, so just set the buffer accordingly
	  buffer_to_send = (char *)&frame_out;
	  len_to_send = sizeof(struct frame);
	}
      
      if(frame_out.type == FRAME_TYPE_ACK)
	{
	  if(!(++((fds->stack)->total_acks_sent) % FRAME_KILL_EVERY_N_ACKS))
	    {
	      dprintf(DID_DLL_INFO, "%d:  PHY:  Injecting error in ACK %d\n", 
		      (fds->stack)->id, frame_out.seq);
	      frame_out.checksum ^= FRAME_KILL_MAGIC; // Flip a single bit of the checksum
	    }
	  else
	    (fds->stack)->total_good_acks_sent++;
	  
	  // Populate the ACK struct
	  ack_out.type = frame_out.type;
	  ack_out.seq = frame_out.seq;
	  ack_out.checksum = frame_out.checksum;

	  buffer_to_send = (char *)&ack_out;
	  len_to_send = sizeof(struct ack);
	}

      dprintf(DID_INFO, "%d:  PHY:  Sending %s of %d bytes\n", 
	      (fds->stack)->id, get_frame_type(frame_out), len_to_send);

      // Send it down to the next pipe, don't block
      if((bytes_sent = send(fds->out, buffer_to_send, len_to_send, 0)) <= 0)
	{
	  printf("%d:  PHY:  Sent %d bytes: %s.  Socket was probably closed.  Terminating!\n", 
		 (fds->stack)->id, bytes_read, strerror(errno));
	  break;
	}
    }
  
  dprintf(DID_INFO, "%d:  PHY:  Send thread entered termination handler!\n", (fds->stack)->id);
  close(fds->in);
  close(fds->out);
  pthread_exit(NULL);
}

void *init_network_layer_send(void *info)
{
  int bytes_read, bytes_written, total_pkt_len, term = 0;
  struct layer_info *fds = (struct layer_info *)info;

  // Since the payload is at most 256 bytes, it will comprise at most two frames
  struct packet pkt_in;
  struct packet_segment s1, s2; 

  char *pkt = (char *)&pkt_in;

  dprintf(DID_INFO, "%d:  NET:  Thread created!\n", (fds->stack)->id);
  
  for(;;)
    {
      memset(&pkt_in, 0, sizeof(struct packet));
      memset(&s1, 0, sizeof(struct packet_segment));
      memset(&s2, 0, sizeof(struct packet_segment));

      // Grab something to process
      if((bytes_read = read(fds->in, &pkt_in, sizeof(struct packet))) <= 0)
	break;
      dprintf(DID_INFO, "%d:  NET:  Read packet of %d bytes with payload of %d bytes\n", 
	     (fds->stack)->id, bytes_read, pkt_in.length + 1);

      total_pkt_len = pkt_in.length + 1 + 
	sizeof(pkt_in.seq_num) + sizeof(pkt_in.opcode) + sizeof(pkt_in.length);

      if(pkt_in.length + 1 <= FRAME_PAYLOAD_SIZE)
	{
	  dprintf(DID_INFO, "%d:  NET:  Constructed segment 1 of length %d bytes with payload of %d bytes.\n", 
		 (fds->stack)->id, total_pkt_len, pkt_in.length);
	  memcpy(s1.payload, &pkt_in, total_pkt_len);
	  s1.length = total_pkt_len;
	  s1.end_of_pkt = FRAME_IS_EOP;
	}
      else // We need >1 frame
	{
	  dprintf(DID_INFO, "%d:  NET:  Constructed segment 1 with payload of %d bytes.\n", 
		 (fds->stack)->id, FRAME_PAYLOAD_SIZE);

	  memcpy(s1.payload, &pkt_in, FRAME_PAYLOAD_SIZE);
	  s1.length = FRAME_PAYLOAD_SIZE;
	  s1.end_of_pkt = FRAME_NOT_EOP;
	  	  
	  dprintf(DID_INFO, "%d:  NET:  Constructed segment 2 with payload of %d bytes.\n", 
		 (fds->stack)->id, total_pkt_len - FRAME_PAYLOAD_SIZE);

	  memcpy(s2.payload, (pkt + FRAME_PAYLOAD_SIZE), 
			      (total_pkt_len - FRAME_PAYLOAD_SIZE));
	  
	  s2.length = total_pkt_len - FRAME_PAYLOAD_SIZE;
	  s2.end_of_pkt = FRAME_IS_EOP;
	}
      
      // Send it down to the next pipe
      pthread_mutex_lock(&((fds->stack)->net_dl_wire_lock));
      
      bytes_written = write(fds->out, &s1, sizeof(struct packet_segment));
      (fds->stack)->net_to_dl_frame_size += bytes_written;
      dprintf(DID_INFO, "%d:  NET:  Sent first segment of %d bytes\n", 
	      (fds->stack)->id, bytes_written);

      if(pkt_in.length > FRAME_PAYLOAD_SIZE) // Send the second frame, if necessary
	{
	  if((bytes_written = write(fds->out, &s2, sizeof(struct packet_segment))) <= 0)
	    {
	      term = 1; // Flag that we need to terminate after we give up the mutex.  
	      (fds->stack)->net_to_dl_frame_size = -1;
	    }
	  else
	    {
	      (fds->stack)->net_to_dl_frame_size += bytes_written;
	      dprintf(DID_INFO, "%d:  NET:  Sent second segment of %d bytes\n", 
		      (fds->stack)->id, bytes_written);
	    }
	}

      pthread_mutex_unlock(&((fds->stack)->net_dl_wire_lock));
      
      if(term)
	break;
    }
  
  dprintf(DID_INFO, "%d:  NET:  Send thread entered termination handler!\n", 
	  (fds->stack)->id);

  pthread_mutex_lock(&((fds->stack)->net_dl_wire_lock));
  (fds->stack)->net_to_dl_frame_size = -1;
  pthread_mutex_unlock(&((fds->stack)->net_dl_wire_lock));

  close(fds->in);
  close(fds->out);
  pthread_exit(NULL);
}

void *init_network_layer_recv(void *info)
{ 
  int bytes_read;
  struct layer_info *fds = (struct layer_info *)info;

  struct packet pkt_out;
  char *pkt = (char *)&pkt_out;

  // Since the payload is at most 256 bytes, it will comprise at most two frames
  struct frame f1, f2;

  dprintf(DID_INFO, "%d:  NET:  Thread created!\n", (fds->stack)->id);
  
  for(;;)
    {
      memset(&pkt_out, 0, sizeof(struct packet));
      memset(&f1, 0, sizeof(struct frame));
      memset(&f2, 0, sizeof(struct frame));

      // Grab a segment to process
      if((bytes_read = read(fds->in, &f1, sizeof(struct frame))) <= 0)
	break;
      dprintf(DID_INFO, "%d:  NET:  Received frame of %d bytes with payload of %d bytes\n", 
	      (fds->stack)->id, bytes_read, f1.length);

      if(f1.end_of_pkt == FRAME_NOT_EOP)
	{
	  bytes_read = read(fds->in, &f2, sizeof(struct frame));
	  dprintf(DID_INFO, "%d:  NET:  Received second frame of %d bytes with payload of %d bytes\n", 
		 (fds->stack)->id, bytes_read, f2.length);
	}
      
      // Copy the segments into our packet
      memcpy(&pkt_out, f1.payload, f1.length);

      if(f1.end_of_pkt == FRAME_NOT_EOP)
	{
	  dprintf(DID_INFO, "%d:  NET:  Appending second frame of length %d bytes to packet after %d bytes\n", 
		 (fds->stack)->id, f2.length, f1.length);
	  memcpy(pkt + f1.length, &(f2.payload), f2.length);
	}

      // Send it down to the next pipe
      if(write(fds->out, &pkt_out, sizeof(struct packet)) <= 0)
	break;
    }

  dprintf(DID_INFO, "%d:  NET:  Recv thread entered termination handler!\n", (fds->stack)->id);
  close(fds->in);
  close(fds->out);
  pthread_exit(NULL);
}

void *init_data_link_layer(void *info)
{
  int i, bytes_read;
  enum frame_event event;
  struct bidirectional_layer_info *fds = (struct bidirectional_layer_info *)info;

  char read_buffer[PIPE_BUFFER_SIZE]; //  Buffer to grab frame from physical or network layer
  
  struct packet_segment *recvd_segment;
  struct frame *recvd_frame;

  uint8_t next_frame_to_send = 0;
  uint8_t frames_buffered = 0;
  uint8_t frame_expected = 0;
  uint8_t ack_expected = 0;

  // Our buffer of outstanding frames waiting for ACKs
  struct frame_window packet_buffer[MAX_SEQ + 1];

  // Maintain as many checksums as our window size to see if we've received any duplicate frames
  uint16_t last_recvd_frames[MAX_SEQ + 1]; 

  memset(packet_buffer, 0, (MAX_SEQ + 1)*sizeof(struct frame_window));
  dprintf(DID_INFO, "%d:  DLL:  Packet size:  %ld, Frame size:  %ld, Payload size:  %d\n", 
	 (fds->stack)->id, sizeof(struct packet), sizeof(struct frame), FRAME_PAYLOAD_SIZE);

  for(;;)
    {
      memset(read_buffer, 0, PIPE_BUFFER_SIZE);
      bytes_read = 0;

      // Wait for one of our layers to tell us to do something.  
      while((event = wait_for_event(fds->stack, fds->top_in, fds->bottom_in, packet_buffer, 
				    frames_buffered, ack_expected, read_buffer, &bytes_read)) != NOP)

      switch(event)
	{
	case PIPE_ERROR:
	  dprintf(DID_INFO, "%d:  DLL:  Entered termination handler!\n", (fds->stack)->id);
	  close(fds->top_in);
	  close(fds->top_out);
	  close(fds->bottom_in);
	  close(fds->bottom_out);
	  pthread_exit(NULL);
	  break;
	case NETWORK_FRAME_READY: // We just received a frame, 
	  dprintf(DID_DLL_INFO, "%d:  DLL:  Got a frame %d from NET of %d bytes with %d currently buffered.\n", 
		 (fds->stack)->id, next_frame_to_send, bytes_read, frames_buffered);
	  
	  recvd_segment = (struct packet_segment *)read_buffer;

	  // Put the frame in our sliding window
	  memcpy(packet_buffer[next_frame_to_send].payload, recvd_segment->payload, 
		 FRAME_PAYLOAD_SIZE);
	  packet_buffer[next_frame_to_send].length = recvd_segment->length;
	  packet_buffer[next_frame_to_send].end_of_pkt = recvd_segment->end_of_pkt;
	  frames_buffered++;

	  // Send it down to the physical layer
	  send_frame(fds->bottom_out, (fds->stack)->id, 
		     packet_buffer, FRAME_TYPE_FRAME, next_frame_to_send);
	  next_frame_to_send++;

	  break;

	case PHYSICAL_FRAME_READY: // Frame is going up to the network layer
	  recvd_frame = (struct frame *)read_buffer; // We just received a frame
	  dprintf(DID_DLL_INFO, "%d:  DLL:  Got a frame from PHY of %d bytes with payload of %d bytes\n", 
		 (fds->stack)->id, bytes_read, recvd_frame->length);

	  // If the frame we just received was what we wanted
	  if(recvd_frame->type == FRAME_TYPE_FRAME)
	    {
	      if(recvd_frame->seq == frame_expected)
		{
		  dprintf(DID_DLL_INFO, "%d:  DLL:  Frame %d was expected frame %d\n", 
			  (fds->stack)->id, recvd_frame->seq, frame_expected);

		  // Record its checksum so we can check for duplicates
		  last_recvd_frames[frame_expected] = recvd_frame->checksum;
		  
		  // Update the good frame counter
		  (fds->stack)->total_good_frames_recvd++;


		  // Send an ACK for that frame
		  send_frame(fds->bottom_out, (fds->stack)->id, 
			     packet_buffer, FRAME_TYPE_ACK, frame_expected);
		  
		  // Send the packet over to the network layer
		  write(fds->top_out, recvd_frame, sizeof(struct frame));
		  frame_expected++;
		}
	      else // We may have received a duplicate frame
		{
		  dprintf(DID_DLL_INFO, "%d:  DLL:  Received unexpected frame %d, was expecting %d\n", 
			 (fds->stack)->id, recvd_frame->seq, frame_expected);

		  // If we find it in our buffer of last few checksums, it's a duplicate, so ACK it again
		  for(i = 0; i < MAX_SEQ + 1; i++)
		    if(recvd_frame->checksum == last_recvd_frames[i])
		      {
			(fds->stack)->total_dup_frames_recvd++;
			dprintf(DID_DLL_INFO, "%d:  DLL:  Sending duplicate ACK for frame %d\n", 
				(fds->stack)->id, recvd_frame->seq);
			send_frame(fds->bottom_out, (fds->stack)->id, 
				   packet_buffer, FRAME_TYPE_ACK, recvd_frame->seq);
		      }
		}
	    }
	  // If it wasn't, it was probably an ACK
	  else if(recvd_frame->type == FRAME_TYPE_ACK && recvd_frame->seq == ack_expected) 
	    {
	      // Update the window, clear the timer, and update the statistics
	      frames_buffered--;
	      timerclear(&(packet_buffer[ack_expected].time_sent)); // Reset (clear) our timer
	      ack_expected++; // Shrink our buffer accordingly

	      (fds->stack)->total_good_acks_recvd++;

	      dprintf(DID_DLL_INFO, "%d:  DLL:  Received ACK for frame %d, now %d frames in buffer\n", 
		     (fds->stack)->id, recvd_frame->seq, frames_buffered);

	    }
	  else // If not, drop the frame	  
	    dprintf(DID_DLL_INFO, "%d:  DLL:  Dropped %X frame %d, expected seq %d, ACK %d, %d frames in buffer, next is %d\n",
		   (fds->stack)->id, recvd_frame->type, recvd_frame->seq, frame_expected, ack_expected, frames_buffered, next_frame_to_send);
	  break;

	case CHECKSUM_ERROR:
	  recvd_frame = (struct frame *)read_buffer; // We just received a frame, albeit a bad one
	  dprintf(DID_DLL_INFO, "%d:  DLL:  Got checksum error for %d, expecting seq %d, ACK %d, %d buffered, next %d, dropping frame.\n", 
		 (fds->stack)->id, recvd_frame->type, frame_expected, ack_expected, frames_buffered, next_frame_to_send);
	  
	  // Update our counters
	  if(recvd_frame->type == FRAME_TYPE_FRAME)
	    (fds->stack)->total_bad_frames_recvd++;
	  else
	    (fds->stack)->total_bad_acks_recvd++;
	  break;
	  
	case TIME_OUT: // We lost one
	  dprintf(DID_DLL_INFO, "%d:  DLL:  Caught timeout for frame %d\n", (fds->stack)->id, ack_expected);
	  next_frame_to_send = ack_expected; // Start retransmitting from there
	  for(i = 1; i <= frames_buffered; i++)
	    {
	      // Resend the frame
	      send_frame(fds->bottom_out, (fds->stack)->id, 
			 packet_buffer, FRAME_TYPE_FRAME, next_frame_to_send);
	      next_frame_to_send++;
	    }
	  break;
	default:
	  dprintf(DID_WARN, "%d:  DLL:  I don't know what's going on here!  Expected %d, %d in buffer, next is %d, waiting for ack %d\n", (fds->stack)->id, frame_expected, frames_buffered, next_frame_to_send, ack_expected);

	}
    }

  // Die.
  pthread_exit(NULL);
}


void *init_physical_layer_recv(void *info)
{
  int bytes_read, bytes_written;
  struct layer_info *fds = (struct layer_info *)info;

  uint8_t type_recvd;
  char *recv_buffer;
  int bytes_to_read = 0, term = 0;
 
  struct frame frame_in;
  struct ack ack_in;

  dprintf(DID_INFO, "%d:  PHY:  Thread created!\n", (fds->stack)->id);

  for(;;)
    {
      // Clear out buffers
      type_recvd = 0;
      memset(&frame_in, 0, sizeof(struct frame));
      memset(&ack_in, 0, sizeof(struct ack));

      // Read one byte without removing it from the queue, just to check the type
      // Based on this, we know how much to read next
      if((bytes_read = recv(fds->in, &type_recvd, sizeof(uint8_t), MSG_PEEK)) <= 0)
	{
	  dprintf(DID_INFO, "%d:  PHY:  Read %d bytes:  %s.  Terminating!\n", 
		  (fds->stack)->id, bytes_read, strerror(errno));
	  break;
	}

      // So, based on that, set the appropriate struct and length to read
      if(type_recvd == FRAME_TYPE_FRAME)
	{
	  recv_buffer = (char *)&frame_in;
	  bytes_to_read = sizeof(struct frame);
	}
      else if(type_recvd == FRAME_TYPE_ACK)
	{
	  recv_buffer = (char *)&ack_in;
	  bytes_to_read = sizeof(struct ack);
	}
      else
	dprintf(DID_INFO, "%d:  PHY:  NO IDEA what I just recvd.\n", (fds->stack)->id);

      // ... and then try to read it from the socket.  
      if((bytes_read = recv(fds->in, recv_buffer, bytes_to_read, 0)) <= 0) 
	{
	  dprintf(DID_INFO, "%d:  PHY:  Read %d bytes: %s.  Socket was probably closed.  Terminating!\n", 
		  (fds->stack)->id, bytes_read, strerror(errno));
	  break;
	}
     
      // So now package the frame/ack for transmission to the DLL
      // For a frame, this means nothing; for an ACK, we need to load it into the frame
      // struct to keep the pipes happy
      if(type_recvd == FRAME_TYPE_FRAME)
	{
	  dprintf(DID_INFO, "%d:  PHY:  Received frame of %d bytes with payload of %d bytes\n", 
	       (fds->stack)->id, bytes_read, frame_in.length);

	  memcpy(&frame_in, recv_buffer, sizeof(struct frame));
	}
      else if(type_recvd == FRAME_TYPE_ACK)
	{
	  dprintf(DID_INFO, "%d:  PHY:  Received ACK %d of %d bytes\n", 
		  (fds->stack)->id, ack_in.seq, bytes_read);
	  frame_in.type = ack_in.type;
	  frame_in.seq = ack_in.seq;
	  frame_in.checksum = ack_in.checksum;
	}
      else
	dprintf(DID_INFO, "%d:  PHY:  NO IDEA what I just received.\n", (fds->stack)->id);
      
      // Send it down to the next pipe
      pthread_mutex_lock(&((fds->stack)->phys_dl_wire_lock));
      if((bytes_written = write(fds->out, &frame_in, sizeof(struct frame))) <= 0)
	{
	  (fds->stack)->phys_to_dl_frame_size = -1;
	  term = 1; // Flag that we need to terminate after we give up the mutex
	}
      else
	(fds->stack)->phys_to_dl_frame_size += bytes_written;
      
      pthread_mutex_unlock(&((fds->stack)->phys_dl_wire_lock));

      if(term)
	break;
    }

  dprintf(DID_INFO, "%d:  PHY:  Recv thread entered termination handler!\n", (fds->stack)->id);

  pthread_mutex_lock(&((fds->stack)->phys_dl_wire_lock));
  (fds->stack)->phys_to_dl_frame_size = -1;
  pthread_mutex_unlock(&((fds->stack)->phys_dl_wire_lock));

  close(fds->in);
  close(fds->out);
  pthread_exit(NULL);
}

enum frame_event wait_for_event(struct layer_stack *stack, int net_fd, int phys_fd, 
				struct frame_window *window, int frames_buffered, 
				int expected, char *buffer, int *b_read)
{
  int bytes_read;
  uint16_t checksum;
  enum frame_event rv = NOP;
  
  struct frame *recvd_frame = (struct frame *)buffer;
  struct timeval curr_time, diff_time;

  // Try to read from our input pipe, but don't block if nothing's there
  pthread_mutex_lock(&(stack->net_dl_wire_lock));

  if(stack->net_to_dl_frame_size == -1) // Check if the network layer has flagged that we should terminate
    rv = PIPE_ERROR;
  else if(stack->net_to_dl_frame_size && frames_buffered < SLIDING_WINDOW_SIZE)
    {
      dprintf(DID_DLL_INFO, "%d:  DLL_EVENT:  NET has %d bytes for us\n", 
	      stack->id, stack->net_to_dl_frame_size);
      if((bytes_read = read(net_fd, buffer, sizeof(struct packet_segment))) <= 0)
	rv = PIPE_ERROR;
      else
	{
	  dprintf(DID_DLL_INFO, "%d:  DLL_EVENT:  Got into NET, read %d bytes with %d buffered.\n", 
		  stack->id, bytes_read, frames_buffered);
	  stack->net_to_dl_frame_size -= bytes_read;
	  rv = NETWORK_FRAME_READY;
	}
    }
  pthread_mutex_unlock(&(stack->net_dl_wire_lock));
 
  // Try to read from the physical layer pipe, without blocking, if that fails
  if(rv == NOP)
    {
      pthread_mutex_lock(&(stack->phys_dl_wire_lock));
      if(stack->phys_to_dl_frame_size == -1)
	rv = PIPE_ERROR;
      else if(stack->phys_to_dl_frame_size)
	{
	  if((bytes_read = read(phys_fd, buffer, sizeof(struct frame))) <= 0)
	    rv = PIPE_ERROR;
	    else
	      {
		dprintf(DID_DLL_INFO, "%d:  DLL_EVENT:  Got into PHYS, read %d bytes.\n", 
			stack->id, bytes_read);
		stack->phys_to_dl_frame_size -= bytes_read;
		rv = PHYSICAL_FRAME_READY;
	      }
	}
      pthread_mutex_unlock(&(stack->phys_dl_wire_lock));
      
      if(rv == PHYSICAL_FRAME_READY) // If our call just found a frame
	{
	  // Verify the frame we just received matches the checksum
	  // If it's an ACK, the "checksum" is the sequence number
	  checksum = (recvd_frame->type == FRAME_TYPE_FRAME) ? 
	    compute_checksum(recvd_frame) : recvd_frame->seq;
	  dprintf(DID_DLL_INFO, "%d:  DLL_EVENT:  Got checksum:  %04X vs %04X\n", 
		  stack->id, checksum, recvd_frame->checksum);
	  if(checksum != recvd_frame->checksum)
	    rv = CHECKSUM_ERROR;
	}
    }

  // If that fails, check the timers for timeouts
  if(rv == NOP)
    {
      gettimeofday(&curr_time, NULL);
      if(timerisset(&(window[expected].time_sent))) // Just check the one timer for now
	{
	  timersub(&curr_time, &(window[expected].time_sent), &diff_time);
	  if(timercmp(&diff_time, &max_wait_time, >))
	    {
	      dprintf(DID_DLL_INFO, "%d:  DLL_EVENT:  Found TIMEOUT for frame %d with %d buffered.\n", 
		     stack->id, expected, frames_buffered);
	      timerclear(&(window[expected].time_sent));
	      rv = TIME_OUT;
	    }
	}
    }

  // Write out how much we just wrote to the buffer
  *b_read = bytes_read;

  return rv;
}

void send_frame(int fd, pid_t id, 
		struct frame_window *pkt_buffer, uint8_t frame_type, uint8_t seq_num)
{
  struct frame out;
  
  // Clear our frame for safety
  memset(&out, 0, sizeof(struct frame));

  // Populate the frame
  out.type = frame_type;
  out.seq = seq_num;
  out.length = (frame_type == FRAME_TYPE_FRAME) ? pkt_buffer[seq_num].length : 0;
  out.end_of_pkt = (frame_type == FRAME_TYPE_FRAME) ? pkt_buffer[seq_num].end_of_pkt : 0;

  // Actually load the payload into the frame, if it's a frame
  if(frame_type == FRAME_TYPE_FRAME)
    {
      memcpy(out.payload, &(pkt_buffer[seq_num].payload), pkt_buffer[seq_num].length);
      
      // THEN compute the checksum.  Doh.  
      out.checksum = compute_checksum(&out);
    }
  else
    out.checksum = seq_num; // ACK checksum is just the sequence number again

  dprintf(DID_DLL_INFO, "%d:  DLL:  Sending %s with seq %d of length %ld bytes with payload of %d bytes\n", 
	 id, get_frame_type(out), out.seq, sizeof(struct frame), out.length);

  // Start the timer by recording when we sent this frame (if it's an actual frame)
  if(frame_type == FRAME_TYPE_FRAME)
  gettimeofday(&(pkt_buffer[seq_num].time_sent), NULL);

  // Send it to the physical layer
  if(write(fd, &out, sizeof(struct frame)) <= 0)
    {
      dprintf(DID_DLL_INFO, "%d:  DLL:  send_frame terminating!\n", id);
      pthread_exit(NULL);
    }
}

uint16_t compute_checksum(struct frame *frame)
{
  int bytes_remaining = frame->length;

  // Our header has an odd length, so load in one byte to start
  uint16_t sum = frame->type; 

  char *f_ptr = (char *)&((*frame).seq); // Start summing at the second byte  
  
  while(f_ptr != (char *)&((*frame).checksum)) // Add the header bytes (sans the checksum) to the sum
    {
      sum += *f_ptr ^ *(f_ptr + 1);
      f_ptr += 2;
    }

  // Then add the payload, if any
  f_ptr = frame->payload;

  while(bytes_remaining > 1)
    {
      sum += *f_ptr ^ *(f_ptr + 1);

      f_ptr +=2;
      bytes_remaining -= 2;
    }

  if(bytes_remaining > 0) // Add the leftover byte, if any
    sum += *f_ptr;

  return sum;
}

void print_layer_stack_statistics(struct layer_stack *stack)
{
  printf("\n\n---- Statistics for Layer Stack Transmission for %d----\n", stack->id);
  printf("Total Frames Sent:  %d\n", stack->total_frames_sent);
  printf("Total ACKs Sent:  %d\n", stack->total_acks_sent);
  printf("Total Fames Sent Successfully:  %d\n", stack->total_good_frames_sent);
  printf("Total Frames Received Successfully:  %d\n", stack->total_good_frames_recvd);
  printf("Total ACKs Sent Successfully:  %d\n", stack->total_good_acks_sent);
  printf("Total ACKs Received with Errors:  %d\n", stack->total_bad_acks_recvd);
  printf("Total Duplicate Frames Received:  %d\n", stack->total_dup_frames_recvd);
  printf("Total Frames Received with Errors:  %d\n", stack->total_bad_frames_recvd);
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
