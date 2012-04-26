/*
 * echo_client.c
 * Stripped as a test platform for Project 1
 * Nicholas DeMarinis
 * 12 March 2012
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <netdb.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <sys/socket.h>

#include "layer_stack.h"

#define MAX_MSG_SIZE        12
#define RECV_BUF_SIZE       1024
#define DEFAULT_ECHO_PORT 4516

#define TERMINATOR_STR { 0x10, 0x03 } // Our termination sequence, a string of two bytes
#define TERM_STR_LEN 2

#define FILE_BUFFER_SIZE (32*1024) // Size of file to read, half the pipe buffer size for safety

// Prototypes
void die_with_error(char *msg);
unsigned int fsize(char* filename);

int main(int argc, char *argv[])
{
  int sock, bytes_recvd, file_len, bytes_remaining_to_read, bytes_remaining_to_write, len;
  int bytes_read, bytes_to_read, to_read_now, to_recv_now, total_bytes_sent = 0;
  char *srv_ip, *file_name, *out_name;
  FILE *fp_in, *fp_out;
  
  struct hostent *srv_host;
  struct sockaddr_in srv_addr;

  int pipes[2];

  char file_buffer[FILE_BUFFER_SIZE];
  char *f_ptr;
  
  pid_t pid = getpid(); // PID to send to the server as an identifier

  struct packet out, in;
 
  if((argc < 3) || (argc > 7))
    {
      fprintf(stderr, 
	      "Usage:  %s <Server IP> <picture filename (limit 100kb)> <dest>\n", argv[0]);
      exit(1);
    }

  srv_ip = argv[1];
  file_name = argv[2];
  out_name = argv[3];

  setvbuf(stdout, NULL, _IONBF, 0);

  // Create a TCP socket for the connection
  if((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    die_with_error("socket() failed!");

  // Resolve the hostname of the server
  if((srv_host = gethostbyname(srv_ip)) == NULL)
    die_with_error("Could not resolve hostname!");

  // Populate our address structure
  memset(&srv_addr, 0, sizeof(srv_addr));
  srv_addr.sin_family = AF_INET;
  srv_addr.sin_addr = *((struct in_addr*)(srv_host->h_addr_list[0]));
  srv_addr.sin_port = htons(DEFAULT_ECHO_PORT);

  // Connect to the echo server!
  if((connect(sock, (struct sockaddr*)(&srv_addr), sizeof(srv_addr))) < 0)
    die_with_error("connect() failed!");
 
  // Send our PID as an identifier to the server.  
  if((send(sock, &pid, sizeof(pid_t), 0) != sizeof(pid_t)))
    die_with_error("Error sending PID to server!");
  
  printf("Client started with PID %d\n", pid);

  // Make the layer stack
  create_layer_stack(sock, pid, pipes);

  sleep(1);  // Wait for the thread creation to settle.  

  fp_in = fopen(file_name, "r+");
  file_len = fsize(file_name);
  fp_out = fopen(out_name, "w+");

  bytes_to_read = file_len;
  
  while(bytes_to_read > 0)
    {
      memset(file_buffer, 0, FILE_BUFFER_SIZE);

      // Read as much of the file as our buffer allows, or the length if it's timy
      to_read_now = (bytes_to_read > FILE_BUFFER_SIZE) ? FILE_BUFFER_SIZE : bytes_to_read;

      printf("Trying to read %d bytes with %d bytes left\n", 
	     to_read_now, bytes_to_read);
      if((bytes_read = read(fileno(fp_in), file_buffer, to_read_now)) != to_read_now)
	{
	  printf("%s\n", strerror(errno));
	  die_with_error("Error reading file!");
	}
      bytes_to_read -= bytes_read;

      bytes_remaining_to_read = to_read_now;
      bytes_remaining_to_write = to_read_now;
      
      f_ptr = file_buffer;  // Set the read pointer to the beginning of the buffer.  

      // Send out those bytes in packets
      while(bytes_remaining_to_read > 0)
	{
	  memset(&out, 0, sizeof(struct packet));
	  memset(&in, 0, sizeof(struct packet));
	  
	  len = (bytes_remaining_to_read > PACKET_PAYLOAD_SIZE) ? 
	    PACKET_PAYLOAD_SIZE : bytes_remaining_to_read;

	  total_bytes_sent += len;

	  printf("Sending packet with %d bytes.  Sent %d bytes of %d bytes\n", 
		 len - 1, total_bytes_sent, file_len);
	  
	  memcpy(out.payload, f_ptr, len);
	  
	  f_ptr += len;
	  bytes_remaining_to_read -= len;
	  
	  out.length = len - 1;
	  
	  if((write(pipe_write(pipes), &out, sizeof(struct packet)) != sizeof(struct packet)))
	    die_with_error("send() sent a different number of bytes than expected");
	}
      
      // Read back those bytes as packets now so we don't overwhelm the pipes.  
      to_recv_now = 0;
      while(to_recv_now < bytes_remaining_to_write)
	{
	  if((bytes_recvd = read(pipe_read(pipes), &in, sizeof(struct packet)) <= 0))
	    die_with_error("recv() failed or connection closed unexpectedly!");      
	  
	  to_recv_now += in.length + 1;

	  write(fileno(fp_out), in.payload, in.length + 1);
	}
      
    }

  fclose(fp_in);
  fflush(fp_out);
  fclose(fp_out);

  // Cleanup
  printf("Test Picture Client:  Done.\n"); 
  close(sock);
  exit(0);
}


unsigned int fsize(char* filename)
{
  struct stat buf;
  
  stat(filename, &buf);
  printf("%i\n", (int)buf.st_size);
  return buf.st_size;
}
