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

#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "layer_stack.h"

#define MAX_MSG_SIZE        12
#define RECV_BUF_SIZE       1024
#define DEFAULT_ECHO_PORT 4516

#define TERMINATOR_STR { 0x10, 0x03 } // Our termination sequence, a string of two bytes
#define TERM_STR_LEN 2

// Prototypes
void die_with_error(char *msg);

int main(int argc, char *argv[])
{
  int n;
  int sock, bytes_recvd;
  unsigned int echo_str_len;
  char *srv_ip, *echo_str;
  
  struct hostent *srv_host;
  struct sockaddr_in srv_addr;

  int pipes[2];

  char recv_buffer[RECV_BUF_SIZE];
  struct packet_segment out, in;
  
  if((argc < 3) || (argc > 7))
    {
      fprintf(stderr, 
	      "Usage:  %s <Server IP> <Echo word>\n", argv[0]);
      exit(1);
    }

  srv_ip = argv[1];
  echo_str = argv[2];

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
 
  echo_str_len = strlen(argv[2]) + 1;

  // Make the layer stack
  if((init_layer_stack(sock, pipes)))
    die_with_error("Layer stack creation failed!");

  sleep(1);

  //send(sock, &echo_str_len, sizeof(int), 0);
  for(n = 0; n < 5; n++)
    {
      memset(&out, 0, sizeof(struct packet_segment));
      memcpy(out.payload, echo_str, echo_str_len);
      out.end_of_pkt = FRAME_IS_EOP;

      if((write(pipe_write(pipes), &out, sizeof(struct packet_segment)) != sizeof(struct packet_segment)))
	die_with_error("send() sent a different number of bytes than expected");
      
      memset(&in, 0, sizeof(struct packet_segment)); // Zero our buffer for safety. 
	     
      if((bytes_recvd = read(pipe_read(pipes), in.payload, sizeof(struct packet_segment)) <= 0))
	 die_with_error("recv() failed or connection closed unexpectedly!");

      recv_buffer[bytes_recvd] = '\0';
      
      printf("Received string of %d bytes:  %s\n", bytes_recvd, out.payload);
    }

  // Cleanup
  printf("Echo Client:  Done.\n"); 
  close(sock);
  exit(0);
}

#if 0
void die_with_error(char *msg)
{
  printf("%s\n", msg);
  exit(2);
}

#endif
