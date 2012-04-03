/*
 * echo_client.c
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

#define MAX_MSG_SIZE        12
#define RECV_BUF_SIZE       32
#define DEFAULT_ECHO_PORT 4516

#define TERMINATOR_STR { 0x10, 0x03 } // Our termination sequence, a string of two bytes
#define TERM_STR_LEN 2

// Prototypes
void die_with_error(char *msg);

int main(int argc, char *argv[])
{
  int i, n, curr_str_len, terminated = 0;
  int sock, bytes_recvd;
  unsigned int echo_str_len;
  char *srv_ip, *echo_str;
  char *curr_str, *buffer_end; // Temporary pointers for string isolation later
  
  struct hostent *srv_host;
  struct sockaddr_in srv_addr;

  char recv_buffer[RECV_BUF_SIZE];
  char term_string[TERM_STR_LEN] = TERMINATOR_STR;

  if((argc < 3) || (argc > 7))
    {
      fprintf(stderr, 
	      "Usage:  %s <Server IP> <Echo word> [ up to four more words ]\n", argv[0]);
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

  //send(sock, &echo_str_len, sizeof(int), 0);
  for(n = 0; n < 5; n++)
    {
      if((send(sock, echo_str, echo_str_len, 0) != echo_str_len))
	die_with_error("send() sent a different number of bytes than expected");
      
      memset(recv_buffer, 0, RECV_BUF_SIZE); // Zero our buffer for safety. 
      
      if((bytes_recvd = recv(sock, recv_buffer, RECV_BUF_SIZE - 1, 0)) <= 0)
	die_with_error("recv() failed or connection closed unexpectedly!");
      
      recv_buffer[bytes_recvd] = '\0';
      
      printf("Received string of %d bytes:  %s\n", bytes_recvd, recv_buffer);
    }
#if 0
  // Send the terminator
  //if((send(sock, term_string, TERM_STR_LEN, 0)) != TERM_STR_LEN)
  //  die_with_error("send() sent a different number of bytes than expected");
  
  for(;;) // Try to recveive the same strings back from the server
    {
      memset(recv_buffer, 0, RECV_BUF_SIZE); // Zero our buffer for safety. 

      if((bytes_recvd = recv(sock, recv_buffer, RECV_BUF_SIZE - 1, 0)) <= 0)
	die_with_error("recv() failed or connection closed unexpectedly!");

      // We could have just taken in more than one string, so we need to separate them.  
      // See the server source file for an explanation of why it's done this way.  
      recv_buffer[bytes_recvd] = '\0'; // Always terminate the end of the buffer for safety.  
      curr_str = recv_buffer;
      buffer_end = recv_buffer + bytes_recvd;

#ifdef DEBUG
      printf("Received %d bytes:  0x", bytes_recvd);
      for(i = 0; i < bytes_recvd; i++)
	printf("%02X", recv_buffer[i]);
      printf("\n");
#endif

      while(curr_str < buffer_end) 
	{
	  curr_str_len = strlen(curr_str) + 1;
	  
	  // Break out if we find the terminator, which, interestingly, is null-terminated
	  if(!strncmp(curr_str, term_string, TERM_STR_LEN))
	    {
	      terminated = 1; // Set a flag so we know to break out
	      break;
	    }

	  printf("Echo Client:  Received string of %d bytes from server:  %s\n", 
		 curr_str_len, curr_str);
	  
	  curr_str += curr_str_len; // Move our pointer to the next string.  
	}
      
      if(terminated) // Stop trying to receive if we found it.  
	break;
    }
#endif

  // Cleanup
  printf("Echo Client:  Done.\n"); 
  close(sock);
  exit(0);
}

void die_with_error(char *msg)
{
  printf("%s\n", msg);
  exit(2);
}
