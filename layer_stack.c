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

int fds[2];
char* str = "Hello world!\n";

// Prototypes
void *init_physical_layer(void *in);

int main(int argc, char *argv[])
{
  int rv;

  long thread_ret;
  pthread_t thread;
 
  
  // Make our pipe
  if(pipe(fds))
    printf("Error creating pipe:  %s\n", strerror(errno));

  printf("Creating a thread.\n");
  if((rv = pthread_create(&thread, NULL, init_physical_layer, NULL)))
    printf("Error creating thread!\n");

  // Send something down our pipe
  write(fds[1], str, strlen(str) + 1);
  
  pthread_exit(NULL);
  exit(0);
}

void *init_physical_layer(void *in)
{
  int n;
  char read_buffer[128];

  printf("Thread created!\n");
  n = read(fds[0], read_buffer, 128);
  printf("Received string of length %d bytes:  %s", n, read_buffer);

  pthread_exit(NULL);
}
