/*did_client.c
  Eric Prouty
  April 3, 2012
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
#define RECV_BUF_SIZE       32
#define DID_DEFAULT_PORT    4516

#define TERMINATOR_STR { 0x10, 0x03 } // Our termination sequence, a string of two bytes

int main(int argc, char *argv[]){
    int n;
    int sock, bytes_recvd;
    char *srv_ip;
    char input[32];
  
    uint16_t cur_seq_num;

    struct hostent *srv_host;
    struct sockaddr_in srv_addr;

    srv_ip = argv[1];

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
    srv_addr.sin_port = htons(DID_DEFAULT_PORT);

    // Connect to the DID server!
    if((connect(sock, (struct sockaddr*)(&srv_addr), sizeof(srv_addr))) < 0)
        die_with_error("connect() failed!");

    int pipes[2]; // Make a pipe to connect to the layer stack

    int to_read;
    char read_buffer[PIPE_BUFFER_SIZE];

    memset(read_buffer, 0, PIPE_BUFFER_SIZE);
  
    if((init_layer_stack(sock, pipes))) // Initialize all of our layer threads
        die_with_error("Layer stack creation failed!");

    sleep(1);
    //main client loop
    while(1){
        //command line interface for the client
        fputs("DID Client: ", stdout);
        fflush(stdout);
        //get user input of command
        fgets(input, sizeof input, stdin);

        //tokenize string and if it is a proper command execute it
        char *token = strtok(input, " \n");
        
        //time to handle the command... create a packet that may be used
        struct packet p;
        //handle the login command
        if (strcmp(token, "login") == 0){
            //process the login command: syntax "login <username> <password>"
            char* username = strtok(NULL, " \n");
            char* password = strtok(NULL, " \n");

            p.seq_num = cur_seq_num;
            p.seq_total = 1;
            p.opcode = 0x01;

            char data[256];
            strcpy(data, username);
            strcat(data, ",");
            strcat(data, password);

            p.length = strlen(data);
            strcpy(p.payload, data);
        } else {
          //invalid command!
          fputs("Invalid Command!\n", stdout);  
        }

        //if the packet has a correct sequence number at this point then it needs to be passed to the data link layer
        if (p.seq_num == cur_seq_num){
            write(pipe_write(pipes), &p, p.length + PACKET_OVERHEAD);
        }
    }

    // Cleanup
    printf("DID Client:  Done.\n"); 
    close(sock);
    exit(0);
}
