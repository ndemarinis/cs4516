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
    int sock;
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

    char read_buffer[PIPE_BUFFER_SIZE];

    memset(read_buffer, 0, PIPE_BUFFER_SIZE);
  
    if((init_layer_stack(sock, pipes))) // Initialize all of our layer threads
        die_with_error("Layer stack creation failed!");

    //wait for the threads to initialize
    usleep(100);
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
        int send = 0;
        //handle the login command
        if (strcmp(token, "login") == 0){
            //process the login command: syntax "login <username>"
            char* username = strtok(NULL, " \n");
            p.opcode = 0x01;

            //copy data into the packet
            p.length = strlen(username);
            strcpy(p.payload, username);

            //set the send flag
            send = 1;
        } else if (strcmp(token, "create") == 0){

        } else if (strcmp(token, "query") == 0){
            //process query command: syntax "query name <firstName> <lastName>" or "query location <location>"
            //tokenize name or location
            char *tmp  = strtok(NULL, " \n");
            if (strcmp(tmp, "name") == 0){
                //if searching for name pull the first and last name from the query string
                char *firstName = strtok(NULL, " \n");
                char *lastName = strtok(NULL, " \n");

                //put data into the packet
                p.opcode = 0x03;
                strcpy(p.payload, "NAME:");
                strcat(p.payload, firstName);
                strcat(p.payload, ",");
                strcat(p.payload, lastName);
                p.length = strlen(p.payload);

                //set the send flag
                send = 1;
            } else if (strcmp(tmp, "location") == 0){
                //if location...
                char *location = strtok(NULL, " \n");

                //add data to packet
                p.opcode = 0x03;
                strcpy(p.payload, "LOCATION:");
                strcat(p.payload, location);
                p.length = strlen(p.payload);

                //set the send flag
                send = 1;
            }
        } else if (strcmp(token, "update") == 0) {
            //process update command: syntax "update <id> <firstName> <lastName>"
            char *id = strtok(NULL, " \n");
            char *firstName = strtok(NULL, " \n");
            char *lastName = strtok(NULL, " \n");

            p.opcode = 0x04;
            strcpy(p.payload, id);
            strcat(p.payload, ",");
            strcat(p.payload, firstName);
            strcat(p.payload, ",");
            strcat(p.payload, lastName);
            p.length = strlen(p.payload);

            send = 1;
        } else if (strcmp(token, "add") == 0){
            printf("\tSending picture to server...\n");
            char *firstName = strtok(NULL, " \n");
            char *lastName = strtok(NULL, " \n");
            char *pictureLoc = strtok(NULL, " \n");

            //open the picture to read the binary data
            FILE *picture = fopen(pictureLoc, "rb");
            if (!picture){
                printf("\tCould not open %s!\n", pictureLoc);
            } else {
                //Send a first packet containing the name associated with the picture and inform the server to prepare for a picture
                strcpy(p.payload, firstName);
                strcat(p.payload, ",");
                strcat(p.payload, lastName);
                p.length = strlen(p.payload);
                //set the opcode for this entire session of sending
                p.opcode = 0x05;
                //set the sequence number
                p.seq_num = cur_seq_num;
                cur_seq_num++;
                //send this first name packet and then start sending the picture data
                write(pipe_write(pipes), &p, p.length + PACKET_OVERHEAD);
                
                //read into packets and send them until the end of file is reached
                //note this uses the same packet pointer the entire time so the opcode does not need to be set again
                while(!feof(picture)){
                    //read at most 251 bytes of the picture into the packets payload
                    int readSize = fread(p.payload, 1, MAX_PAYLOAD, picture);
                    //if there was no error then add the sequence number and the length to the packet then send it
                    //DO NOT SET THE SEND FLAG, this will handle it on its own since there could be multiple sends
                    if (!ferror(picture)){
                        p.seq_num = cur_seq_num;
                        cur_seq_num++;

                        p.length = (uint8_t)readSize;

                        //send this packet down to the data link layer
                        write(pipe_write(pipes), &p, p.length + PACKET_OVERHEAD);
                    } else {
                        printf("\tError sending picture!");
                        break;
                    }
                }
                //close the picture that was being read
                fclose(picture);
                printf("\tPicture sent!\n");
            }
        } else if (strcmp(token, "connect") == 0){
            char *pictureID = strtok(NULL, " \n");
            char *bodyID = strtok(NULL, " \n");

            p.opcode = 0x06;
            strcpy(p.payload, pictureID);
            strcat(p.payload, ",");
            strcat(p.payload, bodyID);
            p.length = strlen(p.payload);

            //set the send flag so that it is sent out
            send = 1;
        } else if (strcmp(token, "logout") == 0){
            //no data is necessary in the payload for this packet
            //server will see the opcode and know to logout
            p.opcode = 0x07;
            p.length = 0;

            send = 1;
        } else if (strcmp(token, "download") == 0){
            char *pictureID = strtok(NULL, " \n");

            p.opcode = 0x08;
            strcpy(p.payload, pictureID);
            p.length = strlen(p.payload);

            send = 1;
        } else if (strcmp(token, "quit") == 0) {
            break;
        } else {
          //invalid command!
          fputs("Invalid Command!\n", stdout);  
        }

        //if the send flag has been set then this packet is ready to be sent
        if (send){
            //add the sequence number and then increment
            p.seq_num = cur_seq_num;
            cur_seq_num++;

            write(pipe_write(pipes), &p, p.length + PACKET_OVERHEAD);

            //REMOVE THIS FOR FINAL PRODUCT ONCE THE LAYER TEST OUTPUTS ARE REMOVED!
            usleep(100);
        }
    }

    // Cleanup
    printf("DID Client:  Done.\n"); 
    close(sock);
    exit(0);
}
