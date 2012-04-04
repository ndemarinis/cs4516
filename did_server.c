/*
* did_server.c
* Nicholas DeMarinis
* Eric Prouty
* 29 March 2012
*/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <sys/socket.h>

#include "layer_stack.h"

#define DID_DEFAULT_PORT 4516
#define MAX_PENDING 2
#define MAX_CLIENTS 5

// Struct for data we send to the client handler
struct client_handler_data
{
    int sock; // Just the client's fd, for now
};

// Prototypes
void *handle_client(void *data);
void send_packet(int pipes[], struct packet p);
void return_error(int pipes[], int error_code, int *cur_seq_num);
void return_response(int pipes[], char *payload, int *cur_seq_num);

// Globals
struct client_handler_data client_data[MAX_CLIENTS];

int main(int argc, char *argv[])
{
    int srv_sock, clnt_sock, curr_clients = 0;
    unsigned int clnt_len;
    struct sockaddr_in srv_addr, clnt_addr;

    pthread_t threads[MAX_CLIENTS];

    // Create our listen socket
    if((srv_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        die_with_error("socket() failed!");

    // Create the address structure
    memset(&srv_addr, 0, sizeof(srv_addr));
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    srv_addr.sin_port = htons(DID_DEFAULT_PORT);

    // Bind to the address we just specified
    if((bind(srv_sock, (struct sockaddr *)(&srv_addr), sizeof(srv_addr)) < 0))
        die_with_error("bind() failed!\n");

    // Listen for connections
    if((listen(srv_sock, MAX_PENDING) < 0))
        die_with_error("listen() failed!");

    for(;;)
    {
        // When we receive a connection from a a client, make a new thread to handle them
        clnt_len = sizeof(clnt_addr);

        if((clnt_sock = accept(srv_sock, (struct sockaddr *)(&clnt_addr), &clnt_len)) < 0)
        die_with_error("accept() failed!");

        client_data[curr_clients].sock = clnt_sock;

        if(curr_clients < MAX_CLIENTS)
            pthread_create(&(threads[curr_clients]), NULL, handle_client, 
                            (void *)(&(client_data[curr_clients])));
        else
            die_with_error("Too many clients!");
    }

    // We should never reach this.  Yet.
    pthread_exit(NULL);
    exit(0);
}


/**
* handle_client:  Main application layer thread for each client
* @author ndemarinis (Basic implementation)
*/
void *handle_client(void *data){
    struct client_handler_data *clnt = (struct client_handler_data *)data;
    int pipes[2]; // Make a pipe to connect to the layer stack
    uint16_t cur_seq_num = 0;

    if((init_layer_stack(clnt->sock, pipes))) // Initialize all of our layer threads
        die_with_error("Layer stack creation failed!");

    int bytes_read;
    struct packet client_p;
    struct packet response_p;
    while(1){
        //Wait for a packet to come in
        bytes_read = read(pipe_read(pipes), &client_p, MAX_PACKET);

        int opcode = client_p.opcode;
        //login
        if (opcode == 1){
            //payload is just the username for this opcode
            if(login(client_p.payload) == 0){
                //response code for SUCCESS
                response_p.opcode = 0x05;
                response_p.seq_num = cur_seq_num;
                cur_seq_num++;
                //no payload basic success response!
                response_p.length = 0;

                send_packet(pipes, response_p);
            } else {
                //basic response packet signaling invailed login
                response_p.opcode = 0x04;
                response_p.seq_num = cur_seq_num;
                cur_seq_num++;
                response_p.length = 0;

                send_packet(pipes, response_p);
            }
        //create record
        } else if (opcode == 2){
            //payload syntax "<firstName>,<lastName>,<location>"
            char *firstName = strtok(client_p.payload, ",");
            char *lastName = strtok(NULL, ",");
            char *location = strtok(NULL, "");

            //replace null with proper response once I determine what its for!
            char *response;
            int resp = createRecord(firstName, lastName, location, response);
            //if a 0 was not returned an error occured
            if (resp) {
                //send an error back!
                return_error(pipes, resp, &cur_seq_num);
            } else {
                //send back the data
                return_response(pipes, response, &cur_seq_num);
            }
        //query record
        } else if (opcode == 3){
            //payload syntax "NAME:<firstName>,<lastName>"
            //               "LOCATION:<location>"
            char *queryType = strtok(client_p.payload, ":");

            char *response;
            int resp;
            //two types of query name and location
            if (strcmp(queryType, "NAME") == 0){
                //handle queries by name
                char *firstName = strtok(NULL, ",");
                char *lastName = strtok(NULL, "");
                
                //get the query response from the database
                resp = queryRecordByName(firstName, lastName, response);
            } else if (strcmp(queryType, "LOCATION") == 0){
                //handle queries by location
                char *location = strtok(NULL, "");

                //get the query response from the database
                resp = queryRecordByLocation(location, response);
            }
            //if resp is not 0 then an error occured
            if (resp){
                //send an error back!
                return_error(pipes, resp, &cur_seq_num);
            } else {
                //send the information back to the client!
                return_response(pipes, response, &cur_seq_num);
            }
        //update record
        } else if (opcode == 4){
            //payload syntax "<recordId>,<firstName>,<lastName>"
            char *recordId = strtok(client_p.payload, ",");
            char *firstName = strtok(NULL, ",");
            char *lastName = strtok(NULL, "");

            //convert recordID into an integer
            int rId = atoi(recordId);
            int resp = updateRecordName(rId, firstName, lastName);

            //if resp is not 0 then an error occured
            if (resp){
                //send an error back!
                return_error(pipes, resp, &cur_seq_num);
            } else {
                //send the information back to the client!
                //basic success packet so an empty string is given for the payload
                return_response(pipes, "", &cur_seq_num);
            }
        //add picture
        } else if (opcode = 5){
            /*payload syntax 1st   packet:   "<firstName>,<lastName>"
                             2nd+ packets:   "<pictureData>"
                             These picture data packets will continue until the entirety of the picture has been transmitted
                             The file is complete once the server has recieved a packet ending with an EOF character
            */
            //get the name information from the first packet transmitted
            char *firstName = strtok(client_p.payload, ",");
            char *lastName = strtok(NULL, "");

            
            FILE *newPicture;
            //start recieving the picture data
            while(1){
                //read in a new packet of data
                bytes_read = read(pipe_read(pipes), &client_p, MAX_PACKET);
                pictureData
            }
        }
    // Send it straight back
    //printf("APP:  Sending string of %d bytes:  %s\n", to_read, read_buffer);
    //write(pipe_write(pipes), read_buffer, to_read);
    }

    pthread_exit(NULL);
}

void send_packet(int pipes[], struct packet p){
    write(pipe_write(pipes), &p, p.length + PACKET_OVERHEAD);
}

void return_error(int pipes[], int error_code, int *cur_seq_num){
    struct packet p;
    //send an error back!
    p.opcode = error_code;
    p.seq_num = *cur_seq_num;
    *cur_seq_num++;
    p.length = 0;
    send_packet(pipes, p);
}

void return_response(int pipes[], char *payload, int *cur_seq_num){
    struct packet p;
    //respond to the clients request
    //set response code for success!
    p.opcode = 0x05;
    p.seq_num = *cur_seq_num;
    *cur_seq_num++;
    //add the payload into the packet
    strcpy(p.payload, payload);
    p.length = strlen(p.payload);

    //send the response
    send_packet(pipes, p);
}
