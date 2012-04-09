/*
* did_server.c
* Nicholas DeMarinis
* Eric Prouty
* Ian Lonergan
* 29 March 2012
*/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <sys/socket.h>

#include "layer_stack.h"
#include "disasterID_sql.h"

#define DID_DEFAULT_PORT 4516
#define MAX_PENDING 2
#define MAX_CLIENTS 5

#define __PIC_TOO_LARGE_CODE 0x01
#define __NO_PIC_ID_CODE 0x02
#define __NO_BODY_ID_CODE 0x03
#define __NOT_AUTHORIZED_CODE 0x04
#define __OK_CODE 0x05
#define __OK_MESSAGE_CODE 0x06

#define __LOGIN_CODE 1
#define __CREATE_CODE 2
#define __QUERY_CODE 3
#define __UPDATE_CODE 4
#define __ADD_PIC_CODE 5
#define __QUERY_PIC_CODE 8
#define __CONNECT_PIC_CODE 6
#define __LOGOUT_CODE 7

// Struct for data we send to the client handler
struct client_handler_data
{
    int sock; // Just the client's fd, for now
};

// Prototypes
void *handle_client(void *data);
void send_packet(int pipes[], struct packet p);
void return_error(int pipes[], int error_code, uint16_t *cur_seq_num);
void return_response(int pipes[], char *payload, uint16_t *cur_seq_num);

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
        if (opcode == __LOGIN_CODE){
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
                response_p.opcode = __NOT_AUTHORIZED_CODE;
                response_p.seq_num = cur_seq_num;
                cur_seq_num++;
                response_p.length = 0;

                send_packet(pipes, response_p);
            }
        //create record
        } else if (opcode == __CREATE_CODE){
            //payload syntax "<firstName>,<lastName>,<location>"
            char *firstName = strtok(client_p.payload, ",");
            char *lastName = strtok(NULL, ",");
            char *location = strtok(NULL, "");

            //replace null with proper response once I determine what its for!
            char *response = malloc(10*sizeof(response));
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
        } else if (opcode == __QUERY_CODE){
            //payload syntax "NAME:<firstName>,<lastName>"
            //               "LOCATION:<location>"
            char *queryType = strtok(client_p.payload, ":");

            bodyEntry *responses;
            int resp;
            //two types of query name and location
            if (strcmp(queryType, "NAME") == 0){
                //handle queries by name
                char *firstName = strtok(NULL, ",");
                char *lastName = strtok(NULL, "");
                
                //get the query response from the database
                resp = queryRecordByName(firstName, lastName, &responses);

                //if resp is not 0 then an error occured
                if (resp){
                    //send an error back!
                    return_error(pipes, resp, &cur_seq_num);
                } else {
                    //send the information back to the client!
                    int total_responses = sizeof(responses) / sizeof(responses[0]);
                    char *response = "";
                    int i;
                    for (i = 0; i < total_responses; i++){
                        int rID = responses[i].id;
                        char *recordID;
                        sprintf(recordID, "%d", rID);
                        strcat(response, recordID);
                        strcat(response, ",");
                        strcat(response, responses[i].location);
                        strcat(response, ",");
                    }
                    return_response(pipes, response, &cur_seq_num);
                }
            } else if (strcmp(queryType, "LOCATION") == 0){
                //handle queries by location
                char *location = strtok(NULL, "");

                //get the query response from the database
                resp = queryRecordByLocation(location, &responses);

                //if resp is not 0 then an error occured
                if (resp){
                    //send an error back!
                    return_error(pipes, resp, &cur_seq_num);
                } else {
                    //send the information back to the client!
                    int total_responses = sizeof(responses) / sizeof(responses[0]);
                    char *response = "";
                    int i;
                    for (i = 0; i < total_responses; i++){
                        int rID = responses[i].id;
                        char *recordID;
                        sprintf(recordID, "%d", rID);
                        strcat(response, recordID);
                        strcat(response, ",");
                        strcat(response, responses[i].firstName);
                        strcat(response, ",");
                        strcat(response, responses[i].lastName);
                        strcat(response, ",");
                    }
                    return_response(pipes, response, &cur_seq_num);
                }
            }
        //update record
        } else if (opcode == __UPDATE_CODE){
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
        } else if (opcode == __ADD_PIC_CODE){
            /*payload syntax 1st   packet:   "<firstName>,<lastName>,<imageSize>"
                             2nd+ packets:   "<pictureData>"
                             These picture data packets will continue until the entirety of the picture has been transmitted
                             The file is complete once the server has recieved a packet ending with an EOF character
            */
            //get the name information from the first packet transmitted
            char *firstName = strtok(client_p.payload, ",");
            char *lastName = strtok(NULL, ",");
            char *imageSize = strtok(NULL, "");

            unsigned long size = atol(imageSize);

            //create an array capable of holding the entire image
            char pictureData[size];
            int i = 0;
            //start recieving the picture data, continue until this entire picture has been recieved
            while(i < size - 1){
                //read in a new packet of data
                bytes_read = read(pipe_read(pipes), &client_p, MAX_PACKET);
                //store the picture data into the array
                memcpy(pictureData + i, client_p.payload, bytes_read);
                //increment i by the length of the payload
                i += bytes_read;
            }

            char *response;
            int resp = addPicture(firstName, lastName, pictureData, response);

            if (resp){
                //send an error back!
                return_error(pipes, resp, &cur_seq_num);
            } else {
                //send the information back to the client!
                return_response(pipes, response, &cur_seq_num);
            }
        //connect picture
        } else if (opcode == __CONNECT_PIC_CODE){
            //payload syntax "<pictureID>,<recordID>"
            char *pictureID = strtok(client_p.payload, ",");
            char *recordID = strtok(NULL, "");

            int pID = atoi(pictureID);
            int rID = atoi(recordID);

            int resp = connectPictureToRecord(pID, rID);

            if (resp){
                //send and error back!
                return_error(pipes, resp, &cur_seq_num);
            } else {
                //send the success packet back!
                return_response(pipes, "", &cur_seq_num);
            }
        //logout!
        } else if (opcode == __LOGOUT_CODE){
            //payload syntax NONE
            //break out of the while loop containin this and allow the thread processing this client to exit
	    logout();
            break;
        //download picture
        } else if (opcode == __QUERY_PIC_CODE){
            //payload syntax "<pictureID>"
            char *pictureID = strtok(client_p.payload, "");
            int pID = atoi(pictureID);

            char *pictureData;
            int resp = queryPicture(pID, pictureData);

            if (resp){
                //send an error back!
                return_error(pipes, resp, &cur_seq_num);
            } else {
                //break the image into packets and send it back to the client!
                //write the data into a tempory file handle for simple reading when breaking into packets
                //base the temporary file off of the pid to ensure uniqueness
                char *filename;
                int pid = getpid();
                char cpid[10];
                sprintf(cpid, "%d", pid);
                strcpy(filename, "temp_");
                strcat(filename, cpid);
                strcat(filename, ".jpg");
                //open the temp file for writing
                FILE *picture = fopen(filename, "w");
                //write the entire image to the file!
                fwrite(pictureData, 1, sizeof(pictureData), picture);

                //send a simple packet informing the client of the size of the image that it is going to recieve
                response_p.opcode = 5;
                response_p.seq_num = cur_seq_num;
                cur_seq_num++;
                sprintf(response_p.payload, "%lu", sizeof(pictureData));
                response_p.length = strlen(response_p.payload);
                send_packet(pipes, response_p);

                //read into packets and send them until the end of file is reached
                //note this uses the same packet pointer the entire time so the opcode does not need to be set again
                response_p.opcode = 5;
                while(!feof(picture)){
                    //read at most 251 bytes of the picture into the packets payload
                    int readSize = fread(response_p.payload, 1, MAX_PAYLOAD, picture);
                    //if there was no error then add the sequence number and the length to the packet then send it
                    //DO NOT SET THE SEND FLAG, this will handle it on its own since there could be multiple sends
                    if (!ferror(picture)){
                        response_p.seq_num = cur_seq_num;
                        cur_seq_num++;

                        response_p.length = (uint8_t)readSize;

                        //send this packet down to the data link layer
                        send_packet(pipes, response_p);
                    } else {
                        //an error occured return an error code so that the client can stop processing the image and drop the corrupt data
                        return_error(pipes, 1, &cur_seq_num);
                        break;
                    }
                }
                //close the picture that was being read
                fclose(picture);
                //delete the temporary file
                remove(filename);
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

void return_error(int pipes[], int error_code, uint16_t *cur_seq_num){
    struct packet p;
    //send an error back!
    p.opcode = error_code;
    p.seq_num = *cur_seq_num;
    (*cur_seq_num)++;
    p.length = 0;
    send_packet(pipes, p);
}

void return_response(int pipes[], char *payload, uint16_t *cur_seq_num){
    struct packet p;
    //respond to the clients request
    //set response code for success!
    p.opcode = 0x05;
    p.seq_num = *cur_seq_num;
    (*cur_seq_num)++;
    //add the payload into the packet
    strcpy(p.payload, payload);
    p.length = strlen(p.payload);

    //send the response
    send_packet(pipes, p);
}
