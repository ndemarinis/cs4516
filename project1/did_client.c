/*did_client.c
  Eric Prouty
  Ian Lonergan
  April 3, 2012
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <netdb.h>
#include <arpa/inet.h>

#include <sys/time.h>
#include <sys/socket.h>

#include "layer_stack.h"
#include "disasterID_sql.h"

#define MAX_MSG_SIZE        12
#define RECV_BUF_SIZE       32
#define DID_DEFAULT_PORT    4516

#define MAX_FILENAME_SIZE 64 // Maximum size of filenames for images


//prototypes
void send_packet(int pipes[], struct packet p, uint16_t *cur_seq_num);
void receive_packet(int pipes[], struct packet *p);

int main(int argc, char *argv[]){
    int sock;
    int loggedIn = 0;
    char *srv_ip;
    char input[32];
  
    uint16_t cur_seq_num;

    struct hostent *srv_host;
    struct sockaddr_in srv_addr;

    int pipes[2]; // Make a pipe to connect to the layer stack
    struct timeval cmd_start, cmd_end, cmd_diff; // Record the start and end time for any command
    char read_buffer[PIPE_BUFFER_SIZE];

    pid_t pid = getpid(); // Store our PID to send to the server

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


    memset(read_buffer, 0, PIPE_BUFFER_SIZE);

    // Send our PID as an identifier to the server.  
    if((send(sock, &pid, sizeof(pid_t), 0) != sizeof(pid_t)))
      die_with_error("Error sending PID to server!");
  
    printf("Client started with PID %d\n", pid);

    create_layer_stack(sock, pid, pipes); // Initialize all of our layer threads
    sleep(1); // Wait a second for the thread creation to settle.  

    //wait for the threads to initialize
    usleep(100);
    //main client loop
    while(1){
        //command line interface for the client
        fputs("DID Client: ", stdout);

        fflush(stdout);
        //get user input of command
        fgets(input, sizeof input, stdin);

	if(input[0] == '\n') // Just restart if the user entered nothing (we read a newline)
	  continue;

	gettimeofday(&cmd_start, NULL); // Record the time we started processing the command

        //tokenize string and if it is a proper command execute it
        char *token = strtok(input, " \n");
		
        //time to handle the command... create a packet that may be used
        struct packet p;
	memset(&p, 0, sizeof(struct packet));

        //handle the login command
        if (!loggedIn){
            if (strcmp(token, "login") == 0){
                //process the login command: syntax "login <username>"
		char *username = strtok(NULL,"\n");

                p.opcode = 0x01;

                //copy data into the packet
                p.length = strlen(username);
                strcpy(p.payload, username);

                //send the packet
                send_packet(pipes, p, &cur_seq_num);

                //wait for the servers response
                receive_packet(pipes, &p);

                //if succesful login
                if (p.opcode == 5){
                    loggedIn = 1;
                    printf("DID Client: Succesfully logged in!\n");
                } else {
                    //failed login so move onto the next iteration of the client
                    printf("DID Client: Invalid login... try again.\n");
                    continue;
                }
            } else {
		printf("Only 'login' request may be done now.");
	    }
        }
	else 
	{
		//if the user enters the quit command it breaks out of the main while loop and allows the client to exit
		if (strcmp(token, "quit") == 0) {
		    break;
		}
		if (strcmp(token, "create") == 0){
		    char *firstName = strtok(NULL, " \n");
		    char *lastName = strtok(NULL, " \n");
		    char *location = strtok(NULL, " \n");

		    //set create opcode
		    p.opcode = 0x02;

		    //copy data into packet
		    strcpy(p.payload, firstName);
		    strcat(p.payload, ",");
		    strcat(p.payload, lastName);
		    strcat(p.payload, ",");
		    strcat(p.payload, location);
		    p.length = strlen(p.payload);
		    
		    //send the packet
		    send_packet(pipes, p, &cur_seq_num);

		    //wait for the servers response
		    receive_packet(pipes, &p);

		    //if succesful print out the recordID that was just created
		    if (p.opcode == 5){
			printf("\tRecord created with ID: %s\n", p.payload);
		    } else {
			printf("\tRecord creation failed, error code: %d\n", p.opcode);
		    }
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

			//send the packet
			send_packet(pipes, p, &cur_seq_num);

			//wait for the response
			receive_packet(pipes, &p);

			//if succesful print out the records
			if (p.opcode == 5){
			    printf("\tResults for name: %s %s\n", firstName, lastName);
			    //parse the string of data back into a response structure
			    char *tmp = strtok(p.payload, ",");
			    while(tmp){
				char recordID[10];
				strcpy(recordID, tmp);
				tmp = strtok(NULL, ",");
				char location[37];
				strcpy(location, tmp);
				tmp = strtok(NULL, ",");
				printf("\t\tRecordID: %s\tLocation: %s\n", recordID, location);
			    }
			} else {
			    printf("\tQuery failed, error code: %d\n", p.opcode);
			}
		    } else if (strcmp(tmp, "location") == 0){
			//if location...
			char *location = strtok(NULL, " \n");

			//add data to packet
			p.opcode = 0x03;
			strcpy(p.payload, "LOCATION:");
			strcat(p.payload, location);
			p.length = strlen(p.payload);

			//send the packet
			send_packet(pipes, p, &cur_seq_num);

			//wait for the response
			receive_packet(pipes, &p);

			//if succesful print out the records
			if (p.opcode == 5){
			    printf("\tResults for location: %s\n", location);
			    //parse the string of data back into a response structure
			    char *tmp = strtok(p.payload, ",");
			    while(tmp){
				char *recordID;
				strcpy(recordID, tmp);
				tmp = strtok(NULL, ",");
				char *firstName;
				strcpy(firstName, tmp);
				tmp = strtok(NULL, ",");
				char *lastName;
				strcpy(lastName, tmp);
				tmp = strtok(NULL, ",");
				printf("\t\tRecordID: %s\tName: %s, %s\n", recordID, lastName, firstName);
			    }
			} else {
			    printf("\tQuery failed, error code: %d\n", p.opcode);
			}
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

		    //send the packet
		    send_packet(pipes, p, &cur_seq_num);

		    //wait for response
		    receive_packet(pipes, &p);

		    if (p.opcode == 5){
			//success
			printf("\tRecord Updated!\n");
		    } else {
			printf("\tUpdate failed, error code: %d\n", p.opcode);
		    }
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
			//get the total size of the picture in bytes!
			//seek to the end of the file
			fseek(picture, 0L, SEEK_END);
			//get the location of the last byte... this is the total size in bytes
			unsigned long sizeInBytes = ftell(picture);
			//seek back to the beggining so the data can be read
			fseek(picture, 0L, SEEK_SET);
			//convert this size into a string so that it can be sent to the server
			char size[4];
			sprintf(size, "%lu", sizeInBytes);

			//Send a first packet containing the name associated with the picture and inform the server to prepare for a picture
			strcpy(p.payload, firstName);
			strcat(p.payload, ",");
			strcat(p.payload, lastName);
			strcat(p.payload, ",");
			strcat(p.payload, size);
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
			int read = 0;
			while(read < sizeInBytes){
			    //read at most 251 bytes of the picture into the packets payload
			    memset(&(p.payload),0,sizeof(p.payload));
			    int readSize = fread(p.payload, sizeof(char), MAX_PAYLOAD, picture);
			    //if there was no error then add the sequence number and the length to the packet then send it
			    //DO NOT SET THE SEND FLAG, this will handle it on its own since there could be multiple sends
			    if (!ferror(picture)){
				p.seq_num = cur_seq_num;
				cur_seq_num++;

				p.length = (uint8_t)readSize;

				//send this packet down to the data link layer
				read += readSize;
				write(pipe_write(pipes), &p, sizeof(struct packet));
				dprintf(DID_APP_INFO, "APP:  Sent packet with payload of %d bytes to layer stack.\n", 
					readSize);
			    } else {
				printf("\tError sending picture!");
				break;
			    }
			}

			//close the picture that was being read
			fclose(picture);
			printf("Successfully sent photo of %d total bytes\n", read);

			//wait for servers response
			receive_packet(pipes, &p);

			if (p.opcode == 5){
			    //success print out the pictureID
			    printf("\tPictured added with pictureID: %s\n", p.payload);
			} else {
			    printf("\tAdd picture failed, error code: %d\n", p.opcode);
			}
		    }
		} else if (strcmp(token, "connect") == 0){
		    char *pictureID = strtok(NULL, " \n");
		    char *bodyID = strtok(NULL, " \n");

		    p.opcode = 0x06;
		    strcpy(p.payload, pictureID);
		    strcat(p.payload, ",");
		    strcat(p.payload, bodyID);
		    p.length = strlen(p.payload);

		    //send the packet
		    send_packet(pipes, p, &cur_seq_num);

		    //wait for servers response
		    if (p.opcode == 5){
			printf("\tPicture succesfully connected to record!\n");
		    } else {
			printf("Connect picture failed, error code: %d\n", p.opcode);
		    }
		} else if (strcmp(token, "logout") == 0){
		    //no data is necessary in the payload for this packet
		    //server will see the opcode and know to logout
		    p.opcode = 0x07;
		    p.length = 0;

		    //send the packet
		    send_packet(pipes, p, &cur_seq_num);

		    //set loggedIn back to 0, this will cause the client to wait for a login request
		    loggedIn = 0;
		} else if (strcmp(token, "download") == 0){
		    char *pictureID = strtok(NULL, " \n");

		    p.opcode = 0x08;
		    strcpy(p.payload, pictureID);
		    p.length = strlen(p.payload);

		    //send the packet
		    send_packet(pipes, p, &cur_seq_num);

		    //wait for response from server
		    receive_packet(pipes, &p);

		    //if not an error then handle it
		    if (p.opcode == 5){
			//this first packet is information about the picture about to be received
			unsigned long sizeOfImage = atol(p.payload);

			//all packets received from now on contain image data!
			char filename[MAX_FILENAME_SIZE];
			
			//filename will be <pictureID.jpg>
			strcpy(filename, pictureID);
			strcat(filename, ".jpg");
			//open the file for writing
			FILE *picture = fopen(filename, "w");

			int bytes_processed = 0;
			while (bytes_processed < sizeOfImage){
			    //receive a packet containing image data
			    receive_packet(pipes, &p);
			    //write this data into the file
			    fwrite(p.payload, 1, p.length, picture);
			    //increment bytes_processed so we know how much of the image has been transferred
			    bytes_processed += p.length;
			}

			//image is complete, close FILE handle
			fclose(picture);
			//inform the user that the picture is finished downloading
			printf("\tPicture succesfully downloaded!\n");
		    } else {
			//an error has occured!
			printf("\tPicture download failed, error code: %d\n", p.opcode);
		    }
		} else {
		  //invalid command!
		  fputs("Invalid Command!\n", stdout);  
		}
	}
	// Print out the total processing time for the command
	gettimeofday(&cmd_end, NULL);
	timersub(&cmd_end, &cmd_start, &cmd_diff);
	printf("Total command processing time:  %ld.%06ld s\n\n", 
	       cmd_diff.tv_sec, cmd_diff.tv_usec);
	
    }

    // Cleanup
    printf("DID Client:  Done.\n"); 
    close(sock);
    exit(0);
}

void send_packet(int pipes[], struct packet p, uint16_t *cur_seq_num){
    //add the sequence number and then increment
    p.seq_num = *cur_seq_num;
    (*cur_seq_num)++;

    write(pipe_write(pipes), &p, sizeof(struct packet));
}

void receive_packet(int pipes[], struct packet *p){
    //Wait for a packet to come in
    read(pipe_read(pipes), p, sizeof(struct packet));
}
