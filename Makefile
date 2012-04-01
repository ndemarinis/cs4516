CC=gcc -Wall

all: did_server

did_server: did_server.o layer_stack.o
	$(CC) -pthread layer_stack.o did_server.o -o did_server

did_server.o: did_server.c layer_stack.h
	$(CC) -pthread -c did_server.c

layer_stack.o: layer_stack.c layer_stack.h
	$(CC) -pthread -c layer_stack.c