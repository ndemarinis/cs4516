CC=gcc -Wall

all: did_server did_client

client: did_client

did_server: did_server.o layer_stack.o
	$(CC) -pthread -g layer_stack.o did_server.o -o did_server

did_client: did_client.o layer_stack.o
	$(CC) -pthread -g layer_stack.o did_client.o -o did_client

did_server.o: did_server.c layer_stack.h
	$(CC) -pthread -c -g did_server.c

layer_stack.o: layer_stack.c layer_stack.h
	$(CC) -pthread -c -g layer_stack.c

did_client.o: did_client.c layer_stack.h
	$(CC) -c -g did_client.c

clean:
	rm -fv did_server.o layer_stack.o did_server did_client.o did_client