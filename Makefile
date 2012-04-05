CC=gcc -Wall -g

all: did_server did_client echo_client_test

did_server: did_server.o layer_stack.o
	$(CC) -pthread layer_stack.o did_server.o -o did_server

did_client: did_client.o layer_stack.o
	$(CC) -pthread layer_stack.o did_client.o -o did_client

echo_client_test: echo_client_test.o layer_stack.o
	$(CC) -pthread layer_stack.o echo_client_test.o -o echo_client_test

echo_client_test.o: echo_client_test.c layer_stack.h
	$(CC) -pthread -c echo_client_test.c

did_server.o: did_server.c layer_stack.h
	$(CC) -pthread -c did_server.c

layer_stack.o: layer_stack.c layer_stack.h
	$(CC) -pthread -c layer_stack.c

did_client.o: did_client.c layer_stack.h
	$(CC) -c did_client.c

clean:
	rm -fv did_server.o layer_stack.o did_server did_client.o did_client