CC=gcc -Wall -g

all: did_server did_client

debug: CC = gcc -Wall -g -DDID_DEBUG_MODE
debug: all

client: did_client

tests: echo_client_test picture_client_test test_did_server

did_server: did_server.o layer_stack.o disasterID_sql.o
	$(CC) -pthread disasterID_sql.o layer_stack.o did_server.o -o did_server -L/usr/local/mysql-current/lib/mysql -lmysqlclient -ldl

did_client: did_client.o layer_stack.o 
	$(CC) -pthread layer_stack.o did_client.o -o did_client

test_did_server: test_did_server.o layer_stack.o
	$(CC) -pthread test_did_server.o layer_stack.o -o test_did_server

test_did_server.o: test_did_server.c layer_stack.h
	$(CC) -pthread -c test_did_server.c

echo_client_test: echo_client_test.o layer_stack.o
	$(CC) -pthread layer_stack.o echo_client_test.o -o echo_client_test

echo_client_test.o: echo_client_test.c layer_stack.h
	$(CC) -pthread -c echo_client_test.c

picture_client_test: picture_client_test.o layer_stack.o
	$(CC) -pthread layer_stack.o picture_client_test.o -o picture_client_test

picture_client_test.o: picture_client_test.c layer_stack.h
	$(CC) -pthread -c picture_client_test.c

did_server.o: did_server.c layer_stack.h disasterID_sql.h
	$(CC) -pthread -c did_server.c

layer_stack.o: layer_stack.c layer_stack.h 
	$(CC) -pthread -c layer_stack.c

did_client.o: did_client.c layer_stack.h disasterID_sql.h
	$(CC) -c did_client.c

disasterID_sql.o: disasterID_sql.c disasterID_sql.h
	$(CC) -I/usr/local/mysql-current/include -g -c disasterID_sql.c 

clean:
	rm -fv did_server.o layer_stack.o disasterID_sql.o did_server did_client.o did_client echo_client_test.o picture_client_test.o echo_client_test picture_client_test
