CC=gcc -Wall

all: did_server did_client

client: did_client

did_server: did_server.o layer_stack.o disasterID_sql.o
	$(CC) -pthread -g disasterID_sql.o layer_stack.o did_server.o -o did_server -L/usr/local/mysql-current/lib/mysql -lmysqlclient -ldl

did_client: did_client.o layer_stack.o 
	$(CC) -pthread -g layer_stack.o did_client.o -o did_client

did_server.o: did_server.c layer_stack.h disasterID_sql.h
	$(CC) -pthread -c -g did_server.c

layer_stack.o: layer_stack.c layer_stack.h 
	$(CC) -pthread -c -g layer_stack.c

did_client.o: did_client.c layer_stack.h disasterID_sql.h
	$(CC) -c -g did_client.c

disasterID_sql.o: disasterID_sql.c disasterID_sql.h
	$(CC) -I/usr/local/mysql-current/include -c disasterID_sql.c 

clean:
	rm -fv did_server.o layer_stack.o did_server did_client.o did_client

