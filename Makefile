CC=gcc

layer_stack: layer_stack.c
	$(CC) -pthread layer_stack.c -o layer_stack