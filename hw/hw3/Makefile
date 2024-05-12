CC=gcc
CFLAGS= -Wall -std=gnu99 -g
LIBS=src/matrix.c
TUNE= -O2

sequential:
		$(CC) $(TUNE) $(CFLAGS) -o bin/seq $(LIBS) src/sequential.c

mpi:
		mpicc $(TUNE) $(CFLAGS) -o bin/mpi $(LIBS) src/mpi.c
