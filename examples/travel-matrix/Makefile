CFLAGS=-I../../include -std=c++11 -fopenmp -march=native -O2 -ggdb3
LDFLAGS=../../lib/libroutingkit.a -lm -lz -pthread -lzmqpp -lzmq

all:
	g++ -o travel-matrix ${CFLAGS} travel-matrix.cpp ${LDFLAGS}
	g++ -o client -std=c++11 client.cpp -lzmqpp -lzmq

clean:
	rm travel-matrix client
