CFLAGS=-I../../include -std=c++11 -fopenmp -O0 -ggdb3
LDFLAGS=../../lib/libroutingkit.a -lm -lz -pthread -lzmqpp -lzmq

all:
	g++ -o quay-router ${CFLAGS} quay-router.cpp ${LDFLAGS}
	g++ -o client -std=c++11 client.cpp -lzmqpp -lzmq

clean:
	rm quay-router
