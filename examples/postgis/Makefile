CFLAGS=-I../../include -std=c++0x -fopenmp -O0 -ggdb3 -std=c++14
LDFLAGS=../../lib/libroutingkit.a -lm -lz -pthread -lzmqpp -lzmq -lpq -lpqxx

all:
	g++ -o nmb ${CFLAGS} nwb-router.cpp ${LDFLAGS}
	g++ -o top10 ${CFLAGS} top10-router.cpp ${LDFLAGS}

