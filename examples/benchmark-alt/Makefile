CFLAGS=-I../../include -std=c++11 -O3 -ggdb3 -Wall
LDFLAGS=../../lib/libroutingkit.a -fopenmp -lm -lz -pthread

all: benchmark-alt

benchmark-alt: benchmark-alt.cpp
	g++ -o benchmark-alt $(CFLAGS) benchmark-alt.cpp $(LDFLAGS)

clean:
	-rm -f benchmark-alt
