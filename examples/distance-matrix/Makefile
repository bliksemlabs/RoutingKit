CFLAGS=-I../../include -std=c++11 -fopenmp -O0 -ggdb3
LDFLAGS=../../lib/libroutingkit.a -lm -lz -pthread -lzmqpp -lzmq

all:
	# g++ -o distance-matrix-webservice ${CFLAGS} distance-matrix-webservice.cpp ${LDFLAGS}
	# g++ -o exact-path-webservice ${CFLAGS} exact-path-webservice.cpp ${LDFLAGS}
	g++ -o calculate-max-speed ${CFLAGS} calculate-max-speed.cpp ${LDFLAGS}
	# g++ -o exact-path-webservice-debug ${CFLAGS} exact-path-webservice-debug.cpp ${LDFLAGS}
	# g++ -o path-webservice ${CFLAGS} path-webservice.cpp ${LDFLAGS}
	# g++ -o client -std=c++11 client.cpp -lzmqpp -lzmq

clean:
	rm distance-matrix-webservice
