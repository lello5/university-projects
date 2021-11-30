PREFIX=..

CXX=g++

CXX_OPTS=-std=c++14 -Wall -Ofast $(INCLUDE_DIRS)

INCLUDE_DIRS=-I$(PREFIX)/src/common -I$(PREFIX)/src/orazio_host/	-I$(PREFIX)/src/host_examples/


LOBJS = ${PREFIX}/host_build/packet_handler.o\
	${PREFIX}/host_build/deferred_packet_handler.o\
	${PREFIX}/host_build/orazio_client.o\
	${PREFIX}/host_build/orazio_print_packet.o\
	${PREFIX}/host_build/serial_linux.o\

BIN = orazio_wrapper_test

CXXOBJ = orazio_wrapper.o

all: ${BIN} ${CXXOBJ}

%.o: %.cpp
	${CXX} ${CXX_OPTS} -c $<

orazio_wrapper_test: orazio_wrapper_test.o ${LOBJS} ${CXXOBJ}
	${CXX} ${CXX_OPTS}	-o $@ $< ${LOBJS} ${CXXOBJ}

clean:
	rm -rf ${BIN} ${CXXOBJ} *.o
