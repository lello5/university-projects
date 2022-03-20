PREFIX=/usr/local/
CC=g++
INCLUDE_DIRS=-I$(PREFIX)include/ompl-1.5 -I/usr/include/eigen3

LIBS=-L$(PREFIX)lib -lompl -lboost_program_options

BINS = CarTrailerPlanning\

.phony: clean all

all:	$(BINS)

CarTrailerPlanning:	CarTrailerPlanning.cpp
	$(CC) $(INCLUDE_DIRS) -o $@ $^ $(LIBS)
	
clean:
	rm -rf $(BINS)
