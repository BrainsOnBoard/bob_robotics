
CC      = g++
CFLAGS  = -std=c++14 -pthread `pkg-config --libs --cflags opencv`
LDFLAGS = 
SOURCE  = 


all: main clean

main: main.o
	$(CC) $(SOURCE) -o $@ $^ $(CFLAGS) $(LDFLAGS)

main.o: main.cpp 
	$(CC) -c $(CFLAGS) $<


.PHONY: clean cleanest

clean:
        rm -f *.o

cleanest: clean
        rm -f main
