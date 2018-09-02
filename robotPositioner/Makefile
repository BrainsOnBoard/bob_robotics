
CC      = g++
CFLAGS  = -std=c++14
LDFLAGS = -pthread -lopencv_core -lopencv_imgproc
SOURCE  = -I/home/ubuntu/local/include/opencv2/imgproc


all: main clean

main: main.o
	$(CC) $(SOURCE) $(CFLAGS) -o $@ $^ $(LDFLAGS)

main.o: main.cpp 
	$(CC) -c $(CFLAGS) $<


.PHONY: clean cleanest

clean:
	rm *.o

cleanest: clean
	rm main
