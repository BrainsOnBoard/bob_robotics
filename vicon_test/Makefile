CXXFLAGS+=-std=c++11 `pkg-config --libs --cflags opencv` -pthread

.PHONY: all clean

all: vicon_test vicon_blocking_test

vicon_test: vicon_test.cc ../vicon/udp.h
	$(CXX) $(CXXFLAGS) vicon_test.cc -o vicon_test

vicon_blocking_test: vicon_blocking_test.cc ../vicon/udp.h
	$(CXX) $(CXXFLAGS) vicon_blocking_test.cc -o vicon_blocking_test

clean:
	rm -f vicon_test
