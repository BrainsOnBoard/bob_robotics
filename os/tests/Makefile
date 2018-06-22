CXXFLAGS=-std=c++11 -g -Wall -Wpedantic -Wextra -MMD -MP -I../..

.PHONY: all clean

all: list_cameras

list_cameras: list_cameras.cc list_cameras.d
	$(CXX) -o $@ $(CXXFLAGS) $<

-include list_cameras.d

%.d: ;

clean:
	rm -f list_cameras list_cameras.d