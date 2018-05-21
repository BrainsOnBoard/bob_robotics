.PHONY: all clean

CXXFLAGS=-g -std=c++11 -I../.. `pkg-config --cflags --libs opencv`

all: deserialise_test serialise_test

deserialise_test: deserialise/test.cc
	$(CXX) -o $@ $< $(CXXFLAGS)

serialise_test: serialise/test.cc
	$(CXX) -o $@ $< $(CXXFLAGS)

clean:
	rm -f deserialise_test serialise_test{,.yaml}
