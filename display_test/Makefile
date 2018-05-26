CXXFLAGS=-std=c++11 -g -I.. `pkg-config --cflags opencv`
LDFLAGS=-pthread `pkg-config --libs opencv`

.PHONY: all clean

all: display_test unwrap_display_test

display_test: display_test.cc
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

unwrap_display_test: unwrap_display_test.cc
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

clean:
	rm -f {unwrap_,}display_test
