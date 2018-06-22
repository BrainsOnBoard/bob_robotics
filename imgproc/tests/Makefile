CXXFLAGS= -std=c++11 -g -Wall -Wpedantic -Wextra -MMD -MP -I../.. `pkg-config --cflags --libs opencv`

.PHONY: all clean

all: deserialise_test serialise_test

deserialise_test: deserialise/test.cc serialise_test.d
	$(CXX) -o $@ $< $(CXXFLAGS)

-include deserialise/test.d

serialise_test: serialise/test.cc serialise_test.d
	$(CXX) -o $@ $< $(CXXFLAGS)

-include serialise/test.d

%.d: ;

clean:
	rm -f deserialise_test serialise_test serialise_test.yaml *.d
