CXXFLAGS+=-std=c++11 -g -Wall -Wpedantic -Wextra -MMD -MP `pkg-config --libs --cflags opencv` -pthread

.PHONY: all clean

all: vicon_test vicon_blocking_test

vicon_test: vicon_test.cc vicon_test.d
	$(CXX) $(CXXFLAGS) vicon_test.cc -o vicon_test
	
-include vicon_test.d

vicon_blocking_test: vicon_blocking_test.cc vicon_blocking_test.d
	$(CXX) $(CXXFLAGS) vicon_blocking_test.cc -o vicon_blocking_test

-include vicon_blocking_test.d

%.d: ;

clean:
	rm -f vicon_test vicon_blocking_test *.d
