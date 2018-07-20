CXXFLAGS+=-std=c++14 -g -Wall -Wpedantic -Wextra -MMD -MP `pkg-config --libs --cflags opencv` -pthread

.PHONY: all clean

all: vicon_test vicon_velocity_test

vicon_test: vicon_test.cc vicon_test.d
	$(CXX) $(CXXFLAGS) vicon_test.cc -o vicon_test

-include vicon_test.d

vicon_velocity_test: vicon_velocity_test.cc vicon_velocity_test.d
	$(CXX) $(CXXFLAGS) vicon_velocity_test.cc -o vicon_velocity_test

-include vicon_velocity_test.d

%.d: ;

clean:
	rm -f vicon_test vicon_velocity_test *.d
