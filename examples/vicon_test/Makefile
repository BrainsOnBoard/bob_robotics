include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

all: vicon_test vicon_velocity_test

vicon_test:
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) vicon_test.cc -o vicon_test

-include vicon_test.d

vicon_velocity_test:
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) vicon_velocity_test.cc -o vicon_velocity_test

-include vicon_velocity_test.d

%.d: ;

clean:
	rm -f vicon_test vicon_velocity_test *.d
