WITH_I2C_ROBOT := 1
include ../../make_common/bob_robotics.mk

EXECUTABLE := program_name

.PHONY: all clean

all: speed_test robot

-include speed_test.d

speed_test: speed_test.cc speed_test.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include robot.d

robot: robot.cc robot.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f speed_test robot *.d
