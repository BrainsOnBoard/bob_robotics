WITH_I2C_ROBOT := 1
include ../../make_common/bob_robotics.mk

.PHONY: all clean

all: speed_test angular_velocity robot

-include speed_test.d

speed_test: speed_test.cc speed_test.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include angular_velocity.d

angular_velocity: angular_velocity.cc angular_velocity.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include robot.d

robot: robot.cc robot.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f speed_test robot *.d
