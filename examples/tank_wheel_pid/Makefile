WITH_MATPLOTLIBCPP := 1
NO_OPENCV := 1
include ../../make_common/bob_robotics.mk

.PHONY: all clean

all: tank_wheel_pid_simulated tank_wheel_pid_robot

-include tank_wheel_pid_simulated.d

tank_wheel_pid_simulated: tank_wheel_pid_simulated.cc tank_wheel_pid_simulated.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include tank_wheel_pid_simulated.d

tank_wheel_pid_robot: tank_wheel_pid_robot.cc tank_wheel_pid_robot.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f tank_wheel_pid_simulated tank_wheel_pid_robot *.d