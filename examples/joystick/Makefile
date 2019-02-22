NO_OPENCV := 1
include ../../make_common/bob_robotics.mk

.PHONY: all clean

all: joystick_test joystick_robot_test joystick_polling_test

-include joystick_test.d

joystick_test: joystick_test.cc joystick_test.d
	$(CXX) -o joystick_test $(CXXFLAGS) $(LINK_FLAGS) joystick_test.cc

-include joystick_robot_test.d

joystick_robot_test: joystick_robot_test.cc joystick_robot_test.d
	$(CXX) joystick_robot_test.cc $(CXXFLAGS) $(LINK_FLAGS) -DI2C `pkg-config --libs --cflags opencv` -o joystick_robot_test

-include joystick_polling_test.d

joystick_polling_test: joystick_polling_test.cc joystick_polling_test.d
	$(CXX) -o joystick_polling_test $(CXXFLAGS) $(LINK_FLAGS) joystick_polling_test.cc

%.d: ;

clean:
	rm -f joystick_test joystick_robot_test joystick_polling_test *.d
