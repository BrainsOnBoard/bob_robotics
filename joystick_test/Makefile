.PHONY: all clean

CXXFLAGS=-std=c++14 -g -Wall -Wpedantic -Wextra -MMD -MP -pthread -I..

all: joystick_test joystick_robot_test joystick_omnirobot_test joystick_polling_test

joystick_test: joystick_test.cc joystick_test.d
	$(CXX) -o joystick_test $(CXXFLAGS) joystick_test.cc

-include joystick_test.d

joystick_robot_test: joystick_robot_test.cc joystick_robot_test.d
	$(CXX) joystick_robot_test.cc $(CXXFLAGS) -DI2C `pkg-config --libs --cflags opencv` -o joystick_robot_test

-include joystick_robot_test.d

joystick_omnirobot_test: joystick_omnirobot_test.cc joystick_omnirobot_test.d
	$(CXX) joystick_omnirobot_test.cc $(CXXFLAGS) -DI2C `pkg-config --libs --cflags opencv` -o joystick_omnirobot_test

-include joystick_robot_test.d

joystick_polling_test: joystick_polling_test.cc joystick_polling_test.d
	$(CXX) -o joystick_polling_test $(CXXFLAGS) joystick_polling_test.cc

-include joystick_polling_test.d

%.d: ;

clean:
	rm -f joystick_test joystick_robot_test joystick_polling_test *.d
