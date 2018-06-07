.PHONY: all clean

TARGET := joystick_test

all: joystick_test joystick_robot_test

joystick_test: joystick_test.cc
	$(CXX) -o joystick_test -g -std=c++11 -I.. -lstdc++ -pthread joystick_test.cc

joystick_robot_test:
	$(CXX) joystick_robot_test.cc -std=c++11 -DI2C `pkg-config --libs --cflags opencv` -o joystick_robot_test

clean:
	rm -f joystick{,_robot}_test
