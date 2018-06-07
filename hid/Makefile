.PHONY: all clean

TARGET := joystick_test

all: $(TARGET)

$(TARGET): $(TARGET).cc joystick.h joystick_linux.h joystick_base.h
	$(CXX) -o $(TARGET) -g -std=c++11 -lstdc++ -pthread $(TARGET).cc

clean:
	rm -f $(TARGET)
