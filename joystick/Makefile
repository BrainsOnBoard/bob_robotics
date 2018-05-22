.PHONY: clean

TARGET := joystick_test

$(TARGET): $(TARGET).cc joystick.h joystick_linux.h joystick_base.h
	$(CXX) -o $(TARGET) -std=c++11 -pthread $(TARGET).cc

clean:
	rm -f $(TARGET)
