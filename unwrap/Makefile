TARGET := unwrapconfig

.PHONY: clean

$(TARGET): $(TARGET).cc ../common/opencv_unwrap_360.h
	$(CXX) -o $(TARGET) -g $(TARGET).cc `pkg-config --cflags --libs opencv`

clean:
	rm -f /unwrapconfig
