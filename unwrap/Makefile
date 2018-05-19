TARGET := unwrapconfig

.PHONY: clean all install

$(TARGET): $(TARGET)_src/$(TARGET).cc ../common/opencv_unwrap_360.h
	$(CXX) -o $(TARGET) -g -std=c++11 -I.. $(TARGET)_src/$(TARGET).cc `pkg-config --cflags --libs opencv`

clean:
	rm -f unwrapconfig

all: $(TARGET)

install: $(TARGET)
	install -Dm 755 unwrapconfig /usr/local/bin
