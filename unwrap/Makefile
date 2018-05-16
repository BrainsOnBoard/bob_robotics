TARGET := unwrapconfig

.PHONY: clean all install

$(TARGET): $(TARGET).cc ../common/opencv_unwrap_360.h
	$(CXX) -o $(TARGET) -g -I.. $(TARGET).cc `pkg-config --cflags --libs opencv`

clean:
	rm -f unwrapconfig

all: $(TARGET)

install: $(TARGET)
	install -Dm 755 unwrapconfig /usr/local/bin
