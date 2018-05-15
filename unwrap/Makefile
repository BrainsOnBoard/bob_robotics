TARGET := unwrapconfig

.PHONY: clean install headers

$(TARGET): $(TARGET).cc ../common/opencv_unwrap_360.h
	$(CXX) -o $(TARGET) -g -I.. $(TARGET).cc `pkg-config --cflags --libs opencv`

clean:
	rm -f unwrapconfig

install: $(TARGET)
	install -Dm 755 unwrapconfig /usr/local/bin

headers:
	sh defaultparams/yaml2h.sh