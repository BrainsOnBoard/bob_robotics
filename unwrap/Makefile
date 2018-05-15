TARGET := unwrapconfig

.PHONY: clean headers

$(TARGET): $(TARGET).cc ../common/opencv_unwrap_360.h
	$(CXX) -o $(TARGET) -g -I.. $(TARGET).cc `pkg-config --cflags --libs opencv`

headers:
	sh defaultparams/yaml2h.sh

clean:
	rm -f unwrapconfig
