TARGET=simpledisplay_test
CXXFLAGS=-std=c++11 -g `pkg-config --cflags opencv`
LDFLAGS=-pthread `pkg-config --libs opencv`

.PHONY: clean

$(TARGET): $(TARGET).cc simpledisplay.h
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

clean:
	rm -f ./$(TARGET)