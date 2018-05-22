TARGET=simpledisplay_test
CXXFLAGS=-std=c++11 -g -I../.. `pkg-config --cflags opencv`
LDFLAGS=-pthread `pkg-config --libs opencv`

.PHONY: all clean

$(TARGET): simpledisplay/test.cc ../simpledisplay.h
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

all: $(TARGET)

clean:
	rm -f ./$(TARGET)
