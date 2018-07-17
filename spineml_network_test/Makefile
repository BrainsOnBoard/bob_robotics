CXXFLAGS= -std=c++14 -g -Wall -Wpedantic -Wextra -MMD -MP -I.. #`pkg-config --cflags opencv`
LDFLAGS= -pthread `pkg-config --libs opencv`
# ACope - pkg-config for CV3 borked on bob-sim UNCOMMENT TO USE
LDFLAGS= -pthread -L/usr/lib/x86_64-linux-gnu -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio

.PHONY: all clean

all: spineml_network_test

spineml_network_test: spineml_network_test.cc spineml_network_test.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

-include spineml_network_test.d

%.d: ;

clean:
	rm -f spineml_network_test spineml_network_test.d
