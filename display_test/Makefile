CXXFLAGS= -std=c++14 -g -Wall -Wpedantic -Wextra -MMD -MP -I.. #`pkg-config --cflags opencv`
LDFLAGS= -pthread `pkg-config --libs opencv`
# ACope - pkg-config for CV3 borked on bob-sim UNCOMMENT TO USE
LDFLAGS= -pthread -L/usr/lib/x86_64-linux-gnu -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio

.PHONY: all clean

all: display_test unwrap_display_test

display_test: display_test.cc display_test.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

-include display_test.d

unwrap_display_test: unwrap_display_test.cc unwrap_display_test.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

-include unwrap_display_test.d

%.d: ;

clean:
	rm -f display_test unwrap_display_test display_test.d unwrap_display_test.d
