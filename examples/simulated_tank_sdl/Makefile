NO_OPENCV := 1
WITH_MATPLOTLIBCPP := 1
include ../../make_common/bob_robotics.mk
CXXFLAGS += `pkg-config --libs --cflags sdl2`

EXECUTABLE := simulated_tank_sdl

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
