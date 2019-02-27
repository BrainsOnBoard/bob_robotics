NO_OPENCV := 1
include ../../make_common/bob_robotics.mk
CXXFLAGS += `pkg-config --cflags sdl2`
LINK_FLAGS += `pkg-config --libs sdl2`

EXECUTABLE := simulated_positioner

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
