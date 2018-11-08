NO_OPENCV := 1
include ../../common/flags.mk
CXXFLAGS += -I../.. `pkg-config --cflags sdl2`
LINK_FLAGS += `pkg-config --libs sdl2`

EXECUTABLE := robot_simulator

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
