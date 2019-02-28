WITH_EIGEN := 1
include ../../make_common/bob_robotics.mk
CXXFLAGS += `pkg-config --cflags sfml-graphics`
LINK_FLAGS += `pkg-config --libs sfml-graphics`

EXECUTABLE := collision

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
