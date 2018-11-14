NO_OPENCV := 1
WITH_I2C := 1
include ../../make_common/bob_robotics.mk

# Ignore spurious gcc warning
CXXFLAGS += -Wno-maybe-uninitialized

EXECUTABLE := robot_positioner

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
