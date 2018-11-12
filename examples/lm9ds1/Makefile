NO_OPENCV := 1
WITH_I2C := 1
include ../../make_common/bob_robotics.mk

EXECUTABLE := lm9ds1_test

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS) -o $(EXECUTABLE)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
