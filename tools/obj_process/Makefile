NO_OPENCV := 1
include ../../make_common/bob_robotics.mk

EXECUTABLE := obj_process

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) $(EXECUTABLE).cc $(CXXFLAGS) -O3 -o $(EXECUTABLE)

%.d: ;

clean:
	rm -rf $(EXECUTABLE) $(EXECUTABLE).d