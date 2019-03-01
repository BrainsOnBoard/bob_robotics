LIB_PATH := ../../third_party/ev3dev-lang-cpp

NO_OPENCV := 1
CXX := arm-linux-gnueabi-g++
CC := arm-linux-gnueabi-g++
ARCH := armv5
include ../../make_common/bob_robotics.mk
CXXFLAGS += -I$(LIB_PATH)/ev3dev-lang-cpp
LINK_FLAGS += -L$(LIB_PATH)/build -lev3dev

EXECUTABLE := ev3_control

.PHONY: all clean strip ev3dev-lang-cpp

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d ev3dev-lang-cpp
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

# It's really slow to copy over Bluetooth, but you can make the file size much
# smaller by stripping debug symbols
strip: $(EXECUTABLE)
	arm-linux-gnueabi-strip $(EXECUTABLE)

ev3dev-lang-cpp:
	$(MAKE) -C $(LIB_PATH)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
