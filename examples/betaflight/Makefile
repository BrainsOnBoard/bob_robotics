LIBMSPFLIGHT_PATH := ../../third_party/msp/build/

NO_OPENCV = ''

include ../../make_common/bob_robotics.mk
CXXFLAGS += -I$(LIBMSPFLIGHT_PATH)../inc
LINK_FLAGS := -L$(LIBMSPFLIGHT_PATH) -lmsp -lmsp_fcu -lmspclient -lpthread

EXECUTABLE := betaflight_test

.PHONY: all clean betaflight_test

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
