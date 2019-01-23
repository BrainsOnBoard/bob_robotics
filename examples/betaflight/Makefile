LIBMSPFLIGHT_PATH := ../../third_party/msp/build/

NO_OPENCV = ''

include ../../make_common/bob_robotics.mk
CXXFLAGS += -I$(LIBMSPFLIGHT_PATH)../inc \
			-DENABLE_PREDEFINED_CURRENT_UNITS -DENABLE_PREDEFINED_VOLTAGE_UNITS
LINK_FLAGS := -L$(LIBMSPFLIGHT_PATH) -lmsp -lmsp_fcu -lmspclient -lpthread \
			  -Wl,-rpath,$(LIBMSPFLIGHT_PATH),--disable-new-dtags

EXECUTABLE := betaflight_test

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
