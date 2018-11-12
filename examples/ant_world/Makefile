WITH_LIBANTWORLD := 1
include ../../make_common/bob_robotics.mk

EXECUTABLE := ant_world_test

.PHONY: all clean libantworld

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d libantworld
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
