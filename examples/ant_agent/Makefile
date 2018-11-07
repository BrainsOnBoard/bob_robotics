WITH_LIBANTWORLD := 1
include ../../make_common/bob_robotics.mk

EXECUTABLE := ant_agent

.PHONY: all clean libantworld

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld
