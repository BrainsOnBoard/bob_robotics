WITH_LIBANTWORLD := 1
WITH_MATPLOTLIBCPP := 1
include ../../make_common/flags.mk

EXECUTABLE := ant_world_test perfect_memory_antworld_test plot_ridf

.PHONY: all clean libantworld

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): %: %.cc %d libantworld
	$(CXX) -o $@ $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld

%.d: ;

clean:
	rm -f $(EXECUTABLE) *.d
