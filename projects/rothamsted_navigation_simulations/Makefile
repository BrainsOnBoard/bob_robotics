WITH_LIBANTWORLD := 1
include ../../make_common/bob_robotics.mk

.PHONY: all clean libantworld

all: perfect_memory save_route

-include perfect_memory.d

perfect_memory: perfect_memory.cc perfect_memory.d libantworld
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include save_route.d

save_route: save_route.cc save_route.d libantworld
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld

%d: ;

clean:
	rm -f perfect_memory save_route *.d
