WITH_LIBSERIAL=1
include ../../make_common/bob_robotics.mk

.PHONY: all

all: radar

radar: radar.cc
	$(CXX) $(CXXFLAGS)  radar.cc -o radar $(LINK_FLAGS)
