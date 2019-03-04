# This makefile is for including in projects where you just want to e.g. build
# all the *.cc files in the current folder.
#
# include (path to BoB robotics)/make_common/bob_robotics.mk
include $(abspath $(dir $(lastword $(MAKEFILE_LIST))))/bob_robotics.mk

SOURCES ?= $(wildcard *.cc)
OBJECTS := $(foreach obj,$(basename $(SOURCES)),$(OBJECT_PATH)$(obj))
DEPS    := $(foreach dep,$(basename $(SOURCES)),$(OBJECT_PATH)$(dep).d)

.PHONY: all clean $(EXTRA_DEPS)

all: $(OBJECTS)

-include $(DEPS)

$(OBJECTS): %: %.cc $(DEPS) $(EXTRA_DEPS)
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld

libbebop:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libbebop

imgui:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/third_party/imgui

%d: ;

clean:
	rm -f $(OBJECTS) $(DEPS)
