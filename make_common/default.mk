# This makefile is for including in projects where you just want to e.g. build
# all the *.cc files in the current folder.
#
# include (path to BoB robotics)/make_common/bob_robotics.mk
include $(abspath $(dir $(lastword $(MAKEFILE_LIST))))/bob_robotics.mk

SOURCES     ?= $(wildcard *.cc)
EXECUTABLES := $(foreach exec,$(basename $(SOURCES)),$(OBJECT_PATH)$(exec))
DEPS        := $(foreach dep,$(basename $(SOURCES)),$(OBJECT_PATH)$(dep).d)

.PHONY: all clean $(EXTRA_DEPS)

all: $(EXECUTABLES)

-include $(DEPS)

$(EXECUTABLES): %: %.cc $(DEPS) $(EXTRA_DEPS)
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld

libbebop:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libbebop

imgui:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/third_party/imgui

# It's a header-only library, but make sure git submodule is checked out
plog:
	@git submodule update --init $(BOB_ROBOTICS_ROOT)/third_party/plog

%d: ;

clean:
	rm -f $(EXECUTABLES) $(DEPS)
