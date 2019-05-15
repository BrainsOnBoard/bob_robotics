WITH_LIBANTWORLD := 1
WITH_IMGUI := 1
include ../../make_common/bob_robotics.mk

SOURCES			:=ardin_mb.cc state_handler.cc mb_memory_hog.cc vector_field.cc visual_navigation_ui.cc opencv_texture.cc
GENERATED_CODE_DIR	:=mb_memory_hog_CODE
CXXFLAGS 		+=-I../.. `pkg-config --cflags eigen3` -DNO_HEADER_DEFINITIONS
LINK_FLAGS		+=-ldl

OBJECTS			:=$(SOURCES:.cc=.o)
DEPS			:=$(SOURCES:.cc=.d)

ifdef RECORD_SPIKES
    CXXFLAGS += -DRECORD_SPIKES
endif

ifdef RECORD_TERMINAL_SYNAPSE_STATE
    CXXFLAGS += -DRECORD_TERMINAL_SYNAPSE_STATE
endif

.PHONY: all clean generated_code libantworld

all: ardin_mb

ardin_mb: $(OBJECTS) generated_code libantworld
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o ardin_mb $(LINK_FLAGS)

-include $(DEPS)

%.o: %.cc %.d
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.d: ;

generated_code:
	$(MAKE) -C $(GENERATED_CODE_DIR)

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld

clean:
	@rm -f ardin_mb *.o *.d
