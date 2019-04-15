WITH_LIBANTWORLD := 1
WITH_IMGUI := 1
include ../../make_common/bob_robotics.mk

GENERATED_CODE_DIR	:=ardin_mb_CODE
CXXFLAGS 		+=-I../.. `pkg-config --cflags eigen3`

ifdef RECORD_SPIKES
    CXXFLAGS += -DRECORD_SPIKES
endif

ifdef RECORD_TERMINAL_SYNAPSE_STATE
    CXXFLAGS += -DRECORD_TERMINAL_SYNAPSE_STATE
endif

.PHONY: all clean generated_code

all: ardin_mb

ardin_mb: ardin_mb.cc state_handler.cc mb_memory_hog.cc vector_field.cc visual_navigation_ui.cc opencv_texture.cc generated_code
	$(CXX) $(CXXFLAGS)  ardin_mb.cc state_handler.cc mb_memory_hog.cc vector_field.cc visual_navigation_ui.cc opencv_texture.cc -o ardin_mb -L$(GENERATED_CODE_DIR) -lrunner -Wl,-rpath=$(GENERATED_CODE_DIR)

generated_code:
	$(MAKE) -C $(GENERATED_CODE_DIR)
