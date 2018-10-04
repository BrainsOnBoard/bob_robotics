include ../../common/flags.mk

EXECUTABLE      := ardin_mb
SOURCES         := ardin_mb.cc state_handler.cc mb_memory.cc
LINK_FLAGS      := -L../../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW $(LINK_FLAGS)
CXXFLAGS        += -I../.. `pkg-config --cflags eigen3`

ifdef RECORD_SPIKES
    CXXFLAGS += -DRECORD_SPIKES
endif

ifdef RECORD_TERMINAL_SYNAPSE_STATE
    CXXFLAGS += -DRECORD_TERMINAL_SYNAPSE_STATE
endif

include $(GENN_PATH)/userproject/include/makefile_common_gnu.mk
