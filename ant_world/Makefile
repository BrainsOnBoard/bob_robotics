include ../common/flags.mk

EXECUTABLE      := ant_world
SOURCES         := ant_world.cc mb_memory.cc perfect_memory.cc
LINK_FLAGS      += -L../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW
CXXFLAGS        += -I../libantworld

ifdef RECORD_SPIKES
    CXXFLAGS += -DRECORD_SPIKES
endif

ifdef RECORD_TERMINAL_SYNAPSE_STATE
    CXXFLAGS += -DRECORD_TERMINAL_SYNAPSE_STATE
endif

include $(GENN_PATH)/userproject/include/makefile_common_gnu.mk