EXECUTABLE      := ant_world
SOURCES         := ant_world.cc mb_memory.cc perfect_memory.cc
LINK_FLAGS      := -L../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW  `pkg-config --libs opencv`
CXXFLAGS        := -I../libantworld -pthread -Wall -Wpedantic -Wextra  `pkg-config --cflags opencv`
CPP_STANDARD    =c++14

ifdef RECORD_SPIKES
    CXXFLAGS += -DRECORD_SPIKES
endif

ifdef RECORD_TERMINAL_SYNAPSE_STATE
    CXXFLAGS += -DRECORD_TERMINAL_SYNAPSE_STATE
endif

include $(GENN_PATH)/userproject/include/makefile_common_gnu.mk