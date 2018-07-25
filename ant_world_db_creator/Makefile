include ../common/flags.mk
CXXFLAGS += -I..

LIBANTWORLD_PATH := ../libantworld
LINK_FLAGS += -L$(LIBANTWORLD_PATH) -lantworld -lglfw -lGL -lGLU -lGLEW
LIBANTWORLD := $(LiBANTWORLD_PATH)/libantworld.a

TARGETS = ant_world_db_creator ant_world_map_creator

.PHONY: all clean

all: $(TARGETS)

clean:
	rm -f $(TARGETS) *.d

-include ant_world_db_creator.d

ant_world_db_creator: ant_world_db_creator.cc ant_world_db_creator.d $(LIBANTWORLD)
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include -ant_world_map_creator

ant_world_map_creator: ant_world_map_creator.cc ant_world_map_creator.d $(LIBANTWORLD)
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

$.d: ;

$(LIBANTWORLD):
	$(MAKE) -C $(LIBANTWORLD_PATH)
