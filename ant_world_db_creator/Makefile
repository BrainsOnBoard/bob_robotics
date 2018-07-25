include ../common/flags.mk
LINK_FLAGS += -L../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW

TARGETS = ant_world_db_creator ant_world_map_creator

.PHONY: all clean libantworld

all: $(TARGETS)

clean:
	rm -f ant_world_db_creator ant_world_map_creator

ant_world_db_creator: libantworld
	$(CXX) -o $@ ant_world_db_creator.cc $(CXXFLAGS) $(LINK_FLAGS)

-include ant_world_db_creator.d

ant_world_map_creator: libantworld
	$(CXX) -o $@ ant_world_map_creator.cc $(CXXFLAGS) $(LINK_FLAGS)

-include -ant_world_map_creator

libantworld:
	$(MAKE) -C ../libantworld