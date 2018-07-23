include ../common/flags.mk
LINK_FLAGS += -L../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW

TARGETS = ant_world_db_creator ant_world_map_creator

.PHONY: all clean libantworld

all: $(TARGETS)

clean:
	rm -f ant_world_db_creator ant_world_map_creator

ant_world_db_creator: ant_world_db_creator.cc libantworld
	$(CXX) -o $@ ant_world_db_creator.cc $(CXXFLAGS) $(LINK_FLAGS)

ant_world_map_creator: ant_world_map_creator.cc libantworld
	$(CXX) -o $@ ant_world_map_creator.cc $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C ../libantworld