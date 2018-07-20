CXXFLAGS := -std=c++14 -Wall -Wpedantic -g -L../libantworld ../libantworld/agent.o -lantworld -lglfw \
		-lGL -lGLU -lGLEW -lm -lstdc++ -pthread `pkg-config --cflags --libs opencv`
TARGETS = ant_world_db_creator ant_world_map_creator

.PHONY: all clean libantworld

all: $(TARGETS)

clean:
	rm -f ant_world_db_creator ant_world_map_creator

ant_world_db_creator: ant_world_db_creator.cc libantworld
	$(CXX) -o $@ ant_world_db_creator.cc $(CXXFLAGS)

ant_world_map_creator: ant_world_map_creator.cc libantworld
	$(CXX) -o $@ ant_world_map_creator.cc $(CXXFLAGS)

libantworld:
	$(MAKE) -C ../libantworld