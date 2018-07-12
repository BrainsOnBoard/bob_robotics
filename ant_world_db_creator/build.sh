pushd ../libantworld
make
popd
g++ ant_world_db_creator.cc -std=c++14 -Wall -Wpedantic -g -L../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW -pthread `pkg-config --cflags --libs opencv` -o ant_world_db_creator
g++ ant_world_map_creator.cc -std=c++14 -Wall -Wpedantic -g -L../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW -pthread `pkg-config --cflags --libs opencv` -o ant_world_map_creator