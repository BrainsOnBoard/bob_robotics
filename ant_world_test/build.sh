pushd ../libantworld
make
popd
g++ ant_world_test.cc -std=c++14 -Wall -Wpedantic -L../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW -pthread `pkg-config --cflags --libs opencv` -o ant_world_test