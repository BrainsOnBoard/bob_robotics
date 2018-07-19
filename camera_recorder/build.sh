g++ camera_recorder.cc -std=c++14 -pthread -DI2C `pkg-config --libs --cflags opencv` -o camera_recorder
