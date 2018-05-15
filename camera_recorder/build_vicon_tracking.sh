g++ camera_recorder.cc -std=c++11 -pthread -DVICON_CAPTURE `pkg-config --libs --cflags opencv` -o camera_recorder_vicon_tracking
