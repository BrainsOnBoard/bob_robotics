include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all

all: camera_recorder camera_recorder_vicon

camera_recorder: camera_recorder.cc
	$(CXX) camera_recorder.cc $(CXXFLAGS) -DI2C $(LINK_FLAGS) -o camera_recorder

camera_recorder_vicon: camera_recorder.cc
	$(CXX) camera_recorder.cc $(CXXFLAGS) -DVICON_CAPTURE $(LINK_FLAGS) -o camera_recorder_vicon_tracking

-include camera_recorder.d