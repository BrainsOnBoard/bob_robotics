include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

all: camera_recorder camera_recorder_vicon

-include camera_recorder.d

camera_recorder: camera_recorder.cc camera_recorder.d
	$(CXX) camera_recorder.cc $(CXXFLAGS) -DI2C $(LINK_FLAGS) -o camera_recorder

camera_recorder_vicon: camera_recorder.cc camera_recorder.d
	$(CXX) camera_recorder.cc $(CXXFLAGS) -DVICON_CAPTURE $(LINK_FLAGS) -o camera_recorder_vicon_tracking

%.d: ;

clean:
	rm -rf camera_recorder camera_recorder_vicon camera_recorder.d