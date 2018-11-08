WITH_MATPLOTLIBCPP := 1
include ../../make_common/bob_robotics.mk

VICON_DATASTREAM_SDK_ROOT := $(BOB_ROBOTICS_ROOT)/third_party/vicon_data_stream_sdk
CXXFLAGS += -I$(VICON_DATASTREAM_SDK_ROOT)/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP
LINK_FLAGS += -L$(VICON_DATASTREAM_SDK_ROOT)/bin/Release \
		-lViconDataStreamSDK_CPP \
		-Wl,-rpath,$(VICON_DATASTREAM_SDK_ROOT)/bin/Release,--disable-new-dtags

EXECUTABLE := vicon_arena_constructor

.PHONY: clean vicon_data_stream_sdk

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d vicon_data_stream_sdk
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

vicon_data_stream_sdk:
	@git submodule update --init $(VICON_DATASTREAM_SDK_ROOT)
	CONFIG=Release $(MAKE) -C $(VICON_DATASTREAM_SDK_ROOT)

%.d: ;

clean:
	rm -f $(EXECUTABLE) *.d
