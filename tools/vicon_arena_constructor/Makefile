WITH_MATPLOTLIBCPP := 1
include ../../common/flags.mk
CXXFLAGS += -I../.. -I$(VICON_DATASTREAM_SDK_ROOT)/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP
LINK_FLAGS += -L$(VICON_DATASTREAM_SDK_ROOT)/bin/Release \
		-lViconDataStreamSDK_CPP \
		-Wl,-rpath,$(VICON_DATASTREAM_SDK_ROOT)/bin/Release,--disable-new-dtags

EXECUTABLE := vicon_arena_constructor

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) *.d
