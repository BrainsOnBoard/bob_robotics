WITH_MATPLOTLIBCPP := 1
include ../../common/flags.mk
CXXFLAGS += -I../.. -I$(VICON_DATASTREAM_SDK_ROOT)/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP
LINK_FLAGS += -L$(VICON_DATASTREAM_SDK_ROOT)/bin/Release \
		-lViconDataStreamSDK_CPP \
		-lboost_wserialization -lboost_wave -lboost_unit_test_framework -lboost_timer -lboost_thread -lboost_signals -lboost_serialization -lboost_regex -lboost_random -lboost_python3 -lboost_program_options -lboost_prg_exec_monitor -lboost_math_tr1l -lboost_math_tr1f -lboost_math_tr1 -lboost_math_c99l -lboost_math_c99f -lboost_math_c99 -lboost_log_setup -lboost_log -lboost_locale -lboost_iostreams -lboost_graph -lboost_filesystem -lboost_date_time -lboost_coroutine -lboost_context -lboost_container -lboost_chrono -lboost_atomic -lboost_system \
		-Wl,-rpath,$(VICON_DATASTREAM_SDK_ROOT)/bin/Release,--disable-new-dtags

EXECUTABLE := vicon_arena_constructor

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm $(EXECUTABLE) *.d`
